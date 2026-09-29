"""Sensor-driven flight state machine for the GARUDA payload."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import time

import config
from core.mission_state import MissionState, can_transition
from core.shared_data import PayloadSnapshot, SharedData

logger = logging.getLogger(__name__)


@dataclass
class FlightStateController:
    """Apply rocket ascent, apogee, glider deployment, and landing transitions."""

    shared: SharedData
    current_state: MissionState = MissionState.BOOT
    previous_altitude_m: float | None = None
    previous_time_s: float | None = None
    state_entry_time_s: float = field(default_factory=time.time)
    launch_time_s: float | None = None
    apogee_time_s: float | None = None
    glider_deploy_time_s: float | None = None
    landing_check_start_s: float | None = None
    max_altitude_m: float = 0.0
    ground_altitude_m: float | None = None
    confirmation_target: MissionState | None = None
    confirmation_count: int = 0

    def start(self) -> None:
        """Initialize the mission in DISARMED, or ARMED_PAD in mock auto-arm mode."""
        self.shared.start_mission_clock()
        self._transition(MissionState.DISARMED, reason="controller_start", status="DISARMED")
        if config.USE_MOCK_HARDWARE and config.AUTO_ARM_IN_MOCK_MODE:
            self.arm()

    def arm(self) -> None:
        """Arm the flight controller on the pad."""
        snap = self.shared.get_snapshot()
        if not snap.barometer_ok or snap.baro_timestamp_ns <= 0:
            raise RuntimeError("Cannot arm without a valid barometer sample")
        self.ground_altitude_m = snap.baro_altitude
        self.previous_altitude_m = 0.0
        self.previous_time_s = time.time()
        self.max_altitude_m = 0.0
        self._transition(
            MissionState.ARMED_PAD,
            reason="operator_arm",
            status="ARMED",
            flight_armed=True,
            guidance_requested=False,
            actuation_enabled=False,
            actuation_inhibit_reason="WAITING_FOR_GUIDED_DESCENT",
        )

    def disarm(self) -> None:
        """Return from ARMED_PAD to DISARMED before launch."""
        self._transition(
            MissionState.DISARMED,
            reason="operator_disarm",
            status="DISARMED",
            flight_armed=False,
            guidance_requested=False,
            actuation_enabled=False,
            actuation_inhibit_reason="DISARMED",
        )

    def abort(self) -> None:
        """Enter abort state without directly firing deployment hardware."""
        self._transition(
            MissionState.ABORT,
            reason="abort_requested",
            status="ABORT",
            guidance_requested=False,
            actuation_enabled=False,
            actuation_inhibit_reason="ABORT",
        )

    def update(self) -> MissionState:
        """Evaluate one state-machine tick from the latest shared snapshot."""
        snap = self.shared.get_snapshot()
        now = time.time()
        # Flight thresholds are AGL.  Capturing the pad pressure altitude at
        # arm prevents a site elevation (for example Pune MSL altitude) from
        # being misinterpreted as launch altitude.
        baseline = snap.baro_altitude if self.ground_altitude_m is None else self.ground_altitude_m
        altitude_m = max(0.0, snap.baro_altitude - baseline)
        velocity_mps = self._vertical_velocity(snap, altitude_m, now)
        accel_g = self._accel_magnitude_g(snap)
        self.max_altitude_m = max(self.max_altitude_m, altitude_m)
        self.current_state = MissionState(snap.state)

        self.shared.update(
            vertical_velocity=velocity_mps,
            max_altitude=self.max_altitude_m,
        )

        if self.launch_time_s is not None and now - self.launch_time_s > config.MAX_FLIGHT_TIME_SEC:
            logger.warning("Max flight time exceeded; entering ABORT.")
            self.abort()
            return MissionState.ABORT

        if self.current_state == MissionState.DISARMED:
            return self.current_state

        if self.current_state == MissionState.ARMED_PAD:
            launch_detected = (
                accel_g > config.LAUNCH_DETECT_ACCEL_G
                or altitude_m > config.LAUNCH_DETECT_ALTITUDE_AGL_M
            )
            if self._confirmed(MissionState.BOOST, launch_detected, required_count=3):
                self.launch_time_s = now
                self.max_altitude_m = altitude_m
                self._transition(
                    MissionState.BOOST,
                    reason="launch_detected",
                    trigger_values={"accel_g": accel_g, "baro_agl_m": altitude_m},
                    launch_detected=True,
                    status="LAUNCH_DETECTED",
                )

        elif self.current_state == MissionState.BOOST:
            burnout = accel_g < config.BOOST_BURNOUT_ACCEL_G
            timed_out = self.launch_time_s is not None and now - self.launch_time_s > config.BOOST_MAX_DURATION_SEC
            if self._confirmed(MissionState.COAST, burnout, required_count=5) or timed_out:
                self._transition(
                    MissionState.COAST,
                    reason="boost_burnout" if burnout else "boost_timeout",
                    trigger_values={"accel_g": accel_g, "timed_out": timed_out},
                    status="COASTING_TO_APOGEE",
                )

        elif self.current_state == MissionState.COAST:
            descending = velocity_mps < config.APOGEE_DESCENT_VELOCITY_MPS
            altitude_dropped = altitude_m < self.max_altitude_m - config.APOGEE_ALTITUDE_DROP_M
            high_enough = altitude_m > config.APOGEE_MIN_ALTITUDE_AGL_M
            backup_apogee = (
                self.launch_time_s is not None
                and now - self.launch_time_s > config.APOGEE_BACKUP_TIME_SEC
            )
            if self._confirmed(MissionState.APOGEE, descending and altitude_dropped and high_enough) or backup_apogee:
                self.apogee_time_s = now
                self._transition(
                    MissionState.APOGEE,
                    reason="apogee_detected" if not backup_apogee else "apogee_backup_timeout",
                    trigger_values={
                        "baro_agl_m": altitude_m,
                        "vertical_velocity_mps": velocity_mps,
                        "max_altitude_m": self.max_altitude_m,
                    },
                    apogee_detected=True,
                    payload_ejected=True,
                    status="PAYLOAD_EJECTED_AT_APOGEE",
                )

        elif self.current_state == MissionState.APOGEE:
            if self.apogee_time_s is None:
                self.apogee_time_s = now
            if now - self.apogee_time_s >= config.GLIDER_DEPLOY_SETTLE_SEC:
                self._transition(
                    MissionState.DESCENT_DROGUE,
                    reason="apogee_settle_elapsed",
                    status="DESCENT_AFTER_EJECTION",
                )

        elif self.current_state in {MissionState.DESCENT, MissionState.DESCENT_DROGUE}:
            deploy_glider = (
                altitude_m <= config.GLIDER_DEPLOY_ALTITUDE_AGL_M
                and velocity_mps < 0.0
            )
            if self._confirmed(
                MissionState.GLIDER_DEPLOY,
                deploy_glider,
                required_count=config.GLIDER_DEPLOY_CONFIRMATION_COUNT,
            ):
                self.glider_deploy_time_s = now
                self._transition(
                    MissionState.GLIDER_DEPLOY,
                    reason="glider_deploy_altitude_reached",
                    trigger_values={
                        "baro_agl_m": altitude_m,
                        "vertical_velocity_mps": velocity_mps,
                        "threshold_m": config.GLIDER_DEPLOY_ALTITUDE_AGL_M,
                    },
                    glider_deployed=True,
                    status="GLIDER_DEPLOYED_600M_AGL",
                )

        elif self.current_state == MissionState.GLIDER_DEPLOY:
            if self.glider_deploy_time_s is None:
                self.glider_deploy_time_s = now
            if now - self.glider_deploy_time_s >= config.GLIDER_DEPLOY_SETTLE_SEC:
                self._transition(
                    MissionState.GUIDED_DESCENT,
                    reason="glider_deploy_settle_elapsed",
                    glider_deployed=True,
                    guidance_requested=True,
                    actuation_enabled=False,
                    actuation_inhibit_reason="AWAITING_SAFETY_GATE",
                    status="GUIDANCE_REQUESTED",
                )

        elif self.current_state == MissionState.GUIDED_DESCENT:
            self._update_landing_detection(altitude_m, velocity_mps, accel_g, now)

        elif self.current_state == MissionState.ABORT:
            self._update_landing_detection(altitude_m, velocity_mps, accel_g, now)

        return MissionState(self.shared.get_snapshot().state)

    def _vertical_velocity(
        self,
        snap: PayloadSnapshot,
        altitude_m: float,
        now_s: float,
    ) -> float:
        if self.previous_altitude_m is None or self.previous_time_s is None:
            velocity = snap.vertical_velocity
        else:
            dt = max(now_s - self.previous_time_s, 1e-3)
            velocity = (altitude_m - self.previous_altitude_m) / dt
        self.previous_altitude_m = altitude_m
        self.previous_time_s = now_s
        return velocity

    def _accel_magnitude_g(self, snap: PayloadSnapshot) -> float:
        accel_norm = math.sqrt(
            snap.raw_accel_x**2 + snap.raw_accel_y**2 + snap.raw_accel_z**2
        )
        if accel_norm <= 0.0:
            return 1.0
        return accel_norm / 9.80665

    def _confirmed(
        self,
        target: MissionState,
        condition: bool,
        required_count: int | None = None,
    ) -> bool:
        required_count = required_count or config.STATE_CONFIRMATION_COUNT
        if condition:
            if self.confirmation_target != target:
                self.confirmation_target = target
                self.confirmation_count = 0
            self.confirmation_count += 1
            if self.confirmation_count >= required_count:
                self.confirmation_target = None
                self.confirmation_count = 0
                return True
        else:
            if self.confirmation_target == target:
                self.confirmation_target = None
                self.confirmation_count = 0
        return False

    def _transition(
        self,
        target: MissionState,
        reason: str = "sensor_condition",
        trigger_values: dict | None = None,
        **updates,
    ) -> None:
        current = MissionState(self.shared.get_snapshot().state)
        if current == target:
            self.shared.update(**updates)
            return
        if not can_transition(current, target):
            logger.warning("Blocked invalid state transition: %s -> %s", current, target)
            self.shared.record_event(
                "STATE_CHANGE_BLOCKED",
                "FSM",
                "WARN",
                f"Blocked invalid transition {current.value} -> {target.value}.",
                {"reason": reason},
            )
            return
        self.state_entry_time_s = time.time()
        self.confirmation_target = None
        self.confirmation_count = 0
        self.current_state = target
        self.shared.transition_state(
            target,
            reason=reason,
            source="FSM",
            trigger_values=trigger_values,
            **updates,
        )
        logger.info("Mission state: %s -> %s", current.value, target.value)

    def _update_landing_detection(
        self,
        altitude_m: float,
        velocity_mps: float,
        accel_g: float,
        now_s: float,
    ) -> None:
        low_altitude = altitude_m < config.LANDING_DETECT_ALTITUDE_AGL_M
        low_velocity = abs(velocity_mps) < config.LANDING_DETECT_VELOCITY_MPS
        accel_near_1g = 0.8 <= accel_g <= 1.2
        if low_altitude and low_velocity and accel_near_1g:
            if self.landing_check_start_s is None:
                self.landing_check_start_s = now_s
            elif now_s - self.landing_check_start_s >= config.LANDING_DETECT_TIME_SEC:
                self._transition(
                    MissionState.LANDED,
                    reason="landing_detector_persistent",
                    trigger_values={
                        "baro_agl_m": altitude_m,
                        "vertical_velocity_mps": velocity_mps,
                        "accel_g": accel_g,
                        "persistence_sec": config.LANDING_DETECT_TIME_SEC,
                    },
                    actuation_enabled=False,
                    guidance_requested=False,
                    flight_armed=False,
                    actuation_inhibit_reason="LANDED",
                    status="LANDED",
                )
        else:
            self.landing_check_start_s = None
