"""
Thread-safe shared data store for all payload subsystems.

Every sensor thread writes here; logger, telemetry, and mapping read from here.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, fields
from datetime import datetime
from typing import Any

import config
from core.mission_state import MissionState
from sensor_fusion.ahrs import AttitudeState


def _wall_time() -> float:
    return time.time()


def _mono_time() -> float:
    return time.monotonic()


@dataclass
class SubsystemMetric:
    """Runtime health and timing evidence for one worker or subsystem."""

    name: str
    status: str = "INITIALIZING"
    reason: str = "No samples yet."
    expected_hz: float = 0.0
    actual_hz: float = 0.0
    last_update_monotonic: float = 0.0
    last_update_wall: float = 0.0
    iteration_count: int = 0
    error_count: int = 0
    consecutive_errors: int = 0
    last_error: str = ""
    last_error_time: float = 0.0
    recovery_count: int = 0
    restart_count: int = 0
    details: dict[str, Any] = field(default_factory=dict)
    _last_iteration_monotonic: float = 0.0

    def as_dict(self, now: float) -> dict[str, Any]:
        age_ms = None
        status = self.status
        reason = self.reason
        if self.last_update_monotonic > 0.0:
            age_ms = (now - self.last_update_monotonic) * 1000.0
            stale_timeout = float(
                self.details.get("stale_timeout_sec", config.WORKER_STALE_TIMEOUT_SEC)
            )
            if status in {"HEALTHY", "DEGRADED"} and now - self.last_update_monotonic > stale_timeout:
                status = "STALE"
                reason = f"No update for {now - self.last_update_monotonic:.2f}s."
        return {
            "name": self.name,
            "status": status,
            "reason": reason,
            "expected_hz": self.expected_hz,
            "actual_hz": self.actual_hz,
            "data_age_ms": age_ms,
            "last_update_wall": self.last_update_wall,
            "iteration_count": self.iteration_count,
            "error_count": self.error_count,
            "consecutive_errors": self.consecutive_errors,
            "last_error": self.last_error,
            "last_error_time": self.last_error_time,
            "recovery_count": self.recovery_count,
            "restart_count": self.restart_count,
            "details": dict(self.details),
        }


@dataclass
class PayloadSnapshot:
    """Point-in-time copy of all shared payload fields."""

    timestamp: float = 0.0
    mission_time: float = 0.0
    state: str = MissionState.BOOT.value
    latitude: float = 0.0
    longitude: float = 0.0
    gps_altitude: float = 0.0
    gps_ground_speed_mps: float = 0.0
    gps_course_deg: float = 0.0
    gps_satellites: int = 0
    gps_hdop: float = 0.0
    gps_fix_type: str = "NO FIX"
    gps_timestamp_ns: int = 0
    baro_altitude: float = 0.0
    baro_timestamp_ns: int = 0
    vertical_velocity: float = 0.0
    max_altitude: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    raw_gyro_x: float = 0.0
    raw_gyro_y: float = 0.0
    raw_gyro_z: float = 0.0
    raw_accel_x: float = 0.0
    raw_accel_y: float = 0.0
    raw_accel_z: float = 0.0
    raw_mag_x: float = 0.0
    raw_mag_y: float = 0.0
    raw_mag_z: float = 0.0
    raw_quat_w: float = 1.0
    raw_quat_x: float = 0.0
    raw_quat_y: float = 0.0
    raw_quat_z: float = 0.0
    raw_imu_timestamp_ns: int = 0
    raw_imu_accuracy_rad: float = 0.0
    raw_imu_calibration_status: int = -1
    raw_baro_pressure_hpa: float = 0.0
    raw_baro_temperature_c: float = 0.0
    gimbal_x_deflection_deg: float = 0.0
    gimbal_y_deflection_deg: float = 0.0
    gimbal_stepper_angle_deg: float = 0.0
    gimbal_servo_angle_deg: float = 90.0
    gimbal_stepper_steps: int = 0
    gimbal_ok: bool = False
    ahrs_enabled: bool = False
    ahrs_source: str = "OFF"
    ahrs_valid: bool = False
    ahrs_healthy: bool = False
    ahrs_confidence: str = "INVALID"
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    ahrs_roll: float = 0.0
    ahrs_pitch: float = 0.0
    ahrs_yaw: float = 0.0
    attitude_accuracy_rad: float = 0.0
    imu_sample_age_ms: float = 0.0
    accel_correction_active: bool = False
    mag_correction_active: bool = False
    ahrs_timestamp_ns: int = 0
    image_name: str = ""
    image_timestamp: float = 0.0
    launch_detected: bool = False
    apogee_detected: bool = False
    payload_ejected: bool = False
    glider_deployed: bool = False
    flight_armed: bool = False
    guidance_requested: bool = False
    actuation_enabled: bool = False
    actuation_inhibit_reason: str = "NOT_ARMED"
    servo_ok: bool = False
    gnc_ok: bool = False
    gnc_timestamp_ns: int = 0
    battery: float = 100.0
    status: str = "OK"
    previous_state: str = ""
    state_entry_timestamp: float = 0.0
    state_entry_mission_time: float = 0.0
    state_transition_reason: str = "runtime_initializing"
    test_mode: str = "FLIGHT"
    telemetry_sequence: int = 0
    telemetry_tx_count: int = 0
    telemetry_last_tx_timestamp: float = 0.0
    bus_voltage_v: float = 0.0
    current_a: float = 0.0
    power_w: float = 0.0
    min_voltage_v: float = 0.0
    max_current_a: float = 0.0
    undervoltage_events: int = 0
    logger_rows_written: int = 0
    logger_errors: int = 0
    logger_last_write_timestamp: float = 0.0
    camera_capture_sequence: int = 0
    camera_total_captures: int = 0
    camera_successful_captures: int = 0
    camera_failed_captures: int = 0
    camera_dropped_captures: int = 0
    camera_last_file_size_bytes: int = 0
    camera_last_write_latency_ms: float = 0.0
    image_sync_imu_delta_ms: float = -1.0
    image_sync_gps_delta_ms: float = -1.0
    image_sync_baro_delta_ms: float = -1.0
    image_quality_sharpness: float = 0.0
    image_quality_brightness: float = 0.0
    image_quality_underexposed_fraction: float = 0.0
    image_quality_overexposed_fraction: float = 0.0
    image_quality_status: str = "UNAVAILABLE"
    images_referenced: int = 0
    images_present: int = 0
    images_missing: int = 0
    images_orphan: int = 0
    camera_ok: bool = False
    gps_ok: bool = False
    imu_ok: bool = False
    barometer_ok: bool = False
    telemetry_ok: bool = False
    # GNC fields
    servo_left: float = 90.0
    servo_right: float = 90.0
    servo_drogue: float = 60.0
    heading_cmd: float = 0.0
    roll_cmd: float = 0.0
    pitch_cmd: float = 0.0

    # Navigation / Diagnostics
    nav_timestamp_ns: int = 0
    nav_origin_latitude: float = 0.0
    nav_origin_longitude: float = 0.0
    estimated_latitude: float = 0.0
    estimated_longitude: float = 0.0
    estimated_north_m: float = 0.0
    estimated_east_m: float = 0.0
    estimated_altitude_m: float = 0.0
    estimated_agl_m: float = 0.0
    estimated_velocity_north_mps: float = 0.0
    estimated_velocity_east_mps: float = 0.0
    estimated_ground_speed_mps: float = 0.0
    estimated_course_deg: float = 0.0
    estimated_heading_deg: float = 0.0
    navigation_mode: str = "INITIALIZING"
    position_quality: str = "INVALID"
    heading_quality: str = "INVALID"
    altitude_quality: str = "INVALID"
    position_source: str = "NONE"
    nav_gps_valid: bool = False
    nav_gps_rejected: bool = False
    nav_gps_rejection_reason: str = "NO_SAMPLE"
    nav_gps_age_ms: float = -1.0
    nav_gps_position_error_m: float = -1.0
    dead_reckoning_active: bool = False
    dead_reckoning_age_s: float = 0.0
    recovery_active: bool = False
    navigation_valid: bool = False
    safe_for_guidance: bool = False
    nav_consecutive_good_gps: int = 0
    nav_consecutive_bad_gps: int = 0
    estimator_reset_count: int = 0
    nav_last_event: str = ""


class SharedData:
    """Thread-safe container for live mission data."""

    CSV_HEADER = (
        "timestamp,mission_time,state,latitude,longitude,gps_altitude,"
        "baro_altitude,vertical_velocity,max_altitude,roll,pitch,yaw,"
        "gyro_x,gyro_y,gyro_z,image_name,"
        "image_timestamp,battery,status,ahrs_enabled,ahrs_source,ahrs_valid,"
        "ahrs_healthy,ahrs_confidence,quat_w,quat_x,quat_y,quat_z,ahrs_roll,"
        "ahrs_pitch,ahrs_yaw,attitude_accuracy_rad,imu_sample_age_ms,"
        "accel_correction_active,mag_correction_active,ahrs_timestamp_ns,"
        "raw_gyro_x,raw_gyro_y,raw_gyro_z,raw_accel_x,raw_accel_y,raw_accel_z,"
        "raw_mag_x,raw_mag_y,raw_mag_z,raw_quat_w,raw_quat_x,raw_quat_y,"
        "raw_quat_z,raw_imu_timestamp_ns,raw_imu_accuracy_rad,"
        "raw_imu_calibration_status,raw_baro_pressure_hpa,"
        "raw_baro_temperature_c,gimbal_x_deflection_deg,gimbal_y_deflection_deg,"
        "gimbal_stepper_angle_deg,gimbal_servo_angle_deg,gimbal_stepper_steps,"
        "gimbal_ok,launch_detected,apogee_detected,payload_ejected,"
        "glider_deployed,actuation_enabled,previous_state,state_transition_reason,"
        "telemetry_sequence,telemetry_tx_count,bus_voltage_v,current_a,power_w,"
        "min_voltage_v,max_current_a,undervoltage_events,logger_rows_written,"
        "logger_errors,camera_capture_sequence,camera_total_captures,"
        "camera_successful_captures,camera_failed_captures,camera_dropped_captures,"
        "camera_last_file_size_bytes,camera_last_write_latency_ms,"
        "image_sync_imu_delta_ms,image_sync_gps_delta_ms,image_sync_baro_delta_ms,"
        "image_quality_sharpness,image_quality_brightness,"
        "image_quality_underexposed_fraction,image_quality_overexposed_fraction,"
        "image_quality_status,images_referenced,images_present,images_missing,"
        "images_orphan,gps_ground_speed_mps,gps_course_deg,gps_satellites,"
        "gps_hdop,gps_fix_type,gps_timestamp_ns,baro_timestamp_ns,"
        "nav_timestamp_ns,nav_origin_latitude,nav_origin_longitude,"
        "estimated_latitude,estimated_longitude,estimated_north_m,"
        "estimated_east_m,estimated_altitude_m,estimated_agl_m,"
        "estimated_velocity_north_mps,estimated_velocity_east_mps,"
        "estimated_ground_speed_mps,estimated_course_deg,estimated_heading_deg,"
        "navigation_mode,position_quality,heading_quality,altitude_quality,"
        "position_source,nav_gps_valid,nav_gps_rejected,"
        "nav_gps_rejection_reason,nav_gps_age_ms,nav_gps_position_error_m,"
        "dead_reckoning_active,dead_reckoning_age_s,recovery_active,"
        "navigation_valid,safe_for_guidance,nav_consecutive_good_gps,"
        "nav_consecutive_bad_gps,estimator_reset_count,nav_last_event,"
        "servo_left,servo_right,servo_drogue,heading_cmd,roll_cmd,pitch_cmd"
    )

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data = PayloadSnapshot()
        self._mission_start: float | None = None
        self._metrics: dict[str, SubsystemMetric] = {}
        self._events: list[dict[str, Any]] = []
        self._event_debounce: dict[tuple[str, str, str], float] = {}
        self._state_history: list[dict[str, Any]] = []
        self._faults: dict[str, bool] = {}
        self._test_session: dict[str, Any] | None = None
        self._test_samples: list[dict[str, Any]] = []

    def start_mission_clock(self) -> None:
        """Reset and start the mission elapsed-time clock."""
        with self._lock:
            self._mission_start = time.time()

    def get_mission_time(self) -> float:
        """Seconds elapsed since mission clock started."""
        with self._lock:
            if self._mission_start is None:
                return 0.0
            return time.time() - self._mission_start

    def update(self, **kwargs: Any) -> None:
        """Update one or more fields atomically."""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._data, key):
                    setattr(self._data, key, value)
                else:
                    raise AttributeError(f"Unknown shared field: {key}")
            self._data.timestamp = time.time()
            if self._mission_start is not None:
                self._data.mission_time = time.time() - self._mission_start

    def transition_state(
        self,
        target: MissionState | str,
        reason: str,
        source: str = "FSM",
        trigger_values: dict[str, Any] | None = None,
        **updates: Any,
    ) -> None:
        """Update mission state through the authoritative transition path."""
        target_value = target.value if isinstance(target, MissionState) else str(target)
        with self._lock:
            previous = self._data.state
            mission_time = 0.0
            if self._mission_start is not None:
                mission_time = _wall_time() - self._mission_start
            self._data.previous_state = previous if previous != target_value else self._data.previous_state
            self._data.state = target_value
            self._data.state_entry_timestamp = _wall_time()
            self._data.state_entry_mission_time = mission_time
            self._data.state_transition_reason = reason
            for key, value in updates.items():
                if hasattr(self._data, key):
                    setattr(self._data, key, value)
                else:
                    raise AttributeError(f"Unknown shared field: {key}")
            self._data.timestamp = _wall_time()
            self._data.mission_time = mission_time
            entry = {
                "timestamp": self._data.timestamp,
                "mission_time": mission_time,
                "from": previous,
                "to": target_value,
                "reason": reason,
                "source": source,
                "trigger_values": dict(trigger_values or {}),
            }
            self._state_history.append(entry)
            self._state_history = self._state_history[-200:]
        self.record_event(
            event_type="STATE_CHANGE",
            source=source,
            severity="INFO",
            message=f"{previous} -> {target_value}: {reason}",
            details=trigger_values or {},
            debounce=False,
        )

    def record_worker_success(
        self,
        name: str,
        expected_hz: float = 0.0,
        reason: str = "Updated.",
        details: dict[str, Any] | None = None,
        status: str = "HEALTHY",
    ) -> None:
        """Record one successful worker/subsystem iteration."""
        now_mono = _mono_time()
        now_wall = _wall_time()
        with self._lock:
            metric = self._metrics.setdefault(name, SubsystemMetric(name=name))
            if expected_hz:
                metric.expected_hz = expected_hz
            previous_status = metric.status
            metric.iteration_count += 1
            if metric._last_iteration_monotonic > 0.0:
                dt = max(now_mono - metric._last_iteration_monotonic, 1e-6)
                instant_hz = 1.0 / dt
                metric.actual_hz = instant_hz if metric.actual_hz <= 0.0 else (0.85 * metric.actual_hz + 0.15 * instant_hz)
            metric._last_iteration_monotonic = now_mono
            metric.last_update_monotonic = now_mono
            metric.last_update_wall = now_wall
            metric.consecutive_errors = 0
            metric.status = status
            metric.reason = reason
            if details:
                metric.details.update(details)
            recovered = previous_status in {"FAILED", "STALE", "DEGRADED"} and metric.error_count > 0
            if recovered:
                metric.recovery_count += 1
        if recovered:
            self.record_event(
                event_type="SENSOR_RECOVERED",
                source=name,
                severity="INFO",
                message=f"{name} recovered.",
            )

    def record_worker_error(
        self,
        name: str,
        error: Exception | str,
        expected_hz: float = 0.0,
        status: str = "FAILED",
    ) -> None:
        """Record one worker/subsystem failure without stopping other workers."""
        text = str(error)
        now_wall = _wall_time()
        with self._lock:
            metric = self._metrics.setdefault(name, SubsystemMetric(name=name))
            if expected_hz:
                metric.expected_hz = expected_hz
            metric.error_count += 1
            metric.consecutive_errors += 1
            metric.last_error = text
            metric.last_error_time = now_wall
            metric.status = status
            metric.reason = text
        self.record_event(
            event_type="THREAD_ERROR",
            source=name,
            severity="ERROR",
            message=text,
        )

    def set_worker_disabled(self, name: str, reason: str = "Disabled by config.") -> None:
        with self._lock:
            metric = self._metrics.setdefault(name, SubsystemMetric(name=name))
            metric.status = "DISABLED"
            metric.reason = reason

    def record_event(
        self,
        event_type: str,
        source: str,
        severity: str,
        message: str,
        details: dict[str, Any] | None = None,
        debounce: bool = True,
    ) -> None:
        """Append a lightweight mission event with debounce for repeated faults."""
        now_wall = _wall_time()
        now_mono = _mono_time()
        key = (source, event_type, message)
        with self._lock:
            if debounce:
                last_mono = self._event_debounce.get(key, 0.0)
                if now_mono - last_mono < config.DIAGNOSTIC_EVENT_DEBOUNCE_SEC:
                    return
                self._event_debounce[key] = now_mono
            mission_time = 0.0 if self._mission_start is None else now_wall - self._mission_start
            self._events.append(
                {
                    "timestamp": now_wall,
                    "mission_time": mission_time,
                    "severity": severity,
                    "source": source,
                    "event_type": event_type,
                    "message": message,
                    "details": dict(details or {}),
                }
            )
            self._events = self._events[-300:]

    def set_fault(self, name: str, enabled: bool) -> None:
        """Enable or disable a mock/bench fault injection flag."""
        with self._lock:
            self._faults[name] = enabled
        self.record_event(
            "FAULT_INJECTION",
            "TEST",
            "WARN" if enabled else "INFO",
            f"{name} {'enabled' if enabled else 'disabled'}.",
            {"fault": name, "enabled": enabled},
            debounce=False,
        )

    def is_fault_active(self, name: str) -> bool:
        with self._lock:
            return bool(self._faults.get(name, False))

    def get_faults(self) -> dict[str, bool]:
        with self._lock:
            return dict(self._faults)

    def start_test_session(self, mode: str, config_snapshot: dict[str, Any] | None = None) -> str:
        """Start a dashboard test recording session."""
        test_id = datetime.utcnow().strftime("garuda-test-%Y%m%d-%H%M%S")
        with self._lock:
            self._test_session = {
                "test_id": test_id,
                "mode": mode,
                "start_timestamp": _wall_time(),
                "end_timestamp": None,
                "config": dict(config_snapshot or {}),
            }
            self._test_samples = []
        self.record_event("TEST_STARTED", "TEST", "INFO", f"Test session {test_id} started.", debounce=False)
        return test_id

    def stop_test_session(self) -> dict[str, Any]:
        """Stop test recording and return a structured summary."""
        with self._lock:
            if self._test_session is None:
                return {"active": False, "message": "No active test session."}
            self._test_session["end_timestamp"] = _wall_time()
            report = self._build_test_report_locked()
            self._test_session = None
        self.record_event("TEST_STOPPED", "TEST", "INFO", f"Test session {report['test_id']} stopped.", debounce=False)
        return report

    def reset_test_session(self) -> None:
        with self._lock:
            self._test_session = None
            self._test_samples = []
        self.record_event("TEST_RESET", "TEST", "INFO", "Test recording reset.", debounce=False)

    def record_test_sample(self) -> None:
        """Record a low-rate snapshot for final test analysis."""
        with self._lock:
            if self._test_session is None:
                return
            snap = self._data
            self._test_samples.append(
                {
                    "timestamp": snap.timestamp,
                    "mission_time": snap.mission_time,
                    "state": snap.state,
                    "baro_altitude": snap.baro_altitude,
                    "gps_altitude": snap.gps_altitude,
                    "vertical_velocity": snap.vertical_velocity,
                    "roll": snap.ahrs_roll if snap.ahrs_healthy else snap.roll,
                    "pitch": snap.ahrs_pitch if snap.ahrs_healthy else snap.pitch,
                    "yaw": snap.ahrs_yaw if snap.ahrs_healthy else snap.yaw,
                    "bus_voltage_v": snap.bus_voltage_v,
                    "current_a": snap.current_a,
                }
            )
            self._test_samples = self._test_samples[-20000:]

    def get_test_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": self._test_session is not None,
                "session": dict(self._test_session or {}),
                "sample_count": len(self._test_samples),
            }

    def _build_test_report_locked(self) -> dict[str, Any]:
        session = dict(self._test_session or {})
        samples = list(self._test_samples)

        def minmax(key: str) -> dict[str, float | None]:
            values = [float(item[key]) for item in samples if item.get(key) is not None]
            return {"min": min(values) if values else None, "max": max(values) if values else None}

        return {
            "test_id": session.get("test_id", ""),
            "mode": session.get("mode", ""),
            "start_timestamp": session.get("start_timestamp"),
            "end_timestamp": session.get("end_timestamp"),
            "config": session.get("config", {}),
            "sample_count": len(samples),
            "events": list(self._events),
            "state_history": list(self._state_history),
            "workers": {
                name: metric.as_dict(_mono_time())
                for name, metric in sorted(self._metrics.items())
            },
            "minmax": {
                "baro_altitude": minmax("baro_altitude"),
                "gps_altitude": minmax("gps_altitude"),
                "vertical_velocity": minmax("vertical_velocity"),
                "roll": minmax("roll"),
                "pitch": minmax("pitch"),
                "yaw": minmax("yaw"),
                "bus_voltage_v": minmax("bus_voltage_v"),
                "current_a": minmax("current_a"),
            },
        }

    def get_diagnostics_snapshot(self) -> dict[str, Any]:
        """Return metrics, event log, and state history without exposing internals."""
        now = _mono_time()
        with self._lock:
            return {
                "workers": {
                    name: metric.as_dict(now)
                    for name, metric in sorted(self._metrics.items())
                },
                "events": list(self._events),
                "state_history": list(self._state_history),
                "faults": dict(self._faults),
                "test": {
                    "active": self._test_session is not None,
                    "session": dict(self._test_session or {}),
                    "sample_count": len(self._test_samples),
                },
            }

    def publish_attitude(self, attitude: AttitudeState) -> None:
        """Atomically publish final AHRS attitude while preserving legacy fields."""
        accuracy = 0.0 if attitude.accuracy_rad is None else attitude.accuracy_rad
        self.update(
            roll=attitude.roll_deg,
            pitch=attitude.pitch_deg,
            yaw=attitude.yaw_deg,
            gyro_x=attitude.gyro_x_dps,
            gyro_y=attitude.gyro_y_dps,
            gyro_z=attitude.gyro_z_dps,
            ahrs_enabled=attitude.enabled,
            ahrs_source=attitude.source,
            ahrs_valid=attitude.valid,
            ahrs_healthy=attitude.healthy,
            ahrs_confidence=attitude.confidence,
            quat_w=attitude.q_w,
            quat_x=attitude.q_x,
            quat_y=attitude.q_y,
            quat_z=attitude.q_z,
            ahrs_roll=attitude.roll_deg,
            ahrs_pitch=attitude.pitch_deg,
            ahrs_yaw=attitude.yaw_deg,
            attitude_accuracy_rad=accuracy,
            imu_sample_age_ms=attitude.sample_age_ms,
            accel_correction_active=attitude.accel_correction_enabled,
            mag_correction_active=attitude.mag_correction_enabled,
            ahrs_timestamp_ns=attitude.timestamp_ns,
        )

    def publish_navigation(self, navigation_state: Any) -> None:
        """Atomically publish one coherent estimated navigation state."""
        updates = navigation_state.as_shared_updates()
        with self._lock:
            for key, value in updates.items():
                if hasattr(self._data, key):
                    setattr(self._data, key, value)
                else:
                    raise AttributeError(f"Unknown shared field: {key}")
            self._data.timestamp = time.time()
            if self._mission_start is not None:
                self._data.mission_time = time.time() - self._mission_start

    def get_snapshot(self) -> PayloadSnapshot:
        """Return a copy of the current data (safe to use outside the lock)."""
        with self._lock:
            return PayloadSnapshot(**{
                f.name: getattr(self._data, f.name) for f in fields(self._data)
            })

    def to_csv_row(self) -> str:
        """Format current data as a CSV row matching CSV_HEADER."""
        snap = self.get_snapshot()
        return (
            f"{snap.timestamp:.3f},{snap.mission_time:.3f},{snap.state},"
            f"{snap.latitude:.6f},{snap.longitude:.6f},"
            f"{snap.gps_altitude:.2f},{snap.baro_altitude:.2f},"
            f"{snap.vertical_velocity:.2f},{snap.max_altitude:.2f},"
            f"{snap.roll:.2f},{snap.pitch:.2f},{snap.yaw:.2f},"
            f"{snap.gyro_x:.3f},{snap.gyro_y:.3f},{snap.gyro_z:.3f},"
            f"{snap.image_name},{snap.image_timestamp:.3f},"
            f"{snap.battery:.1f},{snap.status},"
            f"{int(snap.ahrs_enabled)},{snap.ahrs_source},{int(snap.ahrs_valid)},"
            f"{int(snap.ahrs_healthy)},{snap.ahrs_confidence},"
            f"{snap.quat_w:.8f},{snap.quat_x:.8f},{snap.quat_y:.8f},{snap.quat_z:.8f},"
            f"{snap.ahrs_roll:.2f},{snap.ahrs_pitch:.2f},{snap.ahrs_yaw:.2f},"
            f"{snap.attitude_accuracy_rad:.6f},{snap.imu_sample_age_ms:.2f},"
            f"{int(snap.accel_correction_active)},{int(snap.mag_correction_active)},"
            f"{snap.ahrs_timestamp_ns},"
            f"{snap.raw_gyro_x:.6f},{snap.raw_gyro_y:.6f},{snap.raw_gyro_z:.6f},"
            f"{snap.raw_accel_x:.4f},{snap.raw_accel_y:.4f},{snap.raw_accel_z:.4f},"
            f"{snap.raw_mag_x:.4f},{snap.raw_mag_y:.4f},{snap.raw_mag_z:.4f},"
            f"{snap.raw_quat_w:.8f},{snap.raw_quat_x:.8f},{snap.raw_quat_y:.8f},"
            f"{snap.raw_quat_z:.8f},{snap.raw_imu_timestamp_ns},"
            f"{snap.raw_imu_accuracy_rad:.6f},{snap.raw_imu_calibration_status},"
            f"{snap.raw_baro_pressure_hpa:.4f},{snap.raw_baro_temperature_c:.4f},"
            f"{snap.gimbal_x_deflection_deg:.3f},{snap.gimbal_y_deflection_deg:.3f},"
            f"{snap.gimbal_stepper_angle_deg:.3f},{snap.gimbal_servo_angle_deg:.3f},"
            f"{snap.gimbal_stepper_steps},{int(snap.gimbal_ok)},"
            f"{int(snap.launch_detected)},{int(snap.apogee_detected)},"
            f"{int(snap.payload_ejected)},{int(snap.glider_deployed)},"
            f"{int(snap.actuation_enabled)},{snap.previous_state},"
            f"{snap.state_transition_reason},{snap.telemetry_sequence},"
            f"{snap.telemetry_tx_count},{snap.bus_voltage_v:.3f},"
            f"{snap.current_a:.3f},{snap.power_w:.3f},{snap.min_voltage_v:.3f},"
            f"{snap.max_current_a:.3f},{snap.undervoltage_events},"
            f"{snap.logger_rows_written},{snap.logger_errors},"
            f"{snap.camera_capture_sequence},{snap.camera_total_captures},"
            f"{snap.camera_successful_captures},{snap.camera_failed_captures},"
            f"{snap.camera_dropped_captures},{snap.camera_last_file_size_bytes},"
            f"{snap.camera_last_write_latency_ms:.3f},"
            f"{snap.image_sync_imu_delta_ms:.3f},{snap.image_sync_gps_delta_ms:.3f},"
            f"{snap.image_sync_baro_delta_ms:.3f},{snap.image_quality_sharpness:.3f},"
            f"{snap.image_quality_brightness:.3f},"
            f"{snap.image_quality_underexposed_fraction:.5f},"
            f"{snap.image_quality_overexposed_fraction:.5f},"
            f"{snap.image_quality_status},{snap.images_referenced},"
            f"{snap.images_present},{snap.images_missing},{snap.images_orphan},"
            f"{snap.gps_ground_speed_mps:.3f},{snap.gps_course_deg:.3f},"
            f"{snap.gps_satellites},{snap.gps_hdop:.3f},{snap.gps_fix_type},"
            f"{snap.gps_timestamp_ns},{snap.baro_timestamp_ns},{snap.nav_timestamp_ns},"
            f"{snap.nav_origin_latitude:.8f},{snap.nav_origin_longitude:.8f},"
            f"{snap.estimated_latitude:.8f},{snap.estimated_longitude:.8f},"
            f"{snap.estimated_north_m:.3f},{snap.estimated_east_m:.3f},"
            f"{snap.estimated_altitude_m:.3f},{snap.estimated_agl_m:.3f},"
            f"{snap.estimated_velocity_north_mps:.3f},{snap.estimated_velocity_east_mps:.3f},"
            f"{snap.estimated_ground_speed_mps:.3f},{snap.estimated_course_deg:.3f},"
            f"{snap.estimated_heading_deg:.3f},{snap.navigation_mode},"
            f"{snap.position_quality},{snap.heading_quality},{snap.altitude_quality},"
            f"{snap.position_source},{int(snap.nav_gps_valid)},{int(snap.nav_gps_rejected)},"
            f"{snap.nav_gps_rejection_reason},{snap.nav_gps_age_ms:.3f},"
            f"{snap.nav_gps_position_error_m:.3f},{int(snap.dead_reckoning_active)},"
            f"{snap.dead_reckoning_age_s:.3f},{int(snap.recovery_active)},"
            f"{int(snap.navigation_valid)},{int(snap.safe_for_guidance)},"
            f"{snap.nav_consecutive_good_gps},{snap.nav_consecutive_bad_gps},"
            f"{snap.estimator_reset_count},{snap.nav_last_event}"
        )
