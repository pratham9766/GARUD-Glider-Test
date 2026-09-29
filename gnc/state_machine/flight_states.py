from enum import Enum, auto
import logging

log = logging.getLogger(__name__)


class FlightState(str, Enum):
    BOOST                   = "BOOST"
    DROGUE_DESCENT          = "DROGUE_DESCENT"
    DEPLOYMENT_TRIGGER      = "DEPLOYMENT_TRIGGER"
    DEPLOYMENT_VERIFICATION = "DEPLOYMENT_VERIFICATION"
    GUIDED_DESCENT          = "GUIDED_DESCENT"
    LANDED                  = "LANDED"
    
    # Legacy states for Garuda_TARSR compatibility
    BOOT = "BOOT"
    IDLE = "IDLE"
    DESCENT = "DESCENT"
    ERROR = "ERROR"

    def __str__(self) -> str:
        return self.value


class StateMachine:
    """
    Manages the transitions between flight states.

    The 600m AGL deployment trigger uses a moving average over at least 10
    barometer samples plus a confirmed-descending check from IMU vertical accel.

    Reset Recovery API
    ------------------
    force_state(name)  -- jump to a named state on boot after a Pi crash
    lock_drogue()      -- permanently block drogue re-fire (called on recovery
                          when .state shows drogue_fired=True)
    drogue_fired       -- property: True once DEPLOYMENT_TRIGGER is entered,
                          or True if lock_drogue() was called
    """

    def __init__(self, ground_altitude: float = 0.0) -> None:
        self.state           = FlightState.BOOST
        self.ground_altitude = ground_altitude

        self.baro_history             = []
        self.history_size             = 10
        self.deployment_agl_threshold = 600.0

        # Drogue safety flag — set True when drogue fires, never cleared
        self._drogue_fired  = False
        self._drogue_locked = False   # True = permanently blocked by recovery
        self.transition_history = [self.state]

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def drogue_fired(self) -> bool:
        """True once the drogue has been triggered (or was fired before reset)."""
        return self._drogue_fired

    # ------------------------------------------------------------------
    # Reset Recovery API
    # ------------------------------------------------------------------

    def force_state(self, state_name: str) -> None:
        """
        Jump directly to a named FlightState without going through transitions.

        Called by flight_computer at boot when a valid .state snapshot exists.
        Does nothing and logs a warning if state_name is not recognised.

        Args:
            state_name: Name of the FlightState enum member, e.g. "GUIDED_DESCENT"
        """
        try:
            target = FlightState[state_name]
        except KeyError:
            log.warning("[SM] force_state: unknown state '%s' — ignoring.", state_name)
            return
        log.warning("[SM] force_state: %s -> %s (reset recovery)",
                    self.state.name, target.name)
        self.state = target
        self.transition_history.append(target)
        # Clear history so stale baro samples don't affect transition logic
        self.baro_history.clear()

    def lock_drogue(self) -> None:
        """
        Permanently lock out the drogue channel.

        Called by flight_computer at boot when .state shows drogue_fired=True.
        After this call, drogue_fired is always True and the deployment trigger
        state will not set it again — preventing a double-fire.
        """
        self._drogue_fired  = True
        self._drogue_locked = True
        log.warning("[SM] Drogue LOCKED OUT — reset recovery, already fired before crash.")

    # ------------------------------------------------------------------
    # Main update loop
    # ------------------------------------------------------------------

    def update(self, current_alt: float, vertical_velocity: float) -> FlightState:
        """
        Update the state machine based on sensor data.

        Args:
            current_alt:       Current MSL altitude (from baro/EKF), metres
            vertical_velocity: EKF vertical velocity (m/s), negative = descending

        Returns:
            The current FlightState
        """
        self.baro_history.append(current_alt)
        if len(self.baro_history) > self.history_size:
            self.baro_history.pop(0)

        agl = current_alt - self.ground_altitude

        if self.state == FlightState.BOOST:
            # Transition to drogue descent when apogee is reached
            if len(self.baro_history) == self.history_size:
                if (self.baro_history[-1] < self.baro_history[0] - 5.0
                        and vertical_velocity < -2.0):
                    log.info("[SM] BOOST -> DROGUE_DESCENT (apogee detected)")
                    self.state = FlightState.DROGUE_DESCENT
                    self.transition_history.append(self.state)

        elif self.state == FlightState.DROGUE_DESCENT:
            # 600m AGL moving-average trigger
            if len(self.baro_history) == self.history_size:
                avg_agl = (sum(self.baro_history) / len(self.baro_history)) - self.ground_altitude
                if avg_agl <= self.deployment_agl_threshold and vertical_velocity < -2.0:
                    log.info("[SM] DROGUE_DESCENT -> DEPLOYMENT_TRIGGER (AGL=%.1f m)", avg_agl)
                    self.state = FlightState.DEPLOYMENT_TRIGGER
                    self.transition_history.append(self.state)

        elif self.state == FlightState.DEPLOYMENT_TRIGGER:
            # Mark drogue as fired (unless locked out from a prior flight)
            if not self._drogue_locked:
                self._drogue_fired = True
            log.info("[SM] DEPLOYMENT_TRIGGER -> DEPLOYMENT_VERIFICATION")
            self.state = FlightState.DEPLOYMENT_VERIFICATION
            self.transition_history.append(self.state)

        elif self.state == FlightState.DEPLOYMENT_VERIFICATION:
            # Parafoil wings lock under aero load — then hand off to GNC
            log.info("[SM] DEPLOYMENT_VERIFICATION -> GUIDED_DESCENT")
            self.state = FlightState.GUIDED_DESCENT
            self.transition_history.append(self.state)

        elif self.state == FlightState.GUIDED_DESCENT:
            if agl <= 5.0:
                log.info("[SM] GUIDED_DESCENT -> LANDED (AGL=%.1f m)", agl)
                self.state = FlightState.LANDED
                self.transition_history.append(self.state)

        return self.state
