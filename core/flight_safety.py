"""Fail-closed actuator safety gate for the production flight runtime."""

from __future__ import annotations

import logging
import time

import config
from core.mission_state import MissionState

logger = logging.getLogger(__name__)


def _sample_age_ms(timestamp_ns: int, now_ns: int) -> float:
    if timestamp_ns <= 0:
        return float("inf")
    return max(0.0, (now_ns - timestamp_ns) / 1_000_000.0)


def evaluate_actuation(shared) -> tuple[bool, str]:
    """Return the current fail-closed actuation decision and reason."""
    snap = shared.get_snapshot()
    now_ns = time.monotonic_ns()

    if not snap.flight_armed:
        return False, "NOT_ARMED"
    if snap.state != MissionState.GUIDED_DESCENT.value:
        return False, f"STATE_{snap.state}"
    if not snap.guidance_requested:
        return False, "GUIDANCE_NOT_REQUESTED"
    if not snap.glider_deployed:
        return False, "GLIDER_NOT_DEPLOYED"
    if not snap.imu_ok:
        return False, "IMU_UNHEALTHY"
    if not snap.barometer_ok:
        return False, "BAROMETER_UNHEALTHY"
    if not snap.gps_ok:
        return False, "GPS_NO_FIX"
    if not snap.servo_ok:
        return False, "SERVO_WORKER_UNHEALTHY"
    if not snap.gnc_ok:
        return False, "GNC_UNHEALTHY"

    ages = {
        "IMU_STALE": _sample_age_ms(snap.raw_imu_timestamp_ns, now_ns),
        "BAROMETER_STALE": _sample_age_ms(snap.baro_timestamp_ns, now_ns),
        "GPS_STALE": _sample_age_ms(snap.gps_timestamp_ns, now_ns),
        "GNC_STALE": _sample_age_ms(snap.gnc_timestamp_ns, now_ns),
    }
    limits = {
        "IMU_STALE": config.FLIGHT_MAX_IMU_AGE_MS,
        "BAROMETER_STALE": config.FLIGHT_MAX_BARO_AGE_MS,
        "GPS_STALE": config.FLIGHT_MAX_GPS_AGE_MS,
        "GNC_STALE": config.FLIGHT_MAX_GNC_AGE_MS,
    }
    for reason, age_ms in ages.items():
        if age_ms > limits[reason]:
            return False, f"{reason}:{age_ms:.0f}ms"
    return True, "ENABLED"


def flight_safety_worker(shared, stop_event) -> None:
    """Continuously gate physical control using state and sensor freshness."""
    last_decision = None
    while not stop_event.is_set():
        enabled, reason = evaluate_actuation(shared)
        shared.update(
            actuation_enabled=enabled,
            actuation_inhibit_reason=reason,
        )
        decision = (enabled, reason)
        if decision != last_decision:
            logger.info("Actuation safety gate: %s (%s)", "ENABLED" if enabled else "INHIBITED", reason)
            last_decision = decision
        stop_event.wait(0.05)

    shared.update(
        actuation_enabled=False,
        actuation_inhibit_reason="RUNTIME_STOPPED",
    )
