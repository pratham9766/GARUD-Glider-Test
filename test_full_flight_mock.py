"""Deterministic end-to-end mock flight for Raspberry Pi validation.

This test never opens GPIO, I2C, SPI, or serial hardware. It drives a coherent
mock GPS/IMU/barometer flight profile through the real GNC loop, ONNX policy,
flight state machine, drogue command, and mock left/right servo worker.
"""

from __future__ import annotations

import argparse
import logging
import math
import threading
import time
from pathlib import Path

import yaml

import config
from core.shared_data import SharedData
from core.thread_manager import ManagedThread, ThreadManager
from gnc.glider_servo_worker import glider_servo_worker
from gnc.gnc_worker import FlightComputer


PROJECT_ROOT = Path(__file__).resolve().parent
log = logging.getLogger("FullFlightMock")


def _mission_target() -> tuple[float, float]:
    with open(PROJECT_ROOT / "config" / "gains.yaml", encoding="utf-8") as stream:
        mission = yaml.safe_load(stream).get("mission", {})
    return float(mission["target_latitude"]), float(mission["target_longitude"])


def _publish_sample(
    shared: SharedData,
    altitude_m: float,
    descent_progress: float,
    target_lat: float,
    target_lon: float,
) -> None:
    start_lat = target_lat - 0.006
    start_lon = target_lon + 0.006
    progress = max(0.0, min(1.0, descent_progress))
    latitude = start_lat + (target_lat - start_lat) * progress
    longitude = start_lon + (target_lon - start_lon) * progress

    north_m = (target_lat - latitude) * 111320.0
    east_m = (target_lon - longitude) * 111320.0 * math.cos(math.radians(latitude))
    course_deg = math.degrees(math.atan2(east_m, north_m)) % 360.0
    timestamp_ns = time.monotonic_ns()

    shared.update(
        latitude=latitude,
        longitude=longitude,
        gps_altitude=altitude_m,
        gps_ground_speed_mps=18.0 if altitude_m > 0.0 else 0.0,
        gps_course_deg=course_deg,
        gps_satellites=12,
        gps_hdop=0.8,
        gps_fix_type="3D",
        gps_timestamp_ns=timestamp_ns,
        gps_ok=True,
        baro_altitude=altitude_m,
        baro_timestamp_ns=timestamp_ns,
        barometer_ok=True,
        raw_accel_x=0.0,
        raw_accel_y=0.0,
        raw_accel_z=9.80665,
        raw_gyro_x=0.0,
        raw_gyro_y=0.0,
        raw_gyro_z=0.0,
        raw_mag_x=25.0,
        raw_mag_y=0.0,
        raw_mag_z=35.0,
        raw_imu_timestamp_ns=timestamp_ns,
        imu_ok=True,
    )


def flight_profile_worker(
    shared: SharedData,
    stop_event: threading.Event,
    ascent_sec: float,
    apogee_hold_sec: float,
    descent_sec: float,
) -> None:
    target_lat, target_lon = _mission_target()
    max_altitude_m = config.TARGET_APOGEE_AGL_M
    started = time.monotonic()

    while not stop_event.is_set():
        elapsed = time.monotonic() - started
        if elapsed < ascent_sec:
            altitude_m = max_altitude_m * (elapsed / ascent_sec)
            descent_progress = 0.0
        elif elapsed < ascent_sec + apogee_hold_sec:
            altitude_m = max_altitude_m
            descent_progress = 0.0
        else:
            descent_elapsed = elapsed - ascent_sec - apogee_hold_sec
            descent_progress = min(1.0, descent_elapsed / descent_sec)
            altitude_m = max_altitude_m * (1.0 - descent_progress)

        _publish_sample(
            shared,
            altitude_m=max(0.0, altitude_m),
            descent_progress=descent_progress,
            target_lat=target_lat,
            target_lon=target_lon,
        )
        stop_event.wait(0.05)


def _contains_in_order(actual: list[str], expected: list[str]) -> bool:
    position = 0
    for item in actual:
        if position < len(expected) and item == expected[position]:
            position += 1
    return position == len(expected)


def run_full_flight(ascent_sec: float, descent_sec: float) -> bool:
    config.USE_MOCK_HARDWARE = True
    shared = SharedData()
    shared.start_mission_clock()
    target_lat, target_lon = _mission_target()
    _publish_sample(shared, 0.0, 0.0, target_lat, target_lon)

    flight_computer = FlightComputer(
        shared=shared,
        drop_height=0.0,
        enable_state_persistence=False,
    )
    if not flight_computer.rl_active:
        log.error("RL model did not load; aborting full-flight mock.")
        return False

    apogee_hold_sec = 1.0
    ground_hold_sec = 5.0
    manager = ThreadManager()
    manager.register(ManagedThread(
        "FlightProfile",
        lambda event: flight_profile_worker(
            shared,
            event,
            ascent_sec,
            apogee_hold_sec,
            descent_sec,
        ),
    ))
    manager.register(ManagedThread("GNC", lambda event: flight_computer.run(event)))
    manager.register(ManagedThread(
        "MockServos",
        lambda event: glider_servo_worker(
            shared,
            event,
            use_mock=True,
            command_drogue=True,
        ),
    ))

    log.info("Starting deterministic full-flight mock (no physical hardware).")
    manager.start_all()
    deadline = time.monotonic() + ascent_sec + apogee_hold_sec + descent_sec + ground_hold_sec
    landed_at = None
    try:
        while time.monotonic() < deadline:
            if flight_computer.state_machine.state.name == "LANDED":
                landed_at = time.monotonic()
                break
            time.sleep(0.1)
    finally:
        manager.stop_all()

    state_history = [state.name for state in flight_computer.state_machine.transition_history]
    expected_states = [
        "BOOST",
        "DROGUE_DESCENT",
        "DEPLOYMENT_TRIGGER",
        "DEPLOYMENT_VERIFICATION",
        "GUIDED_DESCENT",
        "LANDED",
    ]
    guided_cycles = (
        flight_computer.controller_counts.get("RL", 0)
        + flight_computer.controller_counts.get("PID", 0)
        + flight_computer.controller_counts.get("SALVAGE", 0)
    )
    rl_cycles = flight_computer.controller_counts.get("RL", 0)
    rl_ratio = rl_cycles / guided_cycles if guided_cycles else 0.0
    runtime = flight_computer.last_loop_monotonic - flight_computer.first_loop_monotonic
    loop_hz = (flight_computer.loop_count - 1) / runtime if runtime > 0.0 else 0.0
    final_snapshot = shared.get_snapshot()

    checks = {
        "GPS remained valid": final_snapshot.gps_ok and final_snapshot.gps_satellites >= 5,
        "Full state sequence": _contains_in_order(state_history, expected_states),
        "Drogue fired once": flight_computer.state_machine.drogue_fired,
        "Drogue command deployed": final_snapshot.servo_drogue == config.GLIDER_DROGUE_DEPLOY_ANGLE,
        "RL controlled descent": rl_cycles > 0 and rl_ratio >= 0.95,
        "Servo commands bounded": 60.0 <= flight_computer.servo_min <= flight_computer.servo_max <= 120.0,
        "Servo actuation occurred": (
            abs(flight_computer.servo_min - 90.0) > 1.0
            or abs(flight_computer.servo_max - 90.0) > 1.0
        ),
        "20 Hz loop sustained": 18.0 <= loop_hz <= 22.0,
        "Landing detected": landed_at is not None,
    }

    print("\n" + "=" * 58)
    print(" GARUD FULL-FLIGHT MOCK RESULTS")
    print("=" * 58)
    print(" State history       :", " -> ".join(state_history))
    print(" Controller counts   :", flight_computer.controller_counts)
    print(f" RL guided ratio     : {rl_ratio * 100.0:.2f}%")
    print(f" GNC loop rate       : {loop_hz:.2f} Hz")
    print(f" Loop overruns       : {flight_computer.loop_overrun_count}")
    print(f" Servo range         : {flight_computer.servo_min:.1f} .. {flight_computer.servo_max:.1f} deg")
    print(f" Final GPS           : {final_snapshot.latitude:.7f}, {final_snapshot.longitude:.7f}")
    print(f" GPS sats / HDOP     : {final_snapshot.gps_satellites} / {final_snapshot.gps_hdop:.1f}")
    print(f" Drogue command      : {final_snapshot.servo_drogue:.1f} deg")
    print("-" * 58)
    for name, passed in checks.items():
        print(f" {'PASS' if passed else 'FAIL':4} | {name}")
    overall = all(checks.values())
    print("-" * 58)
    print(" OVERALL             :", "PASS" if overall else "FAIL")
    print("=" * 58)
    return overall


def main() -> None:
    parser = argparse.ArgumentParser(description="Full mock flight with RL and mock actuation")
    parser.add_argument("--ascent", type=float, default=6.0, help="Mock ascent duration in seconds")
    parser.add_argument("--descent", type=float, default=24.0, help="Mock descent duration in seconds")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    if not run_full_flight(args.ascent, args.descent):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
