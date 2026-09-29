"""GARUDA glider bench and production-flight launcher for Raspberry Pi Zero 2 W.

Bench mode is the default and cannot actuate deployment hardware. Flight mode
requires an explicit arming phrase, valid live sensors, and a loaded RL model.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

import config
from core.flight_safety import flight_safety_worker
from core.flight_state_machine import FlightStateController
from core.mission_state import MissionState
from core.shared_data import SharedData
from core.thread_manager import ManagedThread, ThreadManager
from gnc.glider_servo_worker import glider_servo_worker
from gnc.gnc_worker import FlightComputer
from sensors.barometer import barometer_worker
from sensors.gps import gps_worker
from sensors.imu import imu_worker

logger = logging.getLogger("garuda")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GARUDA glider Raspberry Pi runtime")
    parser.add_argument("--mode", choices=("bench", "flight"), default="bench")
    parser.add_argument("--mock", action="store_true", help="Use mock sensors (bench only)")
    parser.add_argument("--real-servos", action="store_true", help="Enable PCA9685 in bench mode; controls remain neutral")
    parser.add_argument("--duration", type=float, default=30.0, help="Bench duration in seconds; 0 runs until Ctrl+C")
    parser.add_argument(
        "--arm-flight",
        metavar="PHRASE",
        help="Flight-only safety acknowledgement; must be exactly GARUDA",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.mode == "flight":
        if args.mock:
            raise SystemExit("Flight mode refuses --mock")
        if args.arm_flight != "GARUDA":
            raise SystemExit("Flight mode requires: --arm-flight GARUDA")
    elif args.arm_flight:
        raise SystemExit("--arm-flight is only valid with --mode flight")


def _sensors_ready(shared: SharedData) -> tuple[bool, str]:
    snap = shared.get_snapshot()
    missing = []
    if not snap.imu_ok or snap.raw_imu_timestamp_ns <= 0:
        missing.append("BNO085")
    if not snap.barometer_ok or snap.baro_timestamp_ns <= 0:
        missing.append("BMP388")
    if not snap.gps_ok or snap.gps_timestamp_ns <= 0:
        missing.append("GPS_FIX")
    return not missing, ",".join(missing) if missing else "READY"


def _wait_for_sensors(shared: SharedData, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    last_reason = "INITIALIZING"
    while time.monotonic() < deadline:
        ready, last_reason = _sensors_ready(shared)
        if ready:
            return
        time.sleep(0.1)
    raise RuntimeError(f"Preflight sensor timeout: {last_reason}")


def _register_sensor_workers(manager: ThreadManager, shared: SharedData) -> None:
    manager.register(ManagedThread("GPS", lambda event: gps_worker(shared, event)))
    manager.register(ManagedThread("BNO085", lambda event: imu_worker(shared, event)))
    manager.register(ManagedThread("BMP388", lambda event: barometer_worker(shared, event)))


def _install_signal_handlers(stop_callback) -> None:
    def request_stop(signum, _frame):
        logger.warning("Signal %s received; neutralizing and stopping.", signum)
        stop_callback()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def run(args: argparse.Namespace) -> int:
    config.USE_MOCK_HARDWARE = bool(args.mock)
    # The reusable mock workers support auto-arm for legacy tests. This launcher
    # always requires its own explicit mode-specific arming path.
    config.AUTO_ARM_IN_MOCK_MODE = False
    shared = SharedData()
    sensor_manager = ThreadManager()
    control_manager = ThreadManager()
    _register_sensor_workers(sensor_manager, shared)

    stopping = False

    def request_stop() -> None:
        nonlocal stopping
        stopping = True
        shared.update(
            flight_armed=False,
            guidance_requested=False,
            actuation_enabled=False,
            actuation_inhibit_reason="OPERATOR_STOP",
            servo_left=90.0,
            servo_right=90.0,
        )

    _install_signal_handlers(request_stop)
    sensor_manager.start_all()

    try:
        _wait_for_sensors(shared, config.PREFLIGHT_SENSOR_READY_TIMEOUT_SEC)
        logger.info("Preflight sensors READY: BNO085, BMP388, GPS fix")

        flight_computer = FlightComputer(
            shared=shared,
            enable_state_persistence=args.mode == "flight",
            external_state_authority=True,
        )
        if config.FLIGHT_REQUIRE_RL and not flight_computer.rl_active:
            raise RuntimeError("RL model failed validation; refusing runtime")
        logger.info("RL model validated and warmed up")

        controller = FlightStateController(shared)
        controller.start()
        control_manager.register(ManagedThread("GNC", lambda event: flight_computer.run(event)))
        control_manager.register(ManagedThread("SafetyGate", lambda event: flight_safety_worker(shared, event)))

        use_mock_servos = args.mode == "bench" and not args.real_servos
        servo_thread = control_manager.register(ManagedThread(
            "GliderServos",
            lambda event: glider_servo_worker(
                shared,
                event,
                use_mock=use_mock_servos,
                command_drogue=args.mode == "flight",
                require_actuation_enabled=True,
            ),
        ))
        control_manager.start_all()

        servo_deadline = time.monotonic() + 3.0
        while time.monotonic() < servo_deadline:
            if shared.get_snapshot().servo_ok:
                break
            if not servo_thread.is_alive:
                raise RuntimeError("Servo worker failed during initialization")
            time.sleep(0.05)
        else:
            raise RuntimeError("Servo worker did not report ready within 3 seconds")

        if args.mode == "flight":
            controller.arm()
            logger.warning("FLIGHT ARMED: automatic mission transitions are active")
        else:
            logger.info("BENCH SAFE: mission disarmed, controls neutral, drogue inhibited")

        started = time.monotonic()
        while not stopping:
            snap = shared.get_snapshot()
            if args.mode == "flight":
                state = controller.update()
                if state == MissionState.LANDED:
                    logger.info("Landing confirmed; stopping flight runtime")
                    break
            elif args.duration > 0 and time.monotonic() - started >= args.duration:
                break

            if int((time.monotonic() - started) * 2) % 10 == 0:
                logger.debug(
                    "state=%s sensors=%s/%s/%s actuation=%s reason=%s controller=%s",
                    snap.state,
                    snap.gps_ok,
                    snap.imu_ok,
                    snap.barometer_ok,
                    snap.actuation_enabled,
                    snap.actuation_inhibit_reason,
                    flight_computer.last_controller_used,
                )
            time.sleep(0.1)
        return 0
    except Exception as exc:
        logger.exception("Runtime refused or aborted: %s", exc)
        request_stop()
        return 1
    finally:
        request_stop()
        control_manager.stop_all()
        sensor_manager.stop_all()


def main() -> int:
    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL, logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    args = build_parser().parse_args()
    _validate_args(args)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
