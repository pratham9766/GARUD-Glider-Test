"""
Glider-Only Test Script — GARUDA TARSR
=======================================
Tests ONLY the glider subsystem:
  - GNC Flight Computer (guidance + PID/RL)
  - Glider Servo Worker (wing brake actuation)
  - Mock GPS, IMU, Barometer

No camera, gimbal, telemetry, or mapping is started.

Usage:
    python test_glider_only.py              # full test (30 seconds)
    python test_glider_only.py --duration 60
    python test_glider_only.py --rl-only    # test RL inference speed only
    python test_glider_only.py --real-servos --duration 30
"""

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config

# Force mock hardware
config.USE_MOCK_HARDWARE = True

# Disable everything except glider
config.ENABLE_GPS           = True    # needed for guidance
config.ENABLE_IMU           = True    # needed for heading
config.ENABLE_BAROMETER     = True    # needed for altitude
config.ENABLE_CAMERA        = False
config.ENABLE_GIMBAL        = False
config.ENABLE_TELEMETRY     = False
config.ENABLE_MAPPING       = False
config.ENABLE_LOGGING       = True
config.ENABLE_NAVIGATION_ESTIMATOR = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("GliderTest")


def test_rl_inference() -> bool:
    """Benchmark RL model inference speed on this hardware."""
    import numpy as np

    log.info("=" * 50)
    log.info("TEST: RL Inference Speed")
    log.info("=" * 50)

    gains_path = PROJECT_ROOT / "config" / "gains.yaml"
    try:
        import yaml
        with open(gains_path) as f:
            gains = yaml.safe_load(f)
        obs_dim   = gains.get("rl", {}).get("obs_dim", 16)
        model_rel = gains.get("rl", {}).get("onnx_model_path", "models/sac_policy_16D.onnx")
        model_path = PROJECT_ROOT / model_rel
    except Exception as e:
        log.warning("Could not read gains.yaml: %s", e)
        obs_dim    = 16
        model_path = PROJECT_ROOT / "models" / "sac_policy_16D.onnx"

    log.info("Looking for model: %s", model_path)
    log.info("Expected obs_dim: %d", obs_dim)

    if not model_path.exists():
        log.warning("ONNX model not found — RL inference test SKIPPED")
        log.warning("PID fallback will be used during actual flight")
        return False

    try:
        import onnxruntime as ort
    except ImportError:
        log.error("onnxruntime not installed — run: pip install onnxruntime")
        return False

    sess = ort.InferenceSession(str(model_path))
    input_name = sess.get_inputs()[0].name
    obs = np.random.rand(1, obs_dim).astype(np.float32)

    # Warmup
    for _ in range(10):
        sess.run(None, {input_name: obs})

    # Benchmark 200 inferences
    times = []
    for _ in range(200):
        t = time.perf_counter()
        sess.run(None, {input_name: obs})
        times.append((time.perf_counter() - t) * 1000)

    avg  = sum(times) / len(times)
    peak = max(times)
    log.info("RL Inference: avg=%.2fms  peak=%.2fms  (watchdog=5ms)", avg, peak)

    if avg < 5.0:
        log.info("RESULT: PASS ✅ — RL will run as PRIMARY controller")
        return True
    elif avg < 10.0:
        log.warning("RESULT: MARGINAL ⚠️ — RL may occasionally exceed watchdog → PID fallback")
    else:
        log.warning("RESULT: SLOW ❌ — RL exceeds 5ms watchdog consistently → PID fallback always")


    return False


def test_gnc_loop(duration_sec: int = 30, real_servos: bool = False):
    """Run GNC + Servo workers for N seconds and report loop timing."""
    log.info("=" * 50)

    # Use one coherent descending flight profile for both mock altitude
    # sensors. Leave a margin so the state remains airborne through the test.
    config.MOCK_GLIDER_DESCENT_ONLY = True
    config.SIMULATION_DURATION_SEC = max(float(duration_sec) + 5.0, 10.0)
    log.info("TEST: GNC + Servo Loop (%ds)", duration_sec)
    log.info("=" * 50)

    from core.shared_data import SharedData
    from core.thread_manager import ManagedThread, ThreadManager
    from sensors.gps import gps_worker
    from sensors.imu import imu_worker
    from sensors.barometer import barometer_worker
    from gnc.gnc_worker import FlightComputer
    from gnc.glider_servo_worker import glider_servo_worker

    shared = SharedData()
    sensor_mgr = ThreadManager()
    control_mgr = ThreadManager()

    # Keep navigation sensors simulated while optionally driving real servos.
    sensor_mgr.register(ManagedThread("GPS",       lambda evt: gps_worker(shared, evt)))
    sensor_mgr.register(ManagedThread("IMU",       lambda evt: imu_worker(shared, evt)))
    sensor_mgr.register(ManagedThread("Barometer", lambda evt: barometer_worker(shared, evt)))

    # A valid sample must exist before FlightComputer captures its altitude
    # reference. Otherwise a zero first sample can make the test land at once.
    log.info("Starting mock sensors and waiting for valid samples...")
    sensor_mgr.start_all()
    sensor_deadline = time.monotonic() + 5.0
    while time.monotonic() < sensor_deadline:
        snap = shared.get_snapshot()
        if snap.gps_ok and snap.imu_ok and snap.barometer_ok:
            break
        time.sleep(0.05)
    else:
        sensor_mgr.stop_all()
        raise RuntimeError("Mock sensors did not become ready within 5 seconds")

    # Construct GNC only after sensor warmup, then force its authoritative
    # internal state machine into the control state used by this bench test.
    # Mock barometer starts near deployment altitude. Treat zero as ground so
    # the forced guided-descent state remains airborne for the full test.
    drop_height = shared.get_snapshot().baro_altitude
    fc = FlightComputer(
        shared,
        drop_height=drop_height,
        enable_state_persistence=False,
    )
    fc.state_machine.force_state("GUIDED_DESCENT")
    gnc_thread = control_mgr.register(ManagedThread("GNC", lambda evt: fc.run(evt)))
    servo_thread = control_mgr.register(ManagedThread(
        "GliderServos",
        lambda evt: glider_servo_worker(
            shared,
            evt,
            use_mock=not real_servos,
            command_drogue=False,
        ),
    ))

    if real_servos and not fc.rl_active:
        sensor_mgr.stop_all()
        raise RuntimeError("RL model is not active; refusing to move real servos")

    # Force straight into GUIDED_DESCENT so GNC actually runs
    shared.update(
        state="GUIDED_DESCENT",
        glider_deployed=True,
        actuation_enabled=True,
    )
    shared.start_mission_clock()

    if real_servos:
        log.warning("REAL SERVO TEST: unload or disconnect both brake lines.")
        log.warning("Left/right channels will move; the drogue channel is inhibited.")
        for remaining in range(3, 0, -1):
            log.warning("Servo actuation starts in %d...", remaining)
            time.sleep(1.0)

    log.info("Starting GNC and %s servo worker...", "REAL" if real_servos else "mock")
    control_mgr.start_all()
    time.sleep(0.5)
    if not gnc_thread.is_alive:
        control_mgr.stop_all()
        sensor_mgr.stop_all()
        raise RuntimeError("GNC worker failed to start")
    if real_servos and not servo_thread.is_alive:
        control_mgr.stop_all()
        sensor_mgr.stop_all()
        raise RuntimeError("Real servo initialization failed; check I2C, OE, and dependencies")

    stop = threading.Event()

    def _signal(s, f):
        log.info("Ctrl+C — stopping test.")
        stop.set()

    signal.signal(signal.SIGINT, _signal)

    deadline = time.monotonic() + duration_sec
    log.info("Running for %ds — press Ctrl+C to stop early.", duration_sec)

    last_print = time.monotonic()
    while not stop.is_set() and time.monotonic() < deadline:
        if real_servos and not servo_thread.is_alive:
            log.error("Real servo worker stopped unexpectedly; ending test.")
            stop.set()
            continue
        now = time.monotonic()
        if now - last_print >= 2.0:
            snap = shared.get_snapshot()
            log.info(
                "STATE=%-18s | baro=%.1fm | left=%.1f right=%.1f | controller=%s",
                fc.state_machine.state.name,
                snap.baro_altitude,
                snap.servo_left,
                snap.servo_right,
                fc.last_controller_used,
            )
            last_print = now
        time.sleep(0.1)

    log.info("Test complete — stopping threads.")
    control_mgr.stop_all()
    sensor_mgr.stop_all()
    log.info("All threads stopped cleanly ✅")


def test_timing():
    """Quick 20 Hz loop timing test."""
    log.info("=" * 50)
    log.info("TEST: 20 Hz Loop Timing")
    log.info("=" * 50)

    import numpy as np
    times = []
    for _ in range(200):
        t = time.perf_counter()
        time.sleep(0.05)
        times.append((time.perf_counter() - t) * 1000)

    avg    = sum(times) / len(times)
    jitter = (sum((x - avg) ** 2 for x in times) / len(times)) ** 0.5
    log.info("Loop: avg=%.2fms  jitter=%.2fms  (target=50ms ±2ms)", avg, jitter)

    if jitter < 2.0:
        log.info("RESULT: PASS ✅")
    else:
        log.warning("RESULT: HIGH JITTER ⚠️ — scheduler may affect 20Hz loop")


def main():
    parser = argparse.ArgumentParser(description="Glider-only hardware test")
    parser.add_argument("--duration", type=int, default=30, help="GNC test duration in seconds (default 30)")
    parser.add_argument("--rl-only",  action="store_true", help="Only run RL inference speed test")
    parser.add_argument(
        "--real-servos",
        action="store_true",
        help="Drive real PCA9685 left/right servos with mock sensors; drogue is inhibited",
    )
    args = parser.parse_args()

    log.info("GARUDA TARSR — Glider-Only Test")
    log.info("Mock hardware: ON | All non-glider subsystems: OFF")
    log.info("")

    # Test 1: Timing
    test_timing()
    log.info("")

    # Test 2: RL inference
    rl_passed = test_rl_inference()
    log.info("")

    if args.rl_only:
        log.info("--rl-only flag set — skipping GNC loop test.")
        return

    # Test 3: Full GNC loop
    if args.real_servos and not rl_passed:
        raise SystemExit("RL benchmark failed; refusing to start real servos")

    test_gnc_loop(duration_sec=args.duration, real_servos=args.real_servos)


if __name__ == "__main__":
    main()
