import time
import math
import sys
import os
import hashlib
import logging
import yaml
import numpy as np
import config

try:
    import onnxruntime as ort
except ImportError:
    ort = None

# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger("FlightComputer")

# ---------------------------------------------------------------------------
# Hardware Interface Stubs
# ---------------------------------------------------------------------------
class SITL_IMU:
    def __init__(self, hw):
        self.hw = hw
    def read(self):
        imu = self.hw.read_imu()
        return imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_p, imu.gyro_q, imu.gyro_r, imu.mag_x, imu.mag_y, imu.mag_z

class SITL_Baro:
    def __init__(self, hw):
        self.hw = hw
    def read_altitude(self):
        return self.hw.read_baro().altitude

class SITL_GPS:
    """
    BUG FIX (2026-07-09): Previously hardcoded lat=18.52, lon=73.85 (Pune coordinates).
    This caused curr_x = 1,852,000m and curr_y = 7,385,000m in the flight loop.
    Now correctly passes through the simulated x/y via the 1e-5 lat/lon convention.
    """
    def __init__(self, hw):
        self.hw = hw
    def read(self):
        gps = self.hw.read_gps()
        return gps.latitude, gps.longitude, gps.altitude, gps.ground_speed, gps.heading

class SITL_Servos:
    def __init__(self, hw):
        self.hw = hw
    def write(self, left_pwm, right_pwm):
        self.hw.write_servos(left_pwm, right_pwm)

class DummyTelemetry:
    def send(self, packet):
        pass


# ---------------------------------------------------------------------------
# SharedData Integration (Garuda_TARSR Mode)
# ---------------------------------------------------------------------------
class SharedIMU:
    def __init__(self, shared):
        self.shared = shared

    def read(self):
        snap = self.shared.get_snapshot()
        return (snap.raw_accel_x, snap.raw_accel_y, snap.raw_accel_z,
                snap.raw_gyro_x, snap.raw_gyro_y, snap.raw_gyro_z,
                snap.raw_mag_x, snap.raw_mag_y, snap.raw_mag_z)

    def read_attitude(self):
        # We rely on gnc_worker's EKF/Madgwick, so this can return 0s if needed, 
        # or we could return the already fused AHRS data. 
        # gnc_worker.py calculates its own attitude.
        snap = self.shared.get_snapshot()
        return snap.roll, snap.pitch, snap.yaw, snap.raw_gyro_z


class SharedBaro:
    def __init__(self, shared):
        self.shared = shared

    def read_altitude(self):
        return self.shared.get_snapshot().baro_altitude


class SharedGPS:
    def __init__(self, shared):
        self.shared = shared
        self.last_timestamp_ns = 0
        self.last_fix_ok = False

    def read(self):
        snap = self.shared.get_snapshot()
        self.last_timestamp_ns = snap.gps_timestamp_ns
        self.last_fix_ok = snap.gps_ok
        return (snap.latitude, snap.longitude, snap.gps_altitude,
                snap.gps_ground_speed_mps, math.radians(snap.gps_course_deg))


class SharedServos:
    def __init__(self, shared):
        self.shared = shared

    def write(self, left, right):
        self.shared.update(
            servo_left=left,
            servo_right=right,
        )


# ---------------------------------------------------------------------------
# FlightComputer
# ---------------------------------------------------------------------------
class FlightComputer:
    # Physical bounds for post-rescaling output validation
    DELTA_A_MIN = -30.0
    DELTA_A_MAX =  30.0
    DELTA_S_MIN =   0.0
    DELTA_S_MAX =  30.0

    def __init__(
        self,
        shared=None,
        use_simulator=True,
        drop_height: float = 0.0,
        enable_state_persistence: bool = True,
        external_state_authority: bool = False,
    ):
        log.info("Initializing Flight Computer...")

        # --- Resolve glider_gnc module path (lazy, to avoid package name collisions) ---
        _here = os.path.abspath(os.path.dirname(__file__))
        _gnc_root = os.path.abspath(os.path.join(_here, '..', '..', 'System001', 'glider_gnc'))
        if os.path.isdir(_gnc_root) and _gnc_root not in sys.path:
            sys.path.insert(0, _gnc_root)
        if _here not in sys.path:
            sys.path.insert(0, _here)

        from estimation.madgwick import MadgwickFilter as _MadgwickFilter
        from estimation.ekf_altitude import EKFAltitude as _EKFAltitude
        from guidance.heading_pid import HeadingPID as _HeadingPID
        from state_machine.flight_states import StateMachine as _StateMachine, FlightState as _FlightState
        from state_machine.state_persistence import (
            StateSnapshot, STATE_WRITE_INTERVAL_S,
            write_state, load_state, delete_state,
        )
        from estimation.wind_estimator import WindEstimatorRLS as _WindEstimatorRLS

        # Store references so the rest of the class can use them
        self._MadgwickFilter = _MadgwickFilter
        self._EKFAltitude = _EKFAltitude
        self._HeadingPID = _HeadingPID
        self._StateMachine = _StateMachine
        self.FlightState = _FlightState
        self._StateSnapshot = StateSnapshot
        self._STATE_WRITE_INTERVAL_S = STATE_WRITE_INTERVAL_S
        self._write_state = write_state
        self._load_state = load_state
        self._delete_state = delete_state
        self._WindEstimatorRLS = _WindEstimatorRLS

        self.shared = shared
        self.external_state_authority = bool(external_state_authority and shared is not None)
        self.enable_state_persistence = enable_state_persistence and not self.external_state_authority
        self.dt = 0.05  # 20 Hz loop

        if self.shared is not None:
            # ---------------------------------------------------------------
            # Garuda_TARSR INTEGRATED mode — hardware is managed by the
            # sensor worker threads; we just read from SharedData.
            # IMPORTANT: disable use_simulator so the SITL run-loop block
            # (which needs sim_wind / sim_dynamics) is never entered.
            # ---------------------------------------------------------------
            self.use_simulator = False
            log.info("--> INTEGRATED (SharedData) Mode ACTIVE")
            self.imu    = SharedIMU(self.shared)
            self.baro   = SharedBaro(self.shared)
            self.gps    = SharedGPS(self.shared)
            self.servos = SharedServos(self.shared)
        elif use_simulator:
            self.use_simulator = True
            log.info("--> Software-In-The-Loop (SITL) Mode ACTIVE")
            from sim.dynamics import GliderDynamics
            from sim.wind_model import WindModel
            from hw_interface.simulated_hardware import SimulatedHardware
            self.sim_dynamics = GliderDynamics(-1000.0, -1000.0, 0.0, math.radians(45))
            self.sim_wind = WindModel(2.0, math.radians(90))
            self.sim_hw = SimulatedHardware(self.sim_dynamics)

            self.imu    = SITL_IMU(self.sim_hw)
            self.baro   = SITL_Baro(self.sim_hw)
            self.gps    = SITL_GPS(self.sim_hw)
            self.servos = SITL_Servos(self.sim_hw)
        else:
            self.use_simulator = False
            log.info("--> REAL FLIGHT Mode ACTIVE")
            from sensors.drivers import BNO085, BMP388, GPS, INA219
            from hw_interface.real_hardware import RealHardware
            self._real_hw = RealHardware()
            self._bno085  = BNO085()                  # SPI: CS=GPIO5, RST=GPIO6
            self._bmp388  = BMP388(cs_pin_name='D22')  # SPI: CS=GPIO22
            self._gps_hw  = GPS(
                port='/dev/ttyAMA0',
                baudrate=9600,
            )
            self._ina219  = INA219(address=0x41)

            # Pre-flight calibration check -- wait for BNO085 to settle
            log.info("[HW] Waiting for BNO085 to reach stable state...")
            import time as _time
            for _ in range(60):   # up to 30 seconds
                if self._bno085.calibration_ok():
                    log.info("[HW] BNO085 stable -- proceeding.")
                    break
                _time.sleep(0.5)
            else:
                log.warning("[HW] BNO085 calibration timeout -- proceeding anyway.")

            # Thin wrappers so the rest of run() doesn't need special-casing
            class _RealIMU:
                def __init__(self_i, bno):
                    self_i._bno = bno
                def read(self_i):
                    d = self_i._bno.read()
                    if d is None:
                        return (0, 0, 9.81, 0, 0, 0, 0, 0, 0)
                    return (d.accel_x, d.accel_y, d.accel_z,
                            d.gyro_p,  d.gyro_q,  d.gyro_r,
                            d.mag_x,   d.mag_y,   d.mag_z)
                def read_attitude(self_i):
                    """Returns (roll, pitch, yaw, gz) directly from BNO085 fusion."""
                    d = self_i._bno.read()
                    if d is None:
                        return 0.0, 0.0, 0.0, 0.0
                    return d.roll, d.pitch, d.yaw, d.gyro_r

            class _RealBaro:
                def __init__(self_b, bmp):
                    self_b._bmp = bmp
                def read_altitude(self_b):
                    d = self_b._bmp.read()
                    return d.altitude if d else 0.0

            class _RealGPS:
                def __init__(self_g, gps):
                    self_g._gps = gps
                def read(self_g):
                    d = self_g._gps.read()
                    if d is None:
                        return 0.0, 0.0, 0.0, 0.0, 0.0
                    return d.latitude, d.longitude, d.altitude, d.ground_speed, d.heading

            class _RealServos:
                def __init__(self_s, hw):
                    self_s._hw = hw
                def write(self_s, left, right):
                    self_s._hw.write_servos(left, right)

            self.imu    = _RealIMU(self._bno085)
            self.baro   = _RealBaro(self._bmp388)
            self.gps    = _RealGPS(self._gps_hw)
            self.servos = _RealServos(self._real_hw)

        self.telemetry = DummyTelemetry()

        # Read mission target from config and convert to our 1e-5 projection
        # Edit config/gains.yaml section 14 on launch day -- no code change needed.
        _gains_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "gains.yaml")
        if not os.path.exists(_gains_path):
            _gains_path = "config/gains.yaml"  # fallback for standalone run
        with open(_gains_path, "r") as _f:
            _cfg = yaml.safe_load(_f)
        mission_cfg = _cfg.get('mission', {})
        target_lat  = mission_cfg.get('target_latitude',  18.653862)
        target_lon  = mission_cfg.get('target_longitude', 73.762826)
        # Store in decimal degrees for NUE bearing calculation
        self.target_lat = target_lat
        self.target_lon = target_lon
        # Legacy 1e-5 projection kept for RL observation backward-compatibility
        self.target_x = target_lat / 1e-5
        self.target_y = target_lon / 1e-5
        log.info("[MISSION] Target: lat=%.6f lon=%.6f", target_lat, target_lon)

        self.att_filter     = self._MadgwickFilter(beta=0.1)
        _ground_alt         = self.baro.read_altitude()   # read ONCE — shared reference
        self.ekf_alt        = self._EKFAltitude(self.dt, initial_alt=_ground_alt)
        self.heading_pid    = self._HeadingPID(kp=10.0, ki=0.1, kd=1.0, output_limit=30.0)
        self.state_machine  = self._StateMachine(ground_altitude=_ground_alt - drop_height)
        self.wind_estimator = self._WindEstimatorRLS()
        self.prev_delta_a   = 0.0
        self.prev_delta_s   = 0.0
        self.last_controller_used = "NEUTRAL"
        self.controller_counts = {"RL": 0, "PID": 0, "SALVAGE": 0, "NEUTRAL": 0}
        self.loop_count = 0
        self.loop_overrun_count = 0
        self.first_loop_monotonic = 0.0
        self.last_loop_monotonic = 0.0
        self.servo_min = 180.0
        self.servo_max = 0.0

        if drop_height > 0.0:
            log.info("[DROP-TEST] Ground altitude offset by -%.1f m (true AGL baseline corrected)", drop_height)

        self._last_gps_time   = time.time()
        self._last_state_write = time.time()   # throttle .state writes

        # ---------------------------------------------------------------
        # Boot-time reset recovery
        # ---------------------------------------------------------------
        if not self.use_simulator and self.enable_state_persistence:
            snapshot = self._load_state()
            if snapshot is not None:
                log.warning("[RECOVERY] Resuming from .state: state=%s  drogue=%s  alt=%.1f m",
                            snapshot.flight_state, snapshot.drogue_fired,
                            snapshot.last_altitude_m)
                # Re-seed the state machine to the recovered state
                self.state_machine.force_state(snapshot.flight_state)
                # Safety: lock out drogue if it was already fired
                if snapshot.drogue_fired:
                    self.state_machine.lock_drogue()
                    log.warning("[RECOVERY] Drogue LOCKED OUT — was fired before reset.")
                # Seed EKF with last known altitude so estimates are sane
                self.ekf_alt.set_altitude(snapshot.last_altitude_m,
                                          snapshot.last_velocity_ms)
                log.info("[RECOVERY] EKF seeded: alt=%.1f m  vel=%.2f m/s",
                         snapshot.last_altitude_m, snapshot.last_velocity_ms)
            else:
                log.info("[BOOT] Cold start — no valid .state file.")

        with open("config/gains.yaml", "r") as f:
            self.config = yaml.safe_load(f)

        self.glide_ratio         = self.config['airframe']['glide_ratio']
        self.gps_timeout_s       = self.config['telemetry']['gps_staleness_timeout_ms'] / 1000.0
        self.inference_timeout_s = self.config['rl']['inference_timeout_ms'] / 1000.0

        self.rl_session = None
        self.rl_active  = False
        self._load_rl_model()

    def _sha256_short(self, path, chars=8):
        h = hashlib.sha256()
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(65536), b''):
                h.update(chunk)
        return h.hexdigest()[:chars]

    def _load_rl_model(self):
        if ort is None:
            log.warning("[RL] onnxruntime not installed -- PID-only mode.")
            return

        model_path    = self.config['rl']['onnx_model_path']
        expected_obs  = self.config['rl']['obs_dim']
        expected_act  = self.config['rl']['action_dim']

        if not os.path.exists(model_path):
            log.warning(f"[RL] ONNX not found at '{model_path}' -- PID-only mode.")
            return

        try:
            session = ort.InferenceSession(model_path)
            inp = session.get_inputs()[0]
            out = session.get_outputs()[0]
            sha = self._sha256_short(model_path)
            log.info(
                f"[RL] Model loaded | path={model_path} | sha256={sha} | "
                f"in_shape={inp.shape} | out_shape={out.shape}"
            )
            if inp.shape != [1, expected_obs]:
                raise ValueError(f"Input shape mismatch: expected [1,{expected_obs}], got {inp.shape}")
            if out.shape != [1, expected_act]:
                raise ValueError(f"Output shape mismatch: expected [1,{expected_act}], got {out.shape}")

            # Prime ONNX Runtime before the flight watchdog is active. Session
            # setup and the first kernel invocation can be much slower than
            # steady-state inference on resource-constrained hardware.
            warmup_obs = np.zeros((1, expected_obs), dtype=np.float32)
            for _ in range(3):
                session.run(None, {inp.name: warmup_obs})

            self.rl_session    = session
            self.rl_input_name = inp.name
            self.rl_active     = True
            log.info("[RL] Shape check PASSED -- RL is PRIMARY controller.")
        except Exception as e:
            log.error(f"[RL] Load failed: {e} -- falling back to PID.")
            self.rl_session = None
            self.rl_active  = False

    def _obs_from_state(self, curr_x, curr_y, target_bearing, dist,
                        alt_excess, pitch, roll, yaw_rate,
                        gps_speed, gps_heading, altitude):
        """
        16D observation builder. Must match training/env.py _get_obs() exactly.
        obs[15] time_to_impact is capped at 2.0 (preserved from env.py line 116).
        """
        heading_err = (target_bearing - gps_heading + math.pi) % (2 * math.pi) - math.pi

        wx, wy = self.wind_estimator.get_wind_estimate()
        wind_speed = math.hypot(wx, wy)
        wind_dir   = math.atan2(wy, wx)

        gvx = gps_speed * math.cos(gps_heading)
        gvy = gps_speed * math.sin(gps_heading)

        course_over_ground = math.atan2(gvy, gvx)
        track_err = (target_bearing - course_over_ground + math.pi) % (2 * math.pi) - math.pi

        lateral_drift      = -gvx * math.sin(target_bearing) + gvy * math.cos(target_bearing)
        lateral_drift_norm = lateral_drift / 8.0

        airspeed_approx    = math.hypot(gvx - wx, gvy - wy)
        sink_rate          = max(airspeed_approx / self.glide_ratio, 0.1)
        time_to_impact_norm = min(altitude / sink_rate / 200.0, 2.0)

        return np.array([[
            math.sin(heading_err),     # obs[0]
            math.cos(heading_err),     # obs[1]
            dist / 1000.0,             # obs[2]
            alt_excess / 1000.0,       # obs[3]
            wind_speed / 10.0,         # obs[4]
            math.sin(wind_dir),        # obs[5]
            math.cos(wind_dir),        # obs[6]
            pitch / 0.5,               # obs[7]
            roll / 0.5,                # obs[8]
            yaw_rate / 0.5,            # obs[9]
            self.prev_delta_a / 30.0,  # obs[10]
            self.prev_delta_s / 30.0,  # obs[11]
            math.sin(track_err),       # obs[12]
            math.cos(track_err),       # obs[13]
            lateral_drift_norm,        # obs[14]
            time_to_impact_norm,       # obs[15]
        ]], dtype=np.float32)

    def _write_state_snapshot(self) -> None:
        """Write current flight state to .state file for reset recovery."""
        try:
            sm = self.state_machine
            snapshot = self._StateSnapshot(
                flight_state      = sm.state.name,
                ground_altitude_m = sm.ground_altitude,
                drogue_fired      = sm.drogue_fired,
                last_altitude_m   = self.ekf_alt.altitude,
                last_velocity_ms  = self.ekf_alt.vertical_velocity,
                target_lat        = self.target_lat,
                target_lon        = self.target_lon,
                rl_active         = self.rl_active,
            )
            self._write_state(snapshot)
        except Exception as e:
            log.warning("[STATE] Failed to write snapshot: %s", e)

    def _validate_and_rescale(self, raw):
        """
        Rescales raw tanh outputs to physical units and validates.
        Raises ValueError on NaN, Inf, or out-of-range values.
        """
        r0, r1 = float(raw[0]), float(raw[1])
        if not math.isfinite(r0) or not math.isfinite(r1):
            raise ValueError(f"NaN/Inf in ONNX output: [{r0}, {r1}]")
        # Tanh outputs may exceed +/-1 by a few floating-point ULPs. Accept a
        # tiny numerical tolerance, clamp it, and still reject genuine model
        # contract violations.
        tolerance = 1e-5
        if not (-1.0 - tolerance <= r0 <= 1.0 + tolerance):
            raise ValueError(f"raw delta_a={r0:.8f} outside tanh range")
        if not (-1.0 - tolerance <= r1 <= 1.0 + tolerance):
            raise ValueError(f"raw delta_s={r1:.8f} outside tanh range")
        r0 = max(-1.0, min(1.0, r0))
        r1 = max(-1.0, min(1.0, r1))
        delta_a = r0 * 30.0
        delta_s = (r1 + 1.0) / 2.0 * 30.0
        if not (self.DELTA_A_MIN <= delta_a <= self.DELTA_A_MAX):
            raise ValueError(f"delta_a={delta_a:.2f} out of range")
        if not (self.DELTA_S_MIN <= delta_s <= self.DELTA_S_MAX):
            raise ValueError(f"delta_s={delta_s:.2f} out of range")
        return delta_a, delta_s

    def _rl_inference(self, obs):
        """Runs ONNX inference with watchdog. Raises RuntimeError on timeout."""
        t0  = time.perf_counter()
        raw = self.rl_session.run(None, {self.rl_input_name: obs})[0][0]
        elapsed = time.perf_counter() - t0
        if elapsed > self.inference_timeout_s:
            raise RuntimeError(f"Inference timeout: {elapsed*1000:.1f}ms > {self.inference_timeout_s*1000:.0f}ms")
        return raw

    def run(self, stop_event=None):
        log.info("Starting 20Hz Flight Loop...")
        frame_id = 0

        while stop_event is None or not stop_event.is_set():
            loop_start = time.time()

            if self.use_simulator:
                wx, wy = self.sim_wind.get_wind()
                self.sim_dynamics.step(self.dt, wx, wy)
                if self.sim_dynamics.altitude <= 0:
                    log.info("--> Simulation Finished. Glider has landed.")
                    break

            # -- Periodic .state write (every STATE_WRITE_INTERVAL_S seconds)
            now = time.time()
            if (
                not self.use_simulator
                and self.enable_state_persistence
                and (now - self._last_state_write) >= self._STATE_WRITE_INTERVAL_S
            ):
                self._write_state_snapshot()
                self._last_state_write = now
            # 1. Read Sensors
            ax, ay, az, gx, gy, gz, mx, my, mz = self.imu.read()
            baro_alt = self.baro.read_altitude()
            lat, lon, gps_alt, gps_speed, gps_heading = self.gps.read()

            # Legacy 1e-5 projection -- kept only for RL observation space compatibility
            curr_x = lat / 1e-5
            curr_y = lon / 1e-5

            # NUE local frame: convert lat/lon difference to real metres
            # This is used by the PID path for geometrically correct bearing + distance.
            _cos_lat     = math.cos(math.radians(lat))
            _north_m     = (self.target_lat - lat) * 111320.0
            _east_m      = (self.target_lon - lon) * 111320.0 * _cos_lat
            _nue_dist_m  = math.hypot(_north_m, _east_m)
            _nue_bearing = math.atan2(_east_m, _north_m)

            now = time.time()
            gps_timestamp_ns = getattr(self.gps, "last_timestamp_ns", 0)
            if gps_timestamp_ns > 0:
                gps_age_s = max(0.0, (time.monotonic_ns() - gps_timestamp_ns) / 1e9)
                gps_fresh = (
                    getattr(self.gps, "last_fix_ok", True)
                    and gps_age_s <= self.gps_timeout_s
                )
            else:
                # SITL and standalone hardware adapters return a fresh sample
                # directly on every call and do not expose a timestamp.
                gps_fresh = True
            if not gps_fresh:
                log.warning("[SALVAGE] GPS stale >%.0fms -- wings level, straight descent.",
                            self.gps_timeout_s * 1000)
            else:
                self._last_gps_time = now

            # Cross-sensor sanity check: barometer vs GPS altitude
            if gps_alt != 0.0 and abs(baro_alt - gps_alt) > 100.0:
                log.warning("[SANITY] Baro alt %.1fm vs GPS alt %.1fm disagree by >100m!",
                            baro_alt, gps_alt)

            v_gx = gps_speed * math.cos(gps_heading)
            v_gy = gps_speed * math.sin(gps_heading)
            self.wind_estimator.update(v_gx, v_gy, gps_heading)

            # 2. State Estimation
            self.att_filter.update(ax, ay, az, gx, gy, gz, mx, my, mz, self.dt)
            roll, pitch, yaw = self.att_filter.get_euler_angles()

            accel_z_earth_down = (-math.sin(pitch) * ax
                                  + math.sin(roll) * math.cos(pitch) * ay
                                  + math.cos(roll) * math.cos(pitch) * az)
            self.ekf_alt.predict(9.81 - accel_z_earth_down)
            self.ekf_alt.update_baro(baro_alt)

            agl_altitude = max(0.0, self.ekf_alt.altitude - self.state_machine.ground_altitude)

            # Production has one state authority: core.FlightStateController.
            # The legacy internal state machine remains available for SITL and
            # deterministic mock tests only.
            if self.external_state_authority:
                shared_snap = self.shared.get_snapshot()
                try:
                    state = self.FlightState[shared_snap.state]
                except KeyError:
                    state = self.FlightState.BOOST
                guidance_allowed = bool(shared_snap.actuation_enabled)
                drogue_fired = bool(shared_snap.glider_deployed)
            else:
                state = self.state_machine.update(self.ekf_alt.altitude, self.ekf_alt.vertical_velocity)
                guidance_allowed = state == self.FlightState.GUIDED_DESCENT
                drogue_fired = self.state_machine.drogue_fired
            if self.shared is not None:
                drogue_angle = (
                    config.GLIDER_DROGUE_DEPLOY_ANGLE
                    if drogue_fired
                    else config.GLIDER_DROGUE_SAFE_ANGLE
                )
                self.shared.update(servo_drogue=drogue_angle)

            # 4. Guidance
            left_servo  = 90.0
            right_servo = 90.0
            controller_used = "NEUTRAL"

            if state == self.FlightState.GUIDED_DESCENT and guidance_allowed:
                # RL uses legacy 1e-5 coordinate projection for backward compat with trained model
                aim_x = self.target_x
                aim_y = self.target_y
                target_bearing = math.atan2(aim_y - curr_y, aim_x - curr_x)
                dist       = math.hypot(aim_x - curr_x, aim_y - curr_y)
                alt_excess = agl_altitude - (_nue_dist_m / self.glide_ratio)

                rl_succeeded = False
                if self.rl_active and gps_fresh:
                    try:
                        obs = self._obs_from_state(
                            curr_x, curr_y, target_bearing, dist, alt_excess,
                            pitch, roll, gz,
                            gps_speed, gps_heading, agl_altitude
                        )
                        raw     = self._rl_inference(obs)
                        delta_a, delta_s = self._validate_and_rescale(raw)
                        rl_succeeded    = True
                        controller_used = "RL"
                    except Exception as e:
                        log.warning(f"[FALLBACK] RL exception: {e} -- engaging PID.")

                if not rl_succeeded:
                    if not gps_fresh:
                        # SALVAGE MODE: GPS lost -- wings level, let gravity do the rest
                        delta_a = 0.0
                        delta_s = 0.0
                        controller_used = "SALVAGE"
                    else:
                        # PID guidance using NUE local frame bearing (geometrically correct)
                        gains = self.config['gain_schedules']
                        if agl_altitude > gains['cruise']['min_alt_agl']:
                            self.heading_pid.kp = gains['cruise']['heading_kp']
                            self.heading_pid.ki = gains['cruise']['heading_ki']
                            self.heading_pid.kd = gains['cruise']['heading_kd']
                        elif agl_altitude > gains['approach']['min_alt_agl']:
                            self.heading_pid.kp = gains['approach']['heading_kp']
                            self.heading_pid.ki = gains['approach']['heading_ki']
                            self.heading_pid.kd = gains['approach']['heading_kd']
                        else:
                            # Final approach: reset integral to prevent windup slamming servos
                            self.heading_pid.integral = 0.0
                            self.heading_pid.kp = gains['final']['heading_kp']
                            self.heading_pid.ki = gains['final']['heading_ki']
                            self.heading_pid.kd = gains['final']['heading_kd']
                        # PID uses NUE bearing for correct metric-space heading error
                        delta_a = self.heading_pid.compute(_nue_bearing, gps_heading, self.dt)
                        # Final approach: cap aileron to ±output_limit_deg to prevent oscillation
                        if agl_altitude <= gains['approach']['min_alt_agl']:
                            _final_limit = gains['final'].get('output_limit_deg', 10.0)
                            delta_a = max(-_final_limit, min(_final_limit, delta_a))
                        delta_s = 30.0 if agl_altitude < 10.0 else 0.0
                        controller_used = "PID"

                self.prev_delta_a = delta_a
                self.prev_delta_s = delta_s

                left_servo  = max(60.0, min(120.0, 90.0 + delta_s - delta_a))
                right_servo = max(60.0, min(120.0, 90.0 + delta_s + delta_a))

            self.servos.write(left_servo, right_servo)
            if self.shared is not None:
                self.shared.update(
                    gnc_ok=True,
                    gnc_timestamp_ns=time.monotonic_ns(),
                )
            self.last_controller_used = controller_used
            self.controller_counts[controller_used] = self.controller_counts.get(controller_used, 0) + 1
            self.loop_count += 1
            loop_monotonic = time.monotonic()
            if self.first_loop_monotonic == 0.0:
                self.first_loop_monotonic = loop_monotonic
            self.last_loop_monotonic = loop_monotonic
            self.servo_min = min(self.servo_min, left_servo, right_servo)
            self.servo_max = max(self.servo_max, left_servo, right_servo)

            # 5. Telemetry
            packet = (f"{frame_id},{loop_start:.2f},{lat},{lon},{gps_alt},{baro_alt},"
                      f"{math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f}")
            self.telemetry.send(packet)

            if frame_id % 20 == 0:
                log.info(
                    f"[{controller_used:<6}] STATE:{state.name:<22} | "
                    f"AGL:{agl_altitude:>6.1f}m | "
                    f"DIST:{_nue_dist_m:>7.1f}m | "
                    f"SRV L:{left_servo:>5.1f} R:{right_servo:>5.1f}"
                )

            # 6. Loop timing enforcement
            elapsed = time.time() - loop_start
            if elapsed > self.dt:
                self.loop_overrun_count += 1
                log.warning(f"[WATCHDOG] Loop overrun: {elapsed*1000:.1f}ms > {self.dt*1000:.0f}ms budget")
            else:
                time.sleep(self.dt - elapsed)

            frame_id += 1


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="CAN-7U-SAT Flight Computer")
    parser.add_argument("--sitl", action="store_true", help="Run in SITL mode")
    args = parser.parse_args()
    fc = FlightComputer(use_simulator=args.sitl)
    try:
        fc.run()
    except KeyboardInterrupt:
        log.info("Flight Computer shutdown safely.")
