"""Bench-test the real BNO085, BMP388, and optional NEO-M8N on Raspberry Pi.

Run from the repository root:
    python3 hardware_tests/test_sensor_stack.py --seconds 20
    python3 hardware_tests/test_sensor_stack.py --seconds 60 --gps

This script reads sensors only. It never commands the PCA9685 or servos.
"""

from __future__ import annotations

import argparse
import math
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import config  # noqa: E402
from sensors.barometer import RealBarometer  # noqa: E402
from sensors.gps import RealGPS  # noqa: E402
from sensors.imu import RealIMU  # noqa: E402


def is_raspberry_pi() -> bool:
    try:
        return "Raspberry Pi" in Path("/proc/cpuinfo").read_text(encoding="utf-8")
    except OSError:
        machine = platform.machine().lower()
        return sys.platform.startswith("linux") and machine in {"armv7l", "aarch64", "arm64"}


def print_wiring() -> None:
    print("Sensor wiring expected by config.py:")
    print(f"  BNO085 : I2C1 SDA=GPIO{config.I2C_SDA_PIN} pin 3")
    print(f"           I2C1 SCL=GPIO{config.I2C_SCL_PIN} pin 5")
    print(f"           address=0x{config.BNO085_I2C_ADDRESS:02X}")
    print(f"  BMP388 : SPI0 SCLK=GPIO{config.SPI_SCLK_PIN} pin 23")
    print(f"           MOSI=GPIO{config.SPI_MOSI_PIN} pin 19")
    print(f"           MISO=GPIO{config.SPI_MISO_PIN} pin 21")
    print(f"           CS=GPIO{config.BMP388_CS_PIN} pin 24")
    print(f"  GPS    : {config.GPS_PORT} at {config.GPS_BAUDRATE} baud")
    print()


def scan_i2c() -> bool | None:
    if not shutil.which("i2cdetect"):
        print("[WARN] i2cdetect unavailable; install package i2c-tools.")
        return None
    result = subprocess.run(
        ["i2cdetect", "-y", "1"],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    print(result.stdout)
    if result.returncode != 0:
        print(f"[FAIL] i2cdetect exited with {result.returncode}: {result.stderr.strip()}")
        return False
    address = f"{config.BNO085_I2C_ADDRESS:02x}"
    found = address in result.stdout.lower()
    print(f"[{'PASS' if found else 'WARN'}] BNO085 address 0x{address.upper()} "
          f"{'detected' if found else 'not visible in scan'}.")
    return found


def finite_vector(values, length: int) -> bool:
    return values is not None and len(values) == length and all(
        math.isfinite(float(value)) for value in values
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="GARUDA real sensor stack test")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=2.0, help="Display rate in Hz")
    parser.add_argument("--gps", action="store_true", help="Also require a real GPS fix")
    args = parser.parse_args()

    print("=" * 68)
    print(" GARUD SENSOR STACK: BNO085 + BMP388" + (" + NEO-M8N" if args.gps else ""))
    print("=" * 68)
    print_wiring()
    if not is_raspberry_pi():
        print("[WARN] Raspberry Pi was not detected; hardware access will probably fail.")
    scan_i2c()

    config.USE_MOCK_HARDWARE = False
    devices = {}
    failures = []
    try:
        devices["imu"] = RealIMU()
        print("[PASS] BNO085 initialized.")
    except Exception as exc:
        failures.append(f"BNO085 initialization: {exc}")
        print(f"[FAIL] BNO085 initialization: {exc}")

    try:
        devices["barometer"] = RealBarometer()
        print("[PASS] BMP388 initialized.")
    except Exception as exc:
        failures.append(f"BMP388 initialization: {exc}")
        print(f"[FAIL] BMP388 initialization: {exc}")

    if args.gps:
        try:
            devices["gps"] = RealGPS()
            print(f"[PASS] NEO-M8N UART opened on {config.GPS_PORT}.")
        except Exception as exc:
            failures.append(f"GPS initialization: {exc}")
            print(f"[FAIL] GPS initialization: {exc}")

    required = {"imu", "barometer"} | ({"gps"} if args.gps else set())
    if not required.issubset(devices):
        for device in reversed(list(devices.values())):
            device.close()
        print("[FAIL] Required devices did not initialize.")
        return 1

    period = 1.0 / max(args.rate, 0.1)
    deadline = time.monotonic() + max(args.seconds, period)
    samples = 0
    good_imu = good_baro = good_gps = 0
    try:
        while time.monotonic() < deadline:
            samples += 1
            imu = devices["imu"].read()
            baro = devices["barometer"].read()

            imu_ok = (
                finite_vector(imu.get("accel_mps2"), 3)
                and finite_vector(imu.get("gyro_rads"), 3)
                and finite_vector(imu.get("quaternion"), 4)
            )
            baro_ok = all(
                math.isfinite(float(baro[key]))
                for key in ("altitude", "pressure", "temperature")
            )
            good_imu += int(imu_ok)
            good_baro += int(baro_ok)

            gps_text = ""
            if args.gps:
                gps = devices["gps"].read()
                gps_ok = bool(gps.get("fix_ok"))
                good_gps += int(gps_ok)
                gps_text = (
                    f" gps={'FIX' if gps_ok else 'WAIT'} sats={gps.get('satellites') or 0}"
                    f" lat={gps.get('latitude', 0.0):.6f} lon={gps.get('longitude', 0.0):.6f}"
                )

            accel = imu["accel_mps2"]
            print(
                f"#{samples:03d} bno={'OK' if imu_ok else 'BAD'} "
                f"accel=({accel[0]:+.2f},{accel[1]:+.2f},{accel[2]:+.2f}) "
                f"rpy=({imu['roll']:+.1f},{imu['pitch']:+.1f},{imu['yaw']:+.1f}) "
                f"bmp={'OK' if baro_ok else 'BAD'} alt={baro['altitude']:.2f}m "
                f"p={baro['pressure']:.2f}hPa t={baro['temperature']:.2f}C{gps_text}"
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as exc:
        failures.append(f"Runtime read: {exc}")
        print(f"[FAIL] Runtime sensor read: {exc}")
    finally:
        for device in reversed(list(devices.values())):
            try:
                device.close()
            except Exception:
                pass

    checks = {
        "BNO085 valid samples": samples > 0 and good_imu == samples,
        "BMP388 valid samples": samples > 0 and good_baro == samples,
    }
    if args.gps:
        checks["NEO-M8N obtained fix"] = good_gps > 0

    print("\n" + "=" * 68)
    print(" SENSOR RESULTS")
    print("=" * 68)
    for name, passed in checks.items():
        print(f" {'PASS' if passed else 'FAIL':4} | {name}")
    for failure in failures:
        print(f" FAIL | {failure}")
    passed = all(checks.values()) and not failures
    print("-" * 68)
    print(" OVERALL:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
