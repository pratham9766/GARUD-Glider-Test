"""BNO085 wrapper for the tested GARUDA I2C1 sensor setup."""

from __future__ import annotations

import time

import config


class BNO085Sensor:
    """Read raw vectors and the native BNO085 quaternion over I2C."""

    def __init__(self, i2c_bus, address: int = config.BNO085_I2C_ADDRESS) -> None:
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_LINEAR_ACCELERATION,
            BNO_REPORT_MAGNETOMETER,
        )
        import adafruit_bno08x as bno08x
        from adafruit_bno08x.i2c import BNO08X_I2C

        self.bno = BNO08X_I2C(i2c_bus, address=address)
        report_name = (
            "BNO_REPORT_GAME_ROTATION_VECTOR"
            if config.BNO085_ROTATION_MODE == "GAME_ROTATION_VECTOR"
            else "BNO_REPORT_ROTATION_VECTOR"
        )
        rotation_report = getattr(bno08x, report_name)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        if config.AHRS_USE_MAGNETOMETER:
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(rotation_report)

    def read(self) -> dict:
        accel = tuple(float(value) for value in self.bno.acceleration)
        gyro = tuple(float(value) for value in self.bno.gyro)
        linear_accel = tuple(float(value) for value in self.bno.linear_acceleration)
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        magnetometer = None
        if config.AHRS_USE_MAGNETOMETER:
            try:
                magnetometer = tuple(float(value) for value in self.bno.magnetic)
            except Exception:
                magnetometer = None

        return {
            "timestamp_ns": time.monotonic_ns(),
            "accel_mps2": accel,
            "gyro_rads": gyro,
            "mag_ut": magnetometer,
            "linear_accel_mps2": linear_accel,
            "quaternion": (
                float(quat_i),
                float(quat_j),
                float(quat_k),
                float(quat_real),
            ),
            "accuracy_rad": getattr(self.bno, "accuracy", None),
            "calibration_status": getattr(self.bno, "calibration_status", None),
        }
