"""Barometer module — mock and real-hardware implementation."""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from abc import ABC, abstractmethod

import config
from core.mission_state import MissionState
from core.shared_data import SharedData
from sensors.calibration import barometer_sea_level_pressure, load_calibration

logger = logging.getLogger(__name__)


class BaseBarometer(ABC):
    @abstractmethod
    def read(self) -> dict:
        """Return dict with altitude (metres) and pressure (hPa, optional)."""

    @abstractmethod
    def close(self) -> None:
        pass


class MockBarometer(BaseBarometer):
    """Simulated barometric altitude decreasing during descent."""

    def __init__(self) -> None:
        self._started_at = time.time()
        self._altitude = config.MOCK_START_ALTITUDE_M
        self._read_count = 0

    def read(self) -> dict:
        self._read_count += 1
        elapsed = time.time() - self._started_at
        duration = max(config.SIMULATION_DURATION_SEC, 1.0)
        if config.MOCK_GLIDER_DESCENT_ONLY:
            altitude = config.MOCK_START_ALTITUDE_M * max(0.0, 1.0 - elapsed / duration)
        elif elapsed < 2.0:
            altitude = max(0.0, config.MOCK_START_ALTITUDE_M - self._read_count * 0.5)
        elif elapsed < duration * 0.35:
            apogee_time = duration * 0.35
            climb_fraction = (elapsed - 2.0) / max(apogee_time - 2.0, 1.0)
            altitude = config.TARGET_APOGEE_AGL_M * math.sin(climb_fraction * math.pi / 2.0)
        elif elapsed < duration:
            apogee_time = duration * 0.35
            descent_fraction = (elapsed - apogee_time) / max(duration - apogee_time, 1.0)
            altitude = config.TARGET_APOGEE_AGL_M * (1.0 - descent_fraction)
        else:
            altitude = 0.0
        self._altitude = max(0.0, altitude + random.uniform(-0.3, 0.3))
        pressure = 1013.25 * (1.0 - self._altitude / 44330.0) ** 5.255
        return {"timestamp_ns": time.monotonic_ns(), "altitude": self._altitude, "pressure": pressure}

    def close(self) -> None:
        logger.debug("MockBarometer closed.")


class RealBarometer(BaseBarometer):
    """BMP388 barometer on Garud HAT SPI0 CS GPIO8."""

    def __init__(self) -> None:
        import bus_manager
        from sensors.bmp388_sensor import BMP388Sensor

        self._sensor = BMP388Sensor(bus_manager.get_spi())
        sea_level_pressure = barometer_sea_level_pressure(load_calibration())
        if sea_level_pressure is not None:
            self._sensor.set_sea_level_pressure(sea_level_pressure)
        logger.info("BMP388 initialized on SPI0 CS/GPIO%d.", config.BMP388_CS_PIN)

    def read(self) -> dict:
        reading = self._sensor.read()
        return {
            "timestamp_ns": time.monotonic_ns(),
            "altitude": reading["altitude_m"],
            "pressure": reading["pressure_hpa"],
            "temperature": reading["temperature_c"],
        }

    def close(self) -> None:
        pass


def create_barometer() -> BaseBarometer:
    if config.USE_MOCK_HARDWARE:
        return MockBarometer()
    return RealBarometer()


def barometer_worker(shared: SharedData, stop_event: threading.Event) -> None:
    """Background thread: poll barometer and update shared data."""
    try:
        baro = create_barometer()
    except Exception as exc:
        logger.error("Barometer init error: %s", exc)
        shared.update(barometer_ok=False, status="BARO_INIT_ERROR")
        shared.record_worker_error("Barometer", exc, expected_hz=config.BAROMETER_EXPECTED_HZ)
        return
    logger.info("Barometer worker started (mock=%s).", config.USE_MOCK_HARDWARE)
    last_altitude: float | None = None
    last_time: float | None = None
    filtered_velocity = 0.0

    try:
        while not stop_event.is_set():
            try:
                if config.USE_MOCK_HARDWARE and shared.is_fault_active("freeze_barometer"):
                    shared.record_event("SENSOR_STALE", "Barometer", "WARN", "Mock barometer freeze injected.")
                    stop_event.wait(0.5)
                    continue
                now = time.monotonic()
                reading = baro.read()
                if config.USE_MOCK_HARDWARE and shared.is_fault_active("barometer_drift"):
                    reading["altitude"] = float(reading["altitude"]) + 25.0
                altitude = float(reading["altitude"])
                raw_velocity = 0.0
                if last_altitude is not None and last_time is not None:
                    dt = max(now - last_time, 1e-3)
                    raw_velocity = (altitude - last_altitude) / dt
                    alpha = config.BARO_VELOCITY_FILTER_ALPHA
                    filtered_velocity = alpha * raw_velocity + (1.0 - alpha) * filtered_velocity
                last_altitude = altitude
                last_time = now
                shared.update(
                    baro_altitude=altitude,
                    vertical_velocity=filtered_velocity,
                    max_altitude=max(shared.get_snapshot().max_altitude, altitude),
                    raw_baro_pressure_hpa=reading.get("pressure", 0.0),
                    raw_baro_temperature_c=reading.get("temperature", 0.0),
                    baro_timestamp_ns=int(reading.get("timestamp_ns") or now * 1_000_000_000),
                    barometer_ok=True,
                )
                shared.record_worker_success(
                    "Barometer",
                    expected_hz=config.BAROMETER_EXPECTED_HZ,
                    reason="Barometer sample fresh.",
                    details={
                        "baro_agl_m": altitude,
                        "baro_msl_m": None,
                        "raw_vertical_velocity_mps": raw_velocity,
                        "filtered_vertical_velocity_mps": filtered_velocity,
                        "pressure_hpa": reading.get("pressure"),
                        "temperature_c": reading.get("temperature"),
                    },
                )
            except Exception as exc:
                logger.error("Barometer read error: %s", exc)
                shared.update(barometer_ok=False, status="BARO_ERROR")
                shared.record_worker_error("Barometer", exc, expected_hz=config.BAROMETER_EXPECTED_HZ)

            stop_event.wait(0.5)
    finally:
        baro.close()
        logger.info("Barometer worker stopped.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    sd = SharedData()
    evt = threading.Event()
    t = threading.Thread(target=barometer_worker, args=(sd, evt), daemon=True)
    t.start()
    for _ in range(10):
        time.sleep(0.5)
        snap = sd.get_snapshot()
        print(f"baro_alt={snap.baro_altitude:.1f} m")
    evt.set()
    t.join()
