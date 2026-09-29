"""
GPS module — mock and real-hardware placeholder.

When USE_MOCK_HARDWARE is True, generates simulated coordinates near Pune.
Replace MockGPS with a real NMEA reader when hardware arrives.
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from abc import ABC, abstractmethod

import config
from core.shared_data import SharedData

logger = logging.getLogger(__name__)


class BaseGPS(ABC):
    """Abstract GPS interface."""

    @abstractmethod
    def read(self) -> dict:
        """Return dict with latitude, longitude, altitude, fix_ok."""

    @abstractmethod
    def close(self) -> None:
        """Release hardware resources."""


class MockGPS(BaseGPS):
    """Simulated GPS drifting around Pune reference coordinates."""

    def __init__(self) -> None:
        self._lat = config.MOCK_GPS_LAT
        self._lon = config.MOCK_GPS_LON
        self._alt = config.MOCK_START_ALTITUDE_M
        self._step = 0
        self._started_at = time.monotonic()

    def read(self) -> dict:
        self._step += 1
        # Gentle drift simulating movement during descent
        angle = self._step * 0.05
        self._lat += 0.00001 * math.sin(angle) + random.uniform(-0.000005, 0.000005)
        self._lon += 0.00001 * math.cos(angle) + random.uniform(-0.000005, 0.000005)
        if config.MOCK_GLIDER_DESCENT_ONLY:
            elapsed = time.monotonic() - self._started_at
            duration = max(config.SIMULATION_DURATION_SEC, 1.0)
            self._alt = config.MOCK_START_ALTITUDE_M * max(0.0, 1.0 - elapsed / duration)
        else:
            self._alt = max(0.0, self._alt - random.uniform(0.0, 0.5))
        return {
            "timestamp_ns": time.monotonic_ns(),
            "latitude": self._lat,
            "longitude": self._lon,
            "altitude": self._alt,
            "fix_ok": True,
            "fix_type": "3D",
            "satellites": 12,
            "hdop": 0.9,
            "ground_speed_mps": 4.0,
            "course_deg": (angle * 180.0 / math.pi) % 360.0,
        }

    def close(self) -> None:
        logger.debug("MockGPS closed.")


class RealGPS(BaseGPS):
    """NEO-M8N using USB serial NMEA or the Garud HAT SPI-UART bridge."""

    def __init__(self) -> None:
        self._transport = config.GPS_TRANSPORT.upper()
        self._serial = None
        self._gps = None
        self._pynmea2 = None
        self._last_sentence_monotonic = 0.0
        self._gga_fix_quality = 0
        self._rmc_active = False
        self._last = {
            "timestamp_ns": 0,
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0,
            "fix_ok": False,
            "fix_type": "NO FIX",
            "satellites": None,
            "hdop": None,
            "ground_speed_mps": None,
            "course_deg": None,
        }

        if self._transport in {"USB_SERIAL", "SERIAL", "UART"}:
            import pynmea2
            import serial

            self._pynmea2 = pynmea2
            self._serial = serial.Serial(
                config.GPS_PORT,
                config.GPS_BAUDRATE,
                timeout=config.GPS_SERIAL_TIMEOUT_SEC,
            )
            logger.info(
                "GPS M8N initialized on %s @ %d baud (timeout %.1fs).",
                config.GPS_PORT,
                config.GPS_BAUDRATE,
                config.GPS_SERIAL_TIMEOUT_SEC,
            )
        elif self._transport == "SC16IS750_SPI":
            import bus_manager
            from sensors.gps_m8n import GPSM8N

            self._gps = GPSM8N(bus_manager.get_spi())
            logger.info(
                "GPS M8N initialized through SC16IS750 on SPI0 CE1/GPIO%d @ %d.",
                config.GPS_SC16IS750_CS_PIN,
                config.GPS_BAUDRATE,
            )
        else:
            raise ValueError(f"Unsupported GPS_TRANSPORT: {config.GPS_TRANSPORT}")

    def read(self) -> dict:
        if self._serial is not None:
            return self._read_serial_nmea()

        fix = self._gps.read_fix(timeout_s=1.0)
        if not fix:
            return {**self._last, "fix_ok": False}

        if fix.get("lat") is not None:
            self._last["latitude"] = fix["lat"]
        if fix.get("lon") is not None:
            self._last["longitude"] = fix["lon"]
        if fix.get("altitude_m") is not None:
            self._last["altitude"] = fix["altitude_m"]
        self._last["fix_ok"] = bool(fix.get("fixed"))
        fix_code = fix.get("fix")
        self._last["fix_type"] = "3D" if self._last["fix_ok"] and fix_code not in (0, 1, None) else "2D" if self._last["fix_ok"] else "NO FIX"
        self._last["satellites"] = fix.get("satellites")
        self._last["hdop"] = fix.get("hdop")
        self._last["ground_speed_mps"] = fix.get("ground_speed_mps")
        self._last["course_deg"] = fix.get("course_deg")
        self._last["timestamp_ns"] = time.monotonic_ns()
        return dict(self._last)

    def _read_serial_nmea(self) -> dict:
        """Read one NMEA sentence and merge GGA/RMC fields into the last fix."""
        raw = self._serial.readline()
        if not raw:
            return self._serial_snapshot()

        line = raw.decode("ascii", errors="ignore").strip()
        if not line.startswith(("$GNGGA", "$GPGGA", "$GNRMC", "$GPRMC")):
            return self._serial_snapshot()

        try:
            message = self._pynmea2.parse(line)
        except (self._pynmea2.ParseError, ValueError, TypeError) as exc:
            logger.debug("Ignoring malformed NMEA sentence: %s", exc)
            return self._serial_snapshot()

        now_mono = time.monotonic()
        self._last_sentence_monotonic = now_mono
        self._last["timestamp_ns"] = time.monotonic_ns()

        if line.startswith(("$GNGGA", "$GPGGA")):
            self._gga_fix_quality = int(message.gps_qual or 0)
            self._last["satellites"] = int(message.num_sats or 0)
            self._last["hdop"] = self._optional_float(message.horizontal_dil)
            if self._gga_fix_quality > 0:
                self._last["latitude"] = float(message.latitude)
                self._last["longitude"] = float(message.longitude)
                altitude = self._optional_float(message.altitude)
                if altitude is not None:
                    self._last["altitude"] = altitude

        elif line.startswith(("$GNRMC", "$GPRMC")):
            self._rmc_active = message.status == "A"
            if self._rmc_active:
                self._last["latitude"] = float(message.latitude)
                self._last["longitude"] = float(message.longitude)
                speed_knots = self._optional_float(message.spd_over_grnd) or 0.0
                self._last["ground_speed_mps"] = speed_knots * 0.514444
                self._last["course_deg"] = self._optional_float(message.true_course)

        return self._serial_snapshot()

    def _serial_snapshot(self) -> dict:
        sentence_fresh = (
            self._last_sentence_monotonic > 0.0
            and time.monotonic() - self._last_sentence_monotonic
            <= config.GPS_FIX_STALE_TIMEOUT_SEC
        )
        fix_ok = sentence_fresh and (self._gga_fix_quality > 0 or self._rmc_active)
        self._last["fix_ok"] = fix_ok
        self._last["fix_type"] = self._nmea_fix_type() if fix_ok else "NO FIX"
        return dict(self._last)

    def _nmea_fix_type(self) -> str:
        labels = {
            1: "GPS",
            2: "DGPS",
            4: "RTK FIX",
            5: "RTK FLOAT",
            6: "ESTIMATED",
        }
        if self._gga_fix_quality > 0:
            return labels.get(self._gga_fix_quality, f"FIX {self._gga_fix_quality}")
        return "ACTIVE" if self._rmc_active else "NO FIX"

    @staticmethod
    def _optional_float(value) -> float | None:
        if value in (None, ""):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def close(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        if self._gps is not None and hasattr(self._gps, "close"):
            self._gps.close()


def create_gps() -> BaseGPS:
    """Factory: return mock or real GPS based on config."""
    if config.USE_MOCK_HARDWARE:
        return MockGPS()
    return RealGPS()


def gps_worker(shared: SharedData, stop_event: threading.Event) -> None:
    """
    Background thread: poll GPS and update shared data.

    Args:
        shared: Shared data store.
        stop_event: Shutdown signal.
    """
    try:
        gps = create_gps()
    except Exception as exc:
        logger.error("GPS init error: %s", exc)
        shared.update(gps_ok=False, status="GPS_INIT_ERROR")
        shared.record_worker_error("GPS", exc, expected_hz=config.GPS_EXPECTED_HZ)
        return
    logger.info("GPS worker started (mock=%s).", config.USE_MOCK_HARDWARE)

    try:
        while not stop_event.is_set():
            try:
                if config.USE_MOCK_HARDWARE and shared.is_fault_active("freeze_gps"):
                    shared.record_event("SENSOR_STALE", "GPS", "WARN", "Mock GPS freeze injected.")
                    stop_event.wait(0.5)
                    continue
                reading = gps.read()
                if config.USE_MOCK_HARDWARE and shared.is_fault_active("gps_loss"):
                    reading["fix_ok"] = False
                    reading["fix_type"] = "NO FIX"
                if config.USE_MOCK_HARDWARE and shared.is_fault_active("gps_high_hdop"):
                    reading["hdop"] = config.GPS_HDOP_DEGRADED + 2.0
                shared.update(
                    latitude=reading["latitude"],
                    longitude=reading["longitude"],
                    gps_altitude=reading["altitude"],
                    gps_ground_speed_mps=float(reading.get("ground_speed_mps") or 0.0),
                    gps_course_deg=float(reading.get("course_deg") or 0.0),
                    gps_satellites=int(reading.get("satellites") or 0),
                    gps_hdop=float(reading.get("hdop") or 0.0),
                    gps_fix_type=str(reading.get("fix_type", "NO FIX")),
                    gps_timestamp_ns=int(reading.get("timestamp_ns") or time.monotonic_ns()),
                    gps_ok=reading["fix_ok"],
                )
                fix_ok = bool(reading.get("fix_ok"))
                hdop = reading.get("hdop")
                degraded = hdop is not None and hdop > config.GPS_HDOP_DEGRADED
                status_reason = "GPS fix fresh." if fix_ok and not degraded else "No valid GPS fix." if not fix_ok else f"HDOP {hdop:.2f} above warning threshold."
                shared.record_worker_success(
                    "GPS",
                    expected_hz=config.GPS_EXPECTED_HZ,
                    reason=status_reason,
                    status="DEGRADED" if degraded or not fix_ok else "HEALTHY",
                    details={
                        "fix_valid": fix_ok,
                        "fix_type": reading.get("fix_type", "UNAVAILABLE"),
                        "satellites": reading.get("satellites"),
                        "hdop": hdop,
                        "ground_speed_mps": reading.get("ground_speed_mps"),
                        "course_deg": reading.get("course_deg"),
                    },
                )
                if not fix_ok:
                    shared.record_event("GPS_FIX_LOST", "GPS", "WARN", "GPS has no valid fix.")
            except Exception as exc:
                logger.error("GPS read error: %s", exc)
                shared.update(gps_ok=False, status="GPS_ERROR")
                shared.record_worker_error("GPS", exc, expected_hz=config.GPS_EXPECTED_HZ)

            poll_interval = 0.5 if config.USE_MOCK_HARDWARE else 0.05
            stop_event.wait(poll_interval)
    finally:
        gps.close()
        logger.info("GPS worker stopped.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    sd = SharedData()
    evt = threading.Event()
    t = threading.Thread(target=gps_worker, args=(sd, evt), daemon=True)
    t.start()
    for _ in range(5):
        time.sleep(1)
        print(sd.get_snapshot())
    evt.set()
    t.join()
