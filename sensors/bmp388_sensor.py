"""BMP388 wrapper for the tested GARUDA SPI0 sensor setup."""

from __future__ import annotations

import math

import config


class BMP388Sensor:
    """Read validated pressure, temperature, and altitude over SPI."""

    def __init__(
        self,
        spi_bus,
        cs_pin=config.BMP388_CS,
        sea_level_pressure_hpa: float = 1013.25,
    ) -> None:
        import digitalio
        from adafruit_bmp3xx import BMP3XX_SPI

        self._cs = digitalio.DigitalInOut(cs_pin)
        self.bmp = BMP3XX_SPI(spi_bus, self._cs)
        self._sea_level_pressure_hpa = self._validated_sea_level_pressure(
            sea_level_pressure_hpa
        )
        self.bmp.sea_level_pressure = self._sea_level_pressure_hpa
        self.bmp.pressure_oversampling = 8
        self.bmp.temperature_oversampling = 2
        self.bmp.filter_coefficient = 2

    def set_sea_level_pressure(self, hpa: float) -> None:
        self._sea_level_pressure_hpa = self._validated_sea_level_pressure(hpa)
        self.bmp.sea_level_pressure = self._sea_level_pressure_hpa

    @staticmethod
    def _validated_sea_level_pressure(hpa: float) -> float:
        pressure = float(hpa)
        if not math.isfinite(pressure):
            raise ValueError(f"Invalid sea-level pressure: {hpa!r}")
        if not config.BAROMETER_SEA_LEVEL_MIN_HPA <= pressure <= config.BAROMETER_SEA_LEVEL_MAX_HPA:
            raise ValueError(f"Sea-level pressure outside safe range: {pressure:.2f} hPa")
        return pressure

    @staticmethod
    def _validated_pressure(hpa: float) -> float:
        pressure = float(hpa)
        if not math.isfinite(pressure):
            raise ValueError(f"Invalid BMP388 pressure: {hpa!r}")
        if not config.BAROMETER_PRESSURE_MIN_HPA <= pressure <= config.BAROMETER_PRESSURE_MAX_HPA:
            raise ValueError(f"BMP388 pressure outside safe range: {pressure:.2f} hPa")
        return pressure

    def _altitude_from_pressure(self, pressure_hpa: float) -> float:
        ratio = pressure_hpa / self._sea_level_pressure_hpa
        if ratio <= 0.0:
            raise ValueError(f"Invalid BMP388 pressure ratio: {ratio!r}")
        altitude = 44330.0 * (1.0 - ratio ** (1.0 / 5.255))
        if not math.isfinite(altitude):
            raise ValueError(f"Invalid BMP388 altitude: {altitude!r}")
        return altitude

    def read(self) -> dict:
        pressure_hpa = self._validated_pressure(self.bmp.pressure)
        temperature_c = float(self.bmp.temperature)
        return {
            "temperature_c": temperature_c,
            "pressure_hpa": pressure_hpa,
            "altitude_m": self._altitude_from_pressure(pressure_hpa),
        }
