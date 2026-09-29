"""Shared I2C1 and SPI0 buses for the GARUDA sensor stack."""

from __future__ import annotations

import config

_i2c_bus = None
_spi_bus = None


def get_i2c():
    """Return shared I2C1 (GPIO2 SDA, GPIO3 SCL)."""
    global _i2c_bus
    if _i2c_bus is None:
        import busio

        _i2c_bus = busio.I2C(config.I2C_SCL, config.I2C_SDA)
    return _i2c_bus


def get_spi():
    """Return shared SPI0 (GPIO11 SCLK, GPIO10 MOSI, GPIO9 MISO)."""
    global _spi_bus
    if _spi_bus is None:
        import busio

        _spi_bus = busio.SPI(
            config.SPI_SCK,
            MOSI=config.SPI_MOSI,
            MISO=config.SPI_MISO,
        )
    return _spi_bus
