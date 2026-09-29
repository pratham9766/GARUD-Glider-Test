# GARUDA bench and flight procedure

This repository now has separate fail-safe bench and flight entry paths. Passing
software tests is necessary but does not certify an aircraft as flight-safe.
Complete every physical check below on the Raspberry Pi Zero 2 W before flight.

## Wiring used by the runtime

- BNO085: I2C1, GPIO2 SDA / GPIO3 SCL, address `0x4A`
- PCA9685: I2C1, address `0x40`, OE GPIO4; left/right/drogue channels 0/1/2
- BMP388: SPI0, GPIO11 SCLK / GPIO10 MOSI / GPIO9 MISO / GPIO8 CS
- NEO-M8N: USB serial `/dev/ttyUSB0`, 9600 baud

## Bench sequence on the Pi

1. Disconnect the deployment linkage and unload both brake lines.
2. Run the sensor-only hardware check:

   `python3 hardware_tests/test_sensor_stack.py`

3. Run the complete mock flight (no physical outputs):

   `python3 test_full_flight_mock.py`

4. Run live sensors with mock servos; this remains DISARMED:

   `python3 main.py --mode bench --duration 60`

5. With surfaces unloaded, check real PCA9685 initialization and neutral hold:

   `python3 main.py --mode bench --real-servos --duration 30`

6. For the existing full mock-sensor/real-control sweep, keep the deployment
   linkage disconnected and run:

   `python3 test_glider_only.py --real-servos --duration 30`

## Flight launch

Confirm the mission target in `config/gains.yaml`, correct servo directions and
limits, mechanical retention, battery under load, GPS outdoor fix, BNO mounting
orientation/calibration, BMP venting, and an independent recovery method.

The production command is intentionally explicit:

`python3 main.py --mode flight --arm-flight GARUDA`

Flight mode refuses mock sensors, waits for BNO085, BMP388 and a current GPS fix,
requires the ONNX RL model to load, captures pad barometric altitude as zero AGL,
and keeps left/right controls neutral until GUIDED_DESCENT and the safety gate
both agree. Stale or unhealthy GPS, IMU, or barometer data immediately inhibits
left/right actuation. Ctrl+C/SIGTERM also inhibits actuation and commands neutral.

## Required evidence before declaring flight-ready

- All three commands in steps 2-4 pass repeatedly on the actual Pi.
- Step 5 holds neutral and shutdown returns both control channels to neutral.
- Step 6 moves each surface in the correct direction with no binding or brownout.
- The deployment channel stays at its safe angle during every bench command.
- A restrained end-to-end rehearsal demonstrates correct state transitions,
  deployment direction, 20 Hz operation, sensor-loss inhibition, and clean logs.
