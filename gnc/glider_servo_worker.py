import time
import logging
import config

logger = logging.getLogger(__name__)

class MockGliderServos:
    def __init__(self):
        logger.info("MockGliderServos initialized.")
        self.left_angle = 90.0
        self.right_angle = 90.0
        self.drogue_angle = config.GLIDER_DROGUE_SAFE_ANGLE

    def set_angles(self, left: float, right: float, drogue: float = None):
        changed = abs(left - self.left_angle) > 1.0 or abs(right - self.right_angle) > 1.0
        drogue_changed = drogue is not None and abs(drogue - self.drogue_angle) > 1.0
        if changed or drogue_changed:
            logger.info(
                "MockGliderServos -> left=%.1f right=%.1f drogue=%s",
                left,
                right,
                "INHIBITED" if drogue is None else f"{drogue:.1f}",
            )
            self.left_angle = left
            self.right_angle = right
            if drogue is not None:
                self.drogue_angle = drogue

    def close(self):
        logger.info("MockGliderServos closed.")

class RealGliderServos:
    def __init__(self):
        import digitalio
        from adafruit_servokit import ServoKit

        self._oe = digitalio.DigitalInOut(config.PCA9685_OE_PIN)
        self._oe.direction = digitalio.Direction.OUTPUT
        self._oe.value = False

        self._kit = ServoKit(
            channels=16,
            address=config.SERVO_CONTROLLER_ADDRESS,
        )
        
        # Configure left servo (Channel 0)
        self._kit.servo[config.GLIDER_LEFT_CHANNEL].set_pulse_width_range(500, 2500)
        self._kit.servo[config.GLIDER_LEFT_CHANNEL].actuation_range = 180
        
        # Configure right servo (Channel 1)
        self._kit.servo[config.GLIDER_RIGHT_CHANNEL].set_pulse_width_range(500, 2500)
        self._kit.servo[config.GLIDER_RIGHT_CHANNEL].actuation_range = 180

        # Configure drogue servo (Channel 2)
        self._kit.servo[config.GLIDER_DROGUE_CHANNEL].set_pulse_width_range(500, 2500)
        self._kit.servo[config.GLIDER_DROGUE_CHANNEL].actuation_range = 180

        logger.info(
            f"RealGliderServos initialized: left={config.GLIDER_LEFT_CHANNEL}, "
            f"right={config.GLIDER_RIGHT_CHANNEL}, drogue={config.GLIDER_DROGUE_CHANNEL} "
            f"at PCA9685 0x{config.SERVO_CONTROLLER_ADDRESS:02X}."
        )
        # Establish a known safe output before accepting runtime commands.
        self.set_angles(90.0, 90.0, config.GLIDER_DROGUE_SAFE_ANGLE)

    def set_angles(self, left: float, right: float, drogue: float = None):
        # Constrain to 0-180
        left = max(0.0, min(180.0, left))
        right = max(0.0, min(180.0, right))
        
        self._kit.servo[config.GLIDER_LEFT_CHANNEL].angle = left
        self._kit.servo[config.GLIDER_RIGHT_CHANNEL].angle = right
        if drogue is not None:
            self._kit.servo[config.GLIDER_DROGUE_CHANNEL].angle = max(0.0, min(180.0, drogue))

    def close(self):
        # Neutralize flight controls before releasing PWM.  Do not move the
        # deployment channel during shutdown because it may already be fired.
        self._kit.servo[config.GLIDER_LEFT_CHANNEL].angle = 90.0
        self._kit.servo[config.GLIDER_RIGHT_CHANNEL].angle = 90.0
        time.sleep(0.1)
        self._kit.servo[config.GLIDER_LEFT_CHANNEL].angle = None
        self._kit.servo[config.GLIDER_RIGHT_CHANNEL].angle = None
        self._kit.servo[config.GLIDER_DROGUE_CHANNEL].angle = None
        # Only set OE to True if we are the only one controlling PCA9685,
        # but since gimbal might be sharing it, we'll leave it as is to avoid conflict.
        # self._oe.value = True


def glider_servo_worker(
    shared,
    stop_event,
    use_mock: bool | None = None,
    command_drogue: bool = True,
    require_actuation_enabled: bool = False,
) -> None:
    """Read commands from SharedData and drive mock or real glider servos.

    ``use_mock`` supports hardware-in-the-loop tests with mock sensors and real
    servos. ``command_drogue=False`` prevents bench tests from moving the
    deployment servo.
    """
    mock_mode = config.USE_MOCK_HARDWARE if use_mock is None else use_mock
    logger.info(
        "Glider servo worker started (mock=%s, command_drogue=%s).",
        mock_mode,
        command_drogue,
    )
    
    if mock_mode:
        hw = MockGliderServos()
    else:
        try:
            hw = RealGliderServos()
        except Exception as e:
            logger.error("Failed to initialize RealGliderServos: %s", e)
            return

    try:
        while not stop_event.is_set():
            snap = shared.get_snapshot()
            
            # Production is fail-closed: only the safety supervisor can allow
            # left/right control. Bench tests opt out explicitly so they can
            # exercise the complete command path while unloaded.
            controls_enabled = snap.actuation_enabled or not require_actuation_enabled
            left = snap.servo_left if controls_enabled else 90.0
            right = snap.servo_right if controls_enabled else 90.0
            drogue = None
            if command_drogue:
                drogue = (
                    snap.servo_drogue
                    if snap.glider_deployed
                    else config.GLIDER_DROGUE_SAFE_ANGLE
                )
            hw.set_angles(
                left=left,
                right=right,
                drogue=drogue,
            )
            shared.update(servo_ok=True)
            time.sleep(0.05)  # 20 Hz loop
            
    except Exception as e:
        shared.update(servo_ok=False, actuation_enabled=False)
        logger.error("Glider servo worker crashed: %s", e)
    finally:
        shared.update(servo_ok=False)
        hw.close()
        logger.info("Glider servo worker stopped.")
