"""
Central pin, bus, sensor, and runtime configuration for the GARUDA payload.

Pin mapping follows the working Garud HAT setup:

    BNO085 (IMU)         - I2C1 : SDA=GPIO2, SCL=GPIO3
    PCA9685 (Servo drv)  - I2C1 : SDA=GPIO2, SCL=GPIO3, OE=GPIO4
    BMP388 (Baro)        - SPI0 : MISO=GPIO9, MOSI=GPIO10, SCK=GPIO11, CS=GPIO8
    NEO-M8N GPS         - USB UART by default: /dev/ttyUSB0 at 9600 baud
    ULN2003 (Stepper)    - GPIO : IN1=GPIO25, IN2=GPIO24, IN3=GPIO23, IN4=GPIO18
    Buzzer               - GPIO16

Only edit values here; runtime modules import this file so the project stays
in sync with the tested sensor setup.
"""

from pathlib import Path

try:
    import board
except (ImportError, NotImplementedError):
    board = None


def _board_pin(name: str, bcm: int):
    """Return a Blinka board pin on hardware, or the BCM number off-target."""
    if board is None:
        return bcm
    return getattr(board, name)

# ---------------------------------------------------------------------------
# Module enable flags
# ---------------------------------------------------------------------------
ENABLE_CAMERA = False
ENABLE_GPS = False
ENABLE_IMU = True
ENABLE_BAROMETER = True
ENABLE_GIMBAL = True
ENABLE_TELEMETRY = False
ENABLE_MAPPING = True
ENABLE_LOGGING = True
ENABLE_NAVIGATION_ESTIMATOR = True
ENABLE_STEERING = False  # Future: parachute/glider steering
ENABLE_GLIDER_SERVOS = True
ENABLE_POST_FLIGHT_PROCESSING = True
PAUSE_STATE_TRANSITIONS = True
AUTO_ARM_IN_MOCK_MODE = True

# ---------------------------------------------------------------------------
# AHRS attitude estimation
# ---------------------------------------------------------------------------
ENABLE_AHRS = True
AHRS_MODE = "BNO085"  # OFF, BNO085, MADGWICK, MAHONY, AUTO
AHRS_RATE_HZ = 100
AHRS_USE_MAGNETOMETER = True
AHRS_ACCEL_REJECTION_ENABLED = True
AHRS_MAG_REJECTION_ENABLED = True
AHRS_MAX_SAMPLE_AGE_MS = 250.0
AHRS_MIN_DT_SEC = 0.001
AHRS_MAX_DT_SEC = 0.25
AHRS_FAIL_COUNT_THRESHOLD = 5
AHRS_RECOVERY_COUNT_THRESHOLD = 20
AHRS_MADGWICK_BETA = 0.08
AHRS_MAHONY_KP = 0.6
AHRS_MAHONY_KI = 0.02
AHRS_MAHONY_BIAS_LIMIT_RADS = 0.2
AHRS_ACCEL_NORM_MIN_G = 0.70
AHRS_ACCEL_NORM_MAX_G = 1.30
AHRS_MAG_NORM_MIN_UT = 15.0
AHRS_MAG_NORM_MAX_UT = 85.0
AHRS_MAX_ANGULAR_JUMP_DEG = 120.0
AHRS_BNO085_DEGRADED_ACCURACY_RAD = 0.6
AHRS_AUTO_SOFTWARE_FALLBACK = "MADGWICK"
AHRS_INIT_TIMEOUT_SEC = 2.0
IMU_TO_BODY_QUATERNION = (1.0, 0.0, 0.0, 0.0)

# ---------------------------------------------------------------------------
# Runtime hardware mode
# ---------------------------------------------------------------------------
USE_MOCK_HARDWARE = False

# ---------------------------------------------------------------------------
# Timing intervals (seconds)
# ---------------------------------------------------------------------------
IMAGE_CAPTURE_INTERVAL_SEC = 2.0
SENSOR_LOG_INTERVAL_SEC = 1.0
TELEMETRY_INTERVAL_SEC = 1.0

# ---------------------------------------------------------------------------
# Engineering verification / dashboard thresholds
# ---------------------------------------------------------------------------
GROUND_STATION_UPDATE_HZ = 2.0
DIAGNOSTIC_EVENT_DEBOUNCE_SEC = 2.0
WORKER_STALE_TIMEOUT_SEC = 3.0
SENSOR_STALE_TIMEOUT_SEC = 2.0
CAMERA_STALE_MULTIPLIER = 2.5
GPS_EXPECTED_HZ = 2.0
IMU_EXPECTED_HZ = AHRS_RATE_HZ
BAROMETER_EXPECTED_HZ = 2.0
CAMERA_EXPECTED_HZ = 1.0 / IMAGE_CAPTURE_INTERVAL_SEC
GIMBAL_EXPECTED_HZ = 5.0
TELEMETRY_EXPECTED_HZ = 1.0 / TELEMETRY_INTERVAL_SEC
LOGGER_EXPECTED_HZ = 1.0 / SENSOR_LOG_INTERVAL_SEC
HEALTH_MONITOR_EXPECTED_HZ = 0.2
GPS_HDOP_DEGRADED = 3.0
GPS_FIX_STALE_TIMEOUT_SEC = 2.0
NAVIGATION_RATE_HZ = 20.0
NAVIGATION_EXPECTED_HZ = NAVIGATION_RATE_HZ

# Navigation estimator thresholds. Defaults are deliberately conservative and
# must be tuned through bench testing, field walking, vehicle tests, and flight
# validation before being treated as final.
NAV_MIN_SATELLITES = 5
NAV_MAX_HDOP = 4.0
NAV_MAX_GPS_AGE_MS = 1500.0
NAV_MAX_BARO_AGE_MS = 1000.0
NAV_MAX_AHRS_AGE_MS = 500.0
NAV_MAX_PLAUSIBLE_SPEED_MPS = 120.0
NAV_MAX_ABSOLUTE_GPS_JUMP_M = 250.0
NAV_GPS_REJECT_COUNT_TO_LOST = 4
NAV_GPS_GOOD_COUNT_TO_RECOVER = 3
NAV_DEAD_RECKON_MAX_SEC = 5.0
NAV_MIN_SPEED_FOR_GPS_HEADING_MPS = 2.0
NAV_MIN_DT_SEC = 0.001
NAV_MAX_DT_SEC = 0.25
NAV_PROCESS_NOISE_POSITION = 1.0
NAV_PROCESS_NOISE_VELOCITY = 4.0
NAV_GPS_POSITION_NOISE_M2 = 16.0
NAV_GPS_VELOCITY_NOISE_M2PS2 = 4.0
NAV_GPS_RECOVERY_ENABLED = True
NAV_GPS_RECOVERY_MAX_CORRECTION_RATE_MPS = 8.0
NAV_GPS_RECOVERY_MIN_STEP_M = 0.5
NAV_GPS_RECOVERY_POSITION_TOLERANCE_M = 8.0
NAV_SAFE_IN_DEGRADED = False
NAV_SAFE_IN_SHORT_DEAD_RECKONING = False
IMAGE_SYNC_WARN_MS = 250.0
BARO_VELOCITY_FILTER_ALPHA = 0.35
GIMBAL_SATURATION_MARGIN_DEG = 2.0
CPU_TEMP_WARN_C = 75.0
DISK_FREE_WARN_BYTES = 500 * 1024 * 1024
POWER_EXPECTED_HZ = 1.0
POWER_UNDERVOLTAGE_WARN_V = 4.8
POWER_OVERCURRENT_WARN_A = 2.0
INA219_I2C_ADDRESS = None  # Set after confirming wiring; 0x40 is already PCA9685.
IMAGE_QUALITY_ENABLE_LIVE = True
IMAGE_QUALITY_EXPECTED_HZ = 0.2
IMAGE_QUALITY_BLUR_WARN_VARIANCE = 80.0
IMAGE_QUALITY_BRIGHTNESS_LOW = 45.0
IMAGE_QUALITY_BRIGHTNESS_HIGH = 210.0
IMAGE_QUALITY_CLIPPED_WARN_FRACTION = 0.12
STORAGE_VALIDATION_EXPECTED_HZ = 0.05
STORAGE_VALIDATION_INTERVAL_SEC = 20.0

# ---------------------------------------------------------------------------
# Serial / I2C / SPI hardware settings (used when USE_MOCK_HARDWARE = False)
# ---------------------------------------------------------------------------
# Tested GARUDA HAT connections:
#   BNO085 + PCA9685 + INA219: I2C1 on GPIO2/GPIO3
#   BMP388: SPI0 on GPIO9/GPIO10/GPIO11 with CS=GPIO8
#   GPS M8N: USB UART /dev/ttyUSB0 by default; SC16IS750 remains optional
#   XBee3: Pi primary UART on GPIO14/GPIO15
# NEO-M8N connected through a USB-to-UART adapter on Raspberry Pi Zero 2 W.
# Set GPS_TRANSPORT back to "SC16IS750_SPI" for the Garud HAT bridge.
GPS_TRANSPORT = "USB_SERIAL"
GPS_PORT = "/dev/ttyUSB0"
GPS_SERIAL_TIMEOUT_SEC = 0.4
XBEE_SERIAL_PORT = "/dev/ttyAMA0"
XBEE_PORT = XBEE_SERIAL_PORT
GPS_BAUDRATE = 9600
XBEE_BAUDRATE = 9600
XBEE_TX_INTERVAL = TELEMETRY_INTERVAL_SEC

BNO085_TRANSPORT = "I2C"
BNO085_I2C_ADDRESS = 0x4A
IMU_ADDRESS = BNO085_I2C_ADDRESS
BAROMETER_ADDRESS = None

# GARUDA HAT schema pin map (Raspberry Pi BCM numbering)
I2C_SDA_PIN = 2
I2C_SCL_PIN = 3
I2C_SDA = _board_pin("SDA", I2C_SDA_PIN)
I2C_SCL = _board_pin("SCL", I2C_SCL_PIN)

SPI_MOSI_PIN = 10
SPI_MISO_PIN = 9
SPI_SCLK_PIN = 11
SPI_MOSI = _board_pin("MOSI", SPI_MOSI_PIN)
SPI_MISO = _board_pin("MISO", SPI_MISO_PIN)
SPI_SCK = _board_pin("SCK", SPI_SCLK_PIN)

SERVO_OE_PIN = 4
PCA9685_OE_PIN = _board_pin("D4", SERVO_OE_PIN)
PCA9685_I2C_ADDRESS = 0x40
PCA9685_PWM_FREQ = 50
SERVO_CONTROLLER_ADDRESS = PCA9685_I2C_ADDRESS
GIMBAL_PAN_CHANNEL = 3
GIMBAL_TILT_CHANNEL = 4
GIMBAL_SERVO_CHANNEL = 4

# Glider specific servo channels
GLIDER_LEFT_CHANNEL = 0
GLIDER_RIGHT_CHANNEL = 1
GLIDER_DROGUE_CHANNEL = 2
GLIDER_DROGUE_SAFE_ANGLE = 60.0
GLIDER_DROGUE_DEPLOY_ANGLE = 120.0

BMP388_CS_PIN = 8
BMP388_CS = _board_pin("D8", BMP388_CS_PIN)
BMP388_INT_PIN = 17

GPS_SC16IS750_CS_PIN = 7
GPS_SC16IS750_CS = _board_pin("D7", GPS_SC16IS750_CS_PIN)
GPS_SC16IS750_I2C_ADDRESS = 0x4D
GPS_SC16IS750_CRYSTAL_HZ = 14_745_600
GPS_NMEA_TIMEOUT_S = 60
GPS_PPS_PIN = 17

# Retained for SPI bench experiments only. Payload runtime defaults to BNO085
# on I2C1 (GPIO2/GPIO3) at BNO085_I2C_ADDRESS.
BNO085_CS_PIN = None
BNO085_RST_PIN = None
BNO085_INT_PIN = None
BNO085_CS = None
BNO085_RST = None
BNO085_INT = None
BNO085_ROTATION_MODE = "ROTATION_VECTOR"  # or "GAME_ROTATION_VECTOR" if supported cleanly

ULN2003_IN1_PIN = 25
ULN2003_IN2_PIN = 24
ULN2003_IN3_PIN = 23
ULN2003_IN4_PIN = 18
ULN2003_IN1 = _board_pin("D25", ULN2003_IN1_PIN)
ULN2003_IN2 = _board_pin("D24", ULN2003_IN2_PIN)
ULN2003_IN3 = _board_pin("D23", ULN2003_IN3_PIN)
ULN2003_IN4 = _board_pin("D18", ULN2003_IN4_PIN)
STEPPER_STEP_DELAY = 0.002

LORA_TX_PIN = 14
LORA_RX_PIN = 15
LORA_AUX_PIN = 25
LORA_M0_PIN = 23
LORA_M1_PIN = 24

BUZZER_PIN = 16
STATUS_LED_PIN = 26

# ---------------------------------------------------------------------------
# Storage paths (relative to project root)
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent
IMAGE_SAVE_PATH = PROJECT_ROOT / "data" / "images"
LOG_SAVE_PATH = PROJECT_ROOT / "data" / "logs"
MAP_SAVE_PATH = PROJECT_ROOT / "data" / "maps"

# ---------------------------------------------------------------------------
# Mission simulation (pre-hardware)
# ---------------------------------------------------------------------------
SIMULATION_DURATION_SEC = 30.0
SIMULATION_DESCENT_START_SEC = 5.0
MOCK_GLIDER_DESCENT_ONLY = False

# Pune reference coordinates for mock GPS
MOCK_GPS_LAT = 18.5204
MOCK_GPS_LON = 73.8567
MOCK_START_ALTITUDE_M = 700.0

# ---------------------------------------------------------------------------
# Flight state-machine thresholds
# ---------------------------------------------------------------------------
# Mission profile: ascent to roughly 1 km AGL, payload ejection at apogee,
# glider/parachute activation at 600 m AGL, then guidance actuation begins.
TARGET_APOGEE_AGL_M = 1000.0
GLIDER_DEPLOY_ALTITUDE_AGL_M = 600.0
STATE_CONFIRMATION_COUNT = 5
LAUNCH_DETECT_ACCEL_G = 1.5
LAUNCH_DETECT_ALTITUDE_AGL_M = 30.0
BOOST_BURNOUT_ACCEL_G = 1.5
BOOST_MAX_DURATION_SEC = 10.0
APOGEE_DESCENT_VELOCITY_MPS = -1.0
APOGEE_ALTITUDE_DROP_M = 2.0
APOGEE_MIN_ALTITUDE_AGL_M = 50.0
APOGEE_BACKUP_TIME_SEC = 30.0
GLIDER_DEPLOY_CONFIRMATION_COUNT = 5
GLIDER_DEPLOY_SETTLE_SEC = 1.0
LANDING_DETECT_ALTITUDE_AGL_M = 20.0
LANDING_DETECT_VELOCITY_MPS = 1.0
LANDING_DETECT_TIME_SEC = 5.0
MAX_FLIGHT_TIME_SEC = 600.0

# Gimbal logical command limits (degrees).
# Stepper is wire-wrap limited to one full revolution total: -180..+180.
# Servo is represented as a logical -180..+180 command and converted to the
# physical PCA9685 angle range below.
GIMBAL_PITCH_MIN = -180.0
GIMBAL_PITCH_MAX = 180.0
GIMBAL_ROLL_MIN = -180.0
GIMBAL_ROLL_MAX = 180.0
GIMBAL_SERVO_MIN = -180.0
GIMBAL_SERVO_MAX = 180.0
GIMBAL_SERVO_CENTER = 0.0
GIMBAL_SERVO_PHYSICAL_MIN_DEG = 0.0
GIMBAL_SERVO_PHYSICAL_MAX_DEG = 360.0
GIMBAL_SERVO_PHYSICAL_CENTER_DEG = 180.0
GIMBAL_SERVO_ACTUATION_RANGE_DEG = 360.0

# The gimbal now damps large swings and keeps the camera roughly nadir-facing.
# It is not expected to remove all roll/yaw from images.
GIMBAL_POSE_DAMPING_GAIN = 0.35
GIMBAL_MAX_COMMAND_STEP_DEG = 8.0

# ---------------------------------------------------------------------------
# Mapping / camera footprint model
# ---------------------------------------------------------------------------
# Approximate camera field of view. Tune these values for the actual lens.
CAMERA_HORIZONTAL_FOV_DEG = 62.2
CAMERA_VERTICAL_FOV_DEG = 48.8
CAMERA_SENSOR_WIDTH_PX = 4056
CAMERA_SENSOR_HEIGHT_PX = 3040
CAMERA_FOCAL_LENGTH_PX = 3100.0
CAMERA_CENTER_X_PX = CAMERA_SENSOR_WIDTH_PX / 2.0
CAMERA_CENTER_Y_PX = CAMERA_SENSOR_HEIGHT_PX / 2.0
CAMERA_DISTORTION_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]
CAMERA_BACKEND = "AUTO"  # AUTO, PICAMERA2, or OPENCV
CAMERA_DEVICE_INDEX = 0
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

# Altitudes lower than this are ignored for footprint sizing to avoid zero-area
# polygons when the payload has landed.
MAPPING_MIN_FOOTPRINT_ALTITUDE_M = 2.0

# Cell size for approximate unique coverage estimation.
MAPPING_COVERAGE_GRID_M = 5.0

# Post-flight reconstruction profiles and resource limits. These settings are
# only used by processing/*, vision/*, mapping/*, and storage/* modules.
MAPPING_DEFAULT_PROFILE = "BALANCED"
MAPPING_MAX_WORKERS = 2
MAPPING_MAX_NEIGHBORS_PER_IMAGE = 8
MAPPING_TEMPORAL_NEIGHBORS = 2
MAPPING_MAX_ALTITUDE_RATIO = 2.5
MAPPING_MAX_GPS_DISTANCE_M = 250.0
MAPPING_MAX_TIME_SEPARATION_SEC = 20.0
MAPPING_MAX_YAW_DIFF_DEG = 80.0
MAPPING_MAX_ROLL_PITCH_DIFF_DEG = 45.0
MAPPING_MIN_PREDICTED_OVERLAP = 0.03
MAPPING_SPATIAL_GRID_M = 120.0
MAPPING_FEATURE_BACKEND = "SIFT"
MAPPING_FEATURE_MAX_DIM = 2048
MAPPING_PREVIEW_MAX_DIM = 640
MAPPING_FEATURE_MAX_FEATURES = 4000
MAPPING_MATCH_RATIO = 0.75
MAPPING_MIN_FILTERED_MATCHES = 12
MAPPING_MIN_GEOMETRIC_INLIERS = 10
MAPPING_MIN_INLIER_RATIO = 0.18
MAPPING_ENABLE_DENSE_RECONSTRUCTION = False
MAPPING_ENABLE_ORTHOMOSAIC = False
MAPPING_REFINE_FOCAL_LENGTH = False
MAPPING_REFINE_PRINCIPAL_POINT = False
MAPPING_REFINE_DISTORTION = False
MAPPING_INTERPOLATE_SENSOR_TIMELINE = False
MAPPING_CACHE_VERSION = "v1"
POSTFLIGHT_SAVE_PATH = PROJECT_ROOT / "data" / "postflight"
SENSOR_CALIBRATION_PATH = LOG_SAVE_PATH / "sensor_calibration.json"

# Post-flight image quality thresholds.
QUALITY_BLUR_MIN_VARIANCE = 80.0
QUALITY_EXPOSURE_LOW = 35.0
QUALITY_EXPOSURE_HIGH = 220.0
QUALITY_MAX_TILT_DEG = 35.0
QUALITY_MAX_ANGULAR_RATE_DPS = 120.0
QUALITY_CLIPPED_SHADOW_FRACTION = 0.15
QUALITY_CLIPPED_HIGHLIGHT_FRACTION = 0.15
QUALITY_GOOD_MIN_SCORE = 0.70
QUALITY_MARGINAL_MIN_SCORE = 0.40

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
LOG_LEVEL = "INFO"

# ---------------------------------------------------------------------------
# Sensor calibration
# ---------------------------------------------------------------------------
APPLY_SENSOR_CALIBRATION = True
CALIBRATION_SAMPLE_RATE_HZ = 20.0
CALIBRATION_DEFAULT_SECONDS = 20.0
CALIBRATION_STATIONARY_GYRO_MAX_DPS = 3.0
CALIBRATION_ACCEL_NORM_MIN = 8.0
CALIBRATION_ACCEL_NORM_MAX = 11.5
BAROMETER_PRESSURE_MIN_HPA = 300.0
BAROMETER_PRESSURE_MAX_HPA = 1200.0
BAROMETER_SEA_LEVEL_MIN_HPA = 800.0
BAROMETER_SEA_LEVEL_MAX_HPA = 1100.0

# ---------------------------------------------------------------------------
# Gimbal geometry and control
# ---------------------------------------------------------------------------
# Mechanical setup: stepper corrects the opposite X-axis deflection, while the
# PCA9685 servo on channel 0 corrects the opposite Y-axis deflection.
GIMBAL_STEPPER_AXIS = "OPP_X"
GIMBAL_SERVO_AXIS = "OPP_Y"
GIMBAL_STEPPER_SIGN = -1
GIMBAL_SERVO_SIGN = -1
GIMBAL_STEPPER_HOME_DEG = 0.0
GIMBAL_STEPPER_MIN_DEG = -180.0
GIMBAL_STEPPER_MAX_DEG = 180.0
GIMBAL_STEPS_PER_DEG = 11.3777777778  # 4096 half-steps / 360 deg
GIMBAL_MAX_STEPS_PER_TICK = 24
GIMBAL_DEFLECTION_DEADBAND_DEG = 1.0
GIMBAL_LOOP_HZ = 5.0
GIMBAL_SERVO_RATE_LIMIT_DPS = 45.0
GIMBAL_STEPPER_RATE_LIMIT_DPS = 60.0
