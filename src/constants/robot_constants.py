from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpimath import units
from wpimath.system.plant import DCMotor
from constants.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians


class MotorIDs:
    foc_active = False

    # INTAKE
    motor_id_arm = 51
    motor_id_head = 52
    motor_id_roller_top = 53
    motor_id_roller_bottom = 54

    # SPINDEX
    motor_id_motor_spindex = 55


class IntakeConstants:
    ROLLER_MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    ROLLER_GEARING = 4.0  # TODO: measure actual gear ratio
    ROLLER_MOI = 0.01  # TODO: calculate from roller mass/geometry (kg*m^2)
    ROLLER_CIRCUMFERENCE = (
        1.374 * 3.14159
    )  # TODO: measure actual roller diameter (inches)

    ARM_HOME_ROTATIONS = 0  # TODO: calibrate
    ARM_DEPLOYED_ROTATIONS = 10  # TODO: gear-ratio * target-angle
    ARM_STOWED_ROTATIONS = 5  # TODO: calibrate
    HEAD_HOME_ROTATIONS = 0  # TODO: calibrate
    HEAD_DEPLOYED_ROTATIONS = 20  # TODO: calibrate
    HEAD_STOWED_ROTATIONS = 7  # TODO: calibrate

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class SpindexConstants:
    MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    GEARING = 4.0  # TODO: measure actual gear ratio
    MOI = 0.005  # TODO: calculate from mechanism mass/geometry (kg*m^2)

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class DriveConstants:
    TOTAL_WIDTH_INCHES = 27.0  # IN
    TOTAL_WIDTH_INCHES_BUMPERS = 34.5  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL = 21.452  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL_METERS = 0.0254 * CENTER_WHEEL_TO_CENTER_WHEEL  # M
    ROBOT_MASS_KG = 50  # KG
    ROBOT_MOI = (
        ROBOT_MASS_KG
        * (
            CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
            + CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
        )
        / 12
    )  # kg·m^2 assuming sqaure robot
    MAX_TRANSLATIONAL_VELOCITY = 1.0 * TunerConstants.speed_at_12_volts  # m/s
    MAX_ACCL = 3  # m/s^2
    MAX_ANGULAR_VELOCITY = rotationsToRadians(
        0.75
    )  # 3/4 of a rotation per second max angular velocity


class VisionConstants:
    """Vision subsystem constants"""

    # Camera configuration
    CAMERA_LIGHT_ID = 0  # NEED TO FIX/CHANGE
    BACK_LEFT_SWERVE_NAME = "Back_Left_Swerve"
    FRONT_LEFT_SWERVE_NAME = "Front_Left_Swerve"
    BACK_RIGHT_SWERVE_NAME = "Back_Right_Swerve"
    FRONT_RIGHT_SWERVE_NAME = "Front_Right_Swerve"

    K_SINGLE_TAG_STD_DEVS = [4.0, 4.0, 8.0]
    K_MULTI_TAG_STD_DEVS = [0.5, 0.5, 1.0]

    # Vision strategies

    BACK_LEFT_SWERVE_TO_ROBOT = Transform3d(  # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 12.625),
            units.inchesToMeters(
                (-1.0 * (DriveConstants.TOTAL_WIDTH_INCHES / 2) + 2.5)
            ),
            units.inchesToMeters(12),
        ),
        Rotation3d(0, units.degreesToRadians(0), 0),
    )

    BACK_RIGHT_SWERVE_TO_ROBOT = Transform3d(  # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625),
        ),
        Rotation3d(0, units.degreesToRadians(0), 0),
    )
    FRONT_LEFT_SWERVE_TO_ROBOT = Transform3d(  # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625),
        ),
        Rotation3d(0, units.degreesToRadians(0), 0),
    )
    FRONT_RIGHT_SWERVE_TO_ROBOT = Transform3d(  # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625),
        ),
        Rotation3d(0, units.degreesToRadians(0), 0),
    )

    # Look at, call Vision.filter_april_tag_field()

    NULL_APRIL_TAG_ID = -1
    INVALID_ANGLE = -361.0
    NO_TARGET_DISTANCE = -1.0

    AUTO_ALIGN_VELOCITY_CONSTANT = 3.0
    AUTO_ALIGN_ACCELERATION_CONSTANT = 3.0

    AUTO_ALIGN_ANGULAR_VELOCITY_CONSTANT = units.degreesToRadians(
        180.0
    )  # wpk is this a good amount?

    # AprilTag lists for different game pieces and alliances NEED TO CHNAGE
    BLUE_APRIL_TAG_LIST_REEF = [17, 18, 19, 20, 21, 22]
    RED_APRIL_TAG_LIST_REEF = [6, 7, 8, 9, 10, 11]

    BLUE_APRIL_TAG_LIST_CORAL_STATION = [12, 13]
    RED_APRIL_TAG_LIST_CORAL_STATION = [1, 2]

    NOTE_ALIGN_PIXEL_TOLERANCE = 250.0  # NEED TO CHANGE
