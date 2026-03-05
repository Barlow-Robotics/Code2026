class RobotFeatures:
    HAS_DRIVETRAIN = True
    HAS_VISION = True
    HAS_SHOOTER = True
    HAS_TURRET = True
    HAS_INTAKE = True
    HAS_SPINDEX = True
    HAS_FEEDER = True

    @classmethod
    def configure(cls):
        from wpilib import RobotBase

        if RobotBase.isReal():
            cls.HAS_DRIVETRAIN = True
            cls.HAS_VISION = True
            cls.vision_camera_count = 2
            cls.HAS_SHOOTER = False
            cls.HAS_TURRET = False
            cls.HAS_INTAKE = False
            cls.HAS_SPINDEX = False
            cls.HAS_FEEDER = False
        else:
            cls.HAS_DRIVETRAIN = True
            cls.vision_camera_count = 4
            cls.HAS_VISION = True
            cls.HAS_SHOOTER = True
            cls.HAS_TURRET = True
            cls.HAS_INTAKE = True
            cls.HAS_SPINDEX = True
            cls.HAS_FEEDER = True


from robotpy_apriltag import AprilTagField
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpimath import units
from wpimath.system.plant import DCMotor
from constants.tuner_constants import TunerConstants
from wpimath.units import rotationsToRadians
from constants import SI
from pathplannerlib.config import PIDConstants


class MotorIDs:
    foc_active = False

    # INTAKE
    motor_id_arm = 51
    motor_id_head = 52
    motor_id_roller_top = 53
    motor_id_roller_bottom = 54

    # SPINDEX
    motor_id_motor_spindex = 55

    # FEEDER
    motor_id_motor_feeder_constant = 56
    motor_id_motor_feeder_alternating = 57

    # SHOOTER
    motor_id_hood = 58
    motor_id_turret = 59
    motor_id_flywheel_left = 60  # from hood position to left flywheel
    motor_id_flywheel_right = 61


class IntakeConstants:
    ROLLER_MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    ROLLER_GEARING = 4.0  # TODO: measure actual gear ratio
    ROLLER_MOI = 0.01  # TODO: calculate from roller mass/geometry (kg*m^2)
    ROLLER_CIRCUMFERENCE = (
        1.374 * 3.14159
    )  # TODO: measure actual roller diameter (inches)
    ARM_MOTOR = DCMotor.krakenX60(1)
    ARM_GEARING = 1
    ARM_MOI = 0.01
    HEAD_MOTOR = DCMotor.krakenX60(1)
    HEAD_GEARING = 1
    HEAD_MOI = 0.01
    ARM_LENGTH = 10.5 * SI.inches_to_meters
    ARM_MIN_ANGLE = 0.0 * SI.degrees_to_radians  # TODO: calibrate
    ARM_MAX_ANGLE = 180.0 * SI.degrees_to_radians  # TODO: calibrate
    ARM_STARTING_ANGLE = 90.0 * SI.degrees_to_radians  # TODO: calibrate

    ARM_HOME_ROTATIONS = 90 * SI.degrees_to_rotations  # TODO: calibrate
    ARM_DEPLOYED_ROTATIONS = (
        180 * SI.degrees_to_rotations
    )  # TODO: gear-ratio * target-angle
    ARM_STOWED_ROTATIONS = 135 * SI.degrees_to_rotations  # TODO: calibrate
    HEAD_HOME_ROTATIONS = 90 * SI.degrees_to_rotations  # TODO: calibrate
    HEAD_DEPLOYED_ROTATIONS = 180 * SI.degrees_to_rotations  # TODO: calibrate
    HEAD_STOWED_ROTATIONS = 135 * SI.degrees_to_rotations  # TODO: calibrate

    INTAKE_VELOCITY_CONSTANT = 1  # ft/sec

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class ShooterConstants:
    FLYWHEEL_MOTOR = DCMotor.neoVortex(1)  # TODO: verify motor type
    FLYWHEEL_GEARING = 1.0  # TODO: measure actual gear ratio
    FLYWHEEL_MOI = 0.002651  # TODO: calculate from roller mass/geometry (kg*m^2)

    FLYWHEEL_RPM_CONSTANT = 5700
    FLYWHEEL_VELOCITY_CONSTANT = 9.5
    FLYWHEEL_RADIUS_INCHES = 2


class TurretConstants:
    TURRET_MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    TURRET_GEARING = 10.0  # TODO: measure actual gear ratio
    TURRET_MOI = 0.4  # TODO: calculate from roller mass/geometry (kg*m^2)

    HOOD_MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    HOOD_GEARING = 1.0  # TODO: measure actual gear ratio
    HOOD_MOI = 0.1  # TODO: calculate from roller mass/geometry (kg*m^2)

    SHOOTER_HEIGHT_FOR_FUEL_IN = 20.57
    SHOOTER_HEIGHT_FOR_FUEL_M = 20.57 * SI.inches_to_meters
    SHOOTER_SET_VELOCITY_CONSTANT = 9.5


class SpindexConstants:
    MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    GEARING = 4.0  # TODO: measure actual gear ratio
    MOI = 0.005  # TODO: calculate from mechanism mass/geometry (kg*m^2)

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class FeederConstants:
    MOTOR = DCMotor.krakenX60(1)  # TODO: verify motor type
    GEARING = 4.0  # TODO: measure actual gear ratio
    MOI = 0.005  # TODO: calculate from mechanism mass/geometry (kg*m^2)

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class DriveConstants:
    TOTAL_HEIGHT_INCHES_FROM_FLOOR = 3.8125
    TOTAL_HEIGHT_METERS_FROM_FLOOR = SI.inches_to_meters * TOTAL_HEIGHT_INCHES_FROM_FLOOR
    TOTAL_WIDTH_INCHES = 27.0  # IN
    TOTAL_WIDTH_INCHES_BUMPERS = 34.5  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL = 21.75  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL_METERS = (
        SI.inches_to_meters * CENTER_WHEEL_TO_CENTER_WHEEL
    )  # M
    ROBOT_MASS_KG = 50  # KG
    ROBOT_MOI = (
        ROBOT_MASS_KG
        * (
            CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
            + CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
        )
        / 12
    )  # kg·m^2 assuming sqaure robot
    MAX_TRANSLATIONAL_VELOCITY = 0.2 * TunerConstants.speed_at_12_volts  # m/s
    MAX_ACCL = 3  # m/s^2
    MAX_ANGULAR_VELOCITY = rotationsToRadians(
        0.75
    )  # 3/4 of a rotation per second max angular velocity


class VisionConstants:
    FIELD_LAYOUT = AprilTagField.k2026RebuiltAndyMark

    # Camera configuration
    CAMERA_1_NAME = "Left_Cam_Swerve"
    CAMERA_2_NAME = "Right_Cam_Swerve"
    CAMERA_3_NAME = "Back_Left_Swerve"
    CAMERA_4_NAME = "Back_Right_Swerve"

    VISION_SIM = True

    HALF_WIDTH = units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2)
    HALF_LENGTH = units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2)
    HEIGHT = units.inchesToMeters(12.625)
    
    FRONT_LEFT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            -2.5 * SI.inches_to_meters,
            HALF_WIDTH + 0.5 * SI.inches_to_meters, # LEFT
            DriveConstants.TOTAL_HEIGHT_METERS_FROM_FLOOR + SI.inches_to_meters * 22.0625,
        ),
        Rotation3d(0, 0, units.degreesToRadians(0)),
    )

    FRONT_RIGHT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            -2.5 * SI.inches_to_meters,
            -HALF_WIDTH - 0.5 * SI.inches_to_meters, # RIGHT
            DriveConstants.TOTAL_HEIGHT_METERS_FROM_FLOOR + SI.inches_to_meters * 22.0625,
        ),
        Rotation3d(0, 0, units.degreesToRadians(0)),
    )

    BACK_LEFT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            -HALF_LENGTH,
            HALF_WIDTH,
            HEIGHT,
        ),
        Rotation3d(0, 0, units.degreesToRadians(-(180 - 45))),
    )

    BACK_RIGHT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            -HALF_LENGTH,
            -HALF_WIDTH,
            HEIGHT,
        ),
        Rotation3d(0, 0, units.degreesToRadians((180 - 45))),
    )
    AUTO_ALIGN_VELOCITY_CONSTANT = 3.0
    AUTO_ALIGN_ACCELERATION_CONSTANT = 3.0

    BLUE_APRIL_TAG_LIST = range(17, 33)
    RED_APRIL_TAG_LIST = range(1, 17)


class AutoConstants:
    auto_translation_pid = PIDConstants(4, 0.0, 0)
    auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)
    period = 0.02  # seconds per scheduler run
