from robotpy_apriltag import AprilTagField
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpimath import units
from wpimath.system.plant import DCMotor
from constants.tuner_constants import TunerConstants
from constants import SI
from pathplannerlib.config import PIDConstants
import math


class RobotFeatures:
    HAS_DRIVETRAIN = True
    HAS_VISION = True
    HAS_SHOOTER = True
    HAS_TURRET = True
    HAS_INTAKE = True
    HAS_SPINDEX = True
    HAS_FEEDER = True
    HAS_CPROFILE = False
    LOW_LOGGING = False

    @classmethod
    def configure(cls):
        from wpilib import RobotBase

        if RobotBase.isReal():
            cls.HAS_DRIVETRAIN = True
            cls.HAS_VISION = True
            cls.vision_camera_count = 3
            cls.HAS_SHOOTER = True
            cls.HAS_TURRET = True
            cls.HAS_INTAKE = True
            cls.HAS_SPINDEX = True
            cls.HAS_FEEDER = True
            cls.HAS_CPROFILE = False
            cls.TESTING = False
            cls.HAS_TURRET_ANGLE = False
            cls.LOGGING = False
            cls.LOGGING_ROBOT = False
            cls.LOGGING_DRIVETRAIN = False
            cls.LOGGING_VISION = False
            cls.LOGGING_SHOOTER = False
            cls.LOGGING_TURRET = False
            cls.LOGGING_INTAKE = False
            cls.LOGGING_SPINDEX = False
            cls.LOGGING_FEEDER = False
            cls.LOW_LOGGING = True
        else:
            cls.HAS_DRIVETRAIN = True
            cls.vision_camera_count = 4
            cls.HAS_VISION = True
            cls.HAS_SHOOTER = True
            cls.HAS_TURRET = True
            cls.HAS_INTAKE = True
            cls.HAS_SPINDEX = True
            cls.HAS_FEEDER = True
            cls.HAS_CPROFILE = False
            cls.TESTING = True
            cls.HAS_TURRET_ANGLE = False
            cls.LOGGING = True
            cls.LOGGING_ROBOT = True
            cls.LOGGING_DRIVETRAIN = True
            cls.LOGGING_VISION = False
            cls.LOGGING_SHOOTER = False
            cls.LOGGING_TURRET = False
            cls.LOGGING_INTAKE = False
            cls.LOGGING_SPINDEX = False
            cls.LOGGING_FEEDER = False
            cls.LOW_LOGGING = True



class MotorIDs:
    foc_active = True

    # INTAKE
    motor_id_arm_leader_left = 51
    motor_id_arm_follower_right = 52
    motor_id_roller = 53

    # SPINDEX
    motor_id_motor_spindex = 55

    # FEEDER
    motor_id_motor_feeder_constant = 56
    motor_id_motor_feeder_alternating = 57

    # SHOOTER
    motor_id_hood = 58
    motor_id_turret = 59
    motor_id_flywheel_left = 60
    motor_id_flywheel_right = 61
    cancoder_id_hood = 54


class IntakeConstants:
    ROLLER_MOTOR = DCMotor.krakenX44(1)
    ROLLER_GEARING = 30 / 58
    ROLLER_MOI = 0.01  # TODO: calculate from roller mass/geometry (kg*m^2)
    # Diamater is 2in
    ROLLER_CIRCUMFERENCE_IN = 2 * math.pi  # BW: REAL CIRCUMFRENCE IS 2IN * PI
    ROLLER_CIRCUMFERENCE_M = (
        2 * math.pi * SI.inches_to_meters
    )  # BW: REAL CIRCUMFRENCE IS 2IN * PI

    ARM_MOTOR = DCMotor.krakenX44(1)
    ARM_GEARING = 58 / 14
    ARM_MOI = 0.01
    ARM_HOME_ROTATIONS = 0
    ARM_DEPLOYED_LENGTH_IN = 4
    ARM_DEPLOYED_ROTATIONS = 4

    INTAKE_VELOCITY_CONSTANT = 2.6 * 6
    # Amplified from robot speed

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class ShooterConstants:
    FLYWHEEL_MOTOR = DCMotor.neoVortex(1)
    FLYWHEEL_GEARING = 1.0
    FLYWHEEL_MOI = 0.002651
    FLYWHEEL_VELOCITY_TOLERANCE = 1

    FLYWHEEL_RPM_CONSTANT = 5700
    FLYWHEEL_VELOCITY_CONSTANT = 17.5
    FLYWHEEL_RADIUS_INCHES = 2


class TurretConstants:
    TURRET_MOTOR = DCMotor.minion(1)
    TURRET_GEARING = 139 / 12.0
    TURRET_MOI = 0.4  # TODO: calculate from roller mass/geometry (kg*m^2)

    HOOD_MOTOR = DCMotor.minion(1)
    HOOD_GEARING = 1.5
    HOOD_MOI = 0.1  # TODO: calculate from roller mass/geometry (kg*m^2)
    HOOD_MECHANICAL_RATIO = 11.6

    SHOOTER_HEIGHT_FOR_FUEL_IN = 20.57
    SHOOTER_HEIGHT_FOR_FUEL_M = 20.57 * SI.inches_to_meters
    SHOOTER_SET_VELOCITY_CONSTANT = 10.5


class SpindexConstants:
    MOTOR = DCMotor.krakenX44(1)
    GEARING = 42 / 24
    MOI = 0.005  # TODO: calculate from mechanism mass/geometry (kg*m^2)

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class FeederConstants:
    MOTOR = DCMotor.minion(1)
    GEARING = 1.0
    MOI = 0.005  # TODO: calculate from mechanism mass/geometry (kg*m^2)

    PID = {"kP": 0.11, "kI": 0, "kD": 0, "kF": 0, "kA": 0}  # TODO: tune PID


class DriveConstants:
    TOTAL_HEIGHT_INCHES_FROM_FLOOR = 3.8125
    TOTAL_HEIGHT_METERS_FROM_FLOOR = (
        SI.inches_to_meters * TOTAL_HEIGHT_INCHES_FROM_FLOOR
    )
    TOTAL_WIDTH_INCHES = 27.0 + 3 / 16  # IN
    TOTAL_WIDTH_INCHES_BUMPERS = 34.5 + 0.5  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL = 21.75  # IN
    CENTER_WHEEL_TO_CENTER_WHEEL_METERS = (
        SI.inches_to_meters * CENTER_WHEEL_TO_CENTER_WHEEL
    )  # M
    ROBOT_MASS_KG = 52.1631  # KG
    ROBOT_MOI = (
        ROBOT_MASS_KG
        * (
            CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
            + CENTER_WHEEL_TO_CENTER_WHEEL_METERS**2
        )
        / 12
    )  # kg·m^2 assuming sqaure robot
    MAX_TRANSLATIONAL_VELOCITY = TunerConstants.speed_at_12_volts
    MAX_ACCL = TunerConstants.accl_at_12_volts  # m/s^2
    MAX_ANGULAR_VELOCITY = TunerConstants.max_angular_velocity_at_12_volts
    MAX_ANGULAR_ACCL = TunerConstants.max_angular_accl_at_12_volts


class VisionConstants:
    FIELD_LAYOUT = AprilTagField.k2026RebuiltAndyMark

    # Camera configuration
    CAMERA_1_NAME = "Left_Cam_Swerve"
    CAMERA_2_NAME = "Right_Cam_Swerve"
    CAMERA_3_NAME = "Back_Right_Swerve"
    CAMERA_4_NAME = "Back_Left_Swerve"

    VISION_SIM = False

    HALF_WIDTH = units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2)
    HALF_LENGTH = units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2)
    HEIGHT = units.inchesToMeters(12.625)

    FRONT_LEFT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            HALF_LENGTH - ((1 + 3 / 8) * SI.inches_to_meters),
            HALF_WIDTH - (4.25 * SI.inches_to_meters),  # LEFT
            DriveConstants.TOTAL_HEIGHT_METERS_FROM_FLOOR
            + (SI.inches_to_meters * (16 + 1 / 8)),
        ),
        Rotation3d(0, 0, units.degreesToRadians(0)),
    )

    FRONT_RIGHT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            HALF_LENGTH - ((1 + 3 / 8) * SI.inches_to_meters),
            -HALF_WIDTH + ((4 + 13 / 16) * SI.inches_to_meters),  # RIGHT
            DriveConstants.TOTAL_HEIGHT_METERS_FROM_FLOOR
            + (SI.inches_to_meters * (16 + 1 / 8)),
        ),
        Rotation3d(0, 0, units.degreesToRadians(0)),
    )
    # 12 5/8 tall

    BACK_RIGHT_SWERVE_TO_ROBOT = Transform3d(
        Translation3d(
            -HALF_LENGTH + (0.5 * SI.inches_to_meters),
            -HALF_WIDTH + (2.625 * SI.inches_to_meters),  # RIGHT
            DriveConstants.TOTAL_HEIGHT_METERS_FROM_FLOOR
            + (12.625 * SI.inches_to_meters),
        ),
        Rotation3d(0, 0, units.degreesToRadians(180)),
    )

    # BACK_RIGHT_SWERVE_TO_ROBOT = Transform3d(
    #     Translation3d(
    #         -HALF_LENGTH,
    #         -HALF_WIDTH,
    #         HEIGHT,
    #     ),
    #     Rotation3d(0, 0, units.degreesToRadians((180 - 45))),
    # )

    AUTO_ALIGN_VELOCITY_CONSTANT = 3.0
    AUTO_ALIGN_ACCELERATION_CONSTANT = 3.0

    BLUE_APRIL_TAG_LIST = range(17, 33)
    RED_APRIL_TAG_LIST = range(1, 17)
    BLUE_APRIL_TAG_TRENCH_LIST = [17, 22]
    RED_APRIL_TAG_TRENCH_LIST = [1, 6]
    # BW: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf


class AutoConstants:
    auto_translation_pid = PIDConstants(4.0, 0.0, 0)  # TB tuned.
    auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)
    period = 0.04  # seconds per scheduler run
