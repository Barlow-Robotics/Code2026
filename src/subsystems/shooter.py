import math
from commands2 import Subsystem
from wpilib import SmartDashboard
from constants.robot_constants import MotorIDs, RobotFeatures
from constants import SI, ShooterConstants
from rev import (
    FeedbackSensor,
    SparkBaseConfig,
    SparkFlex,
    SparkFlexConfig,
    ResetMode,
    PersistMode,
)
from pykit.logger import Logger as PyKitLogger
from commands2 import cmd
from utils.profiler import LoopTimer


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Shooter")

        SmartDashboard.putNumber(
            "CustomFloatVelocity", ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT
        )

        self.flywheel_motor_left_leader = SparkFlex(
            MotorIDs.motor_id_flywheel_right, type=SparkFlex.MotorType.kBrushless
        )
        self.flywheel_motor_right_follower = SparkFlex(
            MotorIDs.motor_id_flywheel_left, type=SparkFlex.MotorType.kBrushless
        )

        leader_config = SparkFlexConfig()
        leader_config.setIdleMode(
            leader_config.IdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        leader_config.smartCurrentLimit(80, freeLimit=5700)  # set to 5700 for max

        leader_config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            0.0002, 0.0, 0.00014
        ).velocityFF(0.0019).outputRange(-1, 1)

        leader_config.closedLoop.maxMotion.maxVelocity(5700).maxAcceleration(
            10000
        ).allowedClosedLoopError(10)
        leader_config.inverted(True)
        self.flywheel_motor_left_leader.configure(
            leader_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        follower_config = SparkFlexConfig()
        follower_config.setIdleMode(
            follower_config.IdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        follower_config.smartCurrentLimit(80, freeLimit=5700)  # set to 5700 for max

        follower_config.follow(self.flywheel_motor_left_leader, True)
        follower_config.inverted(True)

        self.flywheel_motor_right_follower.configure(
            follower_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.flywheel_target_RPM = 0.0
        self.flywheel_target_velocity = 0.0

        self.start_flywheel_command = cmd.runOnce(
            lambda: self.set_velocity(ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT)
        )

    def set_velocity(self, target_velocity: float = 9.5):
        self.flywheel_target_velocity = target_velocity
        radius = ShooterConstants.FLYWHEEL_RADIUS_INCHES * SI.inches_to_meters  # meters
        setRPM = (60 * target_velocity) / (2 * math.pi * radius)

        self.flywheel_target_RPM = setRPM
        self.flywheel_motor_left_leader.getClosedLoopController().setReference(
            setRPM, SparkFlex.ControlType.kMAXMotionVelocityControl
        )

    def get_current_rpm(self) -> float:
        return self.flywheel_motor_left_leader.getEncoder().getVelocity()

    def get_current_rpm_follower(self) -> float:
        return self.flywheel_motor_right_follower.getEncoder().getVelocity()

    def get_current_velocity(self) -> float:
        radius = ShooterConstants.FLYWHEEL_RADIUS_INCHES * SI.inches_to_meters  # meters
        current_rpm = self.get_current_rpm()
        return (2 * math.pi * radius * current_rpm) / 60

    def stop_flywheel(self):
        self.flywheel_motor_left_leader.set(0)
        self.flywheel_target_RPM = 0.0
        self.flywheel_target_velocity = 0.0

    def periodic(self):
        self._loop_timer.start()

        if RobotFeatures.LOGGING_SHOOTER:
            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_left_RPM", float(self.get_current_rpm())
            )
            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_left_target_RPM",
                float(self.flywheel_target_RPM),
            )

            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_right_RPM",
                float(self.get_current_rpm_follower()),
            )

            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_left_current_velocity",
                float(self.get_current_velocity()),
            )
            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_left_target_velocity",
                float(self.flywheel_target_velocity),
            )

            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_current_limit",
                float(self.flywheel_motor_left_leader.getOutputCurrent()),
            )
            PyKitLogger.recordOutput(
                "Shooter/flywheel_motor_current_volts",
                float(self.flywheel_motor_left_leader.getAppliedOutput()),
            )

        self._loop_timer.stop()
