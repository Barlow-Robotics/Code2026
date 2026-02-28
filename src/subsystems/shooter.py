from commands2 import Subsystem
from constants.robot_constants import MotorIDs
from constants import ShooterConstants
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


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()

        self.flywheel_motor_left_leader = SparkFlex(
            MotorIDs.motor_id_flywheel_left, type=SparkFlex.MotorType.kBrushless
        )
        self.flywheel_motor_right_follower = SparkFlex(
            MotorIDs.motor_id_flywheel_right, type=SparkFlex.MotorType.kBrushless
        )

        leader_config = SparkFlexConfig()
        leader_config.setIdleMode(
            leader_config.IdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        leader_config.smartCurrentLimit(80, freeLimit=5700)  # set to 5700 for max

        leader_config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            0.0001, 0.0, 0.0
        ).velocityFF(0.000175).outputRange(-1, 1)

        leader_config.closedLoop.maxMotion.maxVelocity(5700).maxAcceleration(
            10000
        ).allowedClosedLoopError(10)

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

        follower_config.follow(self.flywheel_motor_left_leader, False)

        self.flywheel_motor_right_follower.configure(
            follower_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.flywheel_target_velocity = 0.0

        self.start_flywheel_command = cmd.runOnce(
            lambda: self.setRPM(ShooterConstants.FLYWHEEL_RPM_CONSTANT)
        )

    def setRPM(self, targetRPM: float):
        self.flywheel_target_velocity = targetRPM
        self.flywheel_motor_left_leader.getClosedLoopController().setReference(
            targetRPM, SparkFlex.ControlType.kMAXMotionVelocityControl
        )

    def get_current_rpm(self) -> float:
        return self.flywheel_motor_left_leader.getEncoder().getVelocity()

    def stop_flywheel(self):
        self.flywheel_motor_left_leader.set(0)

    def periodic(self):
        PyKitLogger.recordOutput(
            "Shooter/flywheel_motor_left_velocity", float(self.get_current_rpm())
        )
        PyKitLogger.recordOutput(
            "Shooter/flywheel_motor_left_target_velocity",
            float(self.flywheel_target_velocity),
        )
