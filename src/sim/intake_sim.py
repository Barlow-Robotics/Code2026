from constants import IntakeConstants
from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems.intake import Intake


class IntakeSim:
    def __init__(self, intake: Intake):
        self._roller_top = FlywheelMotorSim(
            intake.motor_roller_top,
            IntakeConstants.ROLLER_MOTOR,
            IntakeConstants.ROLLER_GEARING,
            IntakeConstants.ROLLER_MOI,
        )
        self._roller_bottom = FlywheelMotorSim(
            intake.motor_roller_bottom,
            IntakeConstants.ROLLER_MOTOR,
            IntakeConstants.ROLLER_GEARING,
            IntakeConstants.ROLLER_MOI,
        )
        self._arm = FlywheelMotorSim(
            intake.motor_arm,
            IntakeConstants.ARM_MOTOR,
            IntakeConstants.ARM_GEARING,
            IntakeConstants.ARM_MOI,
        )

        self._head = FlywheelMotorSim(
            intake.motor_head,
            IntakeConstants.HEAD_MOTOR,
            IntakeConstants.ARM_GEARING,
            IntakeConstants.ARM_MOI,
        )

        # self._arm = ArmMotorSim(
        #     intake.motor_arm,
        #     IntakeConstants.ARM_MOTOR,
        #     IntakeConstants.ARM_GEARING,
        #     IntakeConstants.ARM_MOI,
        #     IntakeConstants.ARM_LENGTH,
        #     IntakeConstants.ARM_MIN_ANGLE,
        #     IntakeConstants.ARM_MAX_ANGLE,
        #     IntakeConstants.ARM_STARTING_ANGLE
        # )
        # self._head = ArmMotorSim(
        #     intake.motor_head,
        #     IntakeConstants.HEAD_MOTOR,
        #     IntakeConstants.HEAD_GEARING,
        #     IntakeConstants.HEAD_MOI,
        #     IntakeConstants.ARM_LENGTH,
        #     IntakeConstants.ARM_MIN_ANGLE,
        #     IntakeConstants.ARM_MAX_ANGLE,
        #     IntakeConstants.ARM_STARTING_ANGLE

        # )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._roller_top.update(tm_diff)
        self._roller_bottom.update(tm_diff)
        self._arm.update(tm_diff)
        self._head.update(tm_diff)
