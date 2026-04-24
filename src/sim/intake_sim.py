from constants import IntakeConstants
from sim.flywheel_motor_sim import FlywheelMotorSim, MotorPositionSim
from subsystems.intake import Intake


class IntakeSim:
    def __init__(self, intake: Intake):
        self._roller_top = FlywheelMotorSim(
            intake.motor_roller,
            IntakeConstants.ROLLER_MOTOR,
            IntakeConstants.ROLLER_GEARING,
            IntakeConstants.ROLLER_MOI,
        )
        self._arm = MotorPositionSim(
            intake.motor_arm_leader,
            IntakeConstants.ARM_MOTOR,
            IntakeConstants.ARM_GEARING,
            IntakeConstants.ARM_MOI,
            viscous_damping=IntakeConstants.ARM_VISCOUS_DAMPING,
            static_friction=IntakeConstants.ARM_STATIC_FRICTION,
            load_voltage=intake.kG_arm,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._roller_top.update(tm_diff)
        self._arm.update(tm_diff)
