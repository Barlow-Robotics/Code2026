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

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._roller_top.update(tm_diff)
        self._roller_bottom.update(tm_diff)
