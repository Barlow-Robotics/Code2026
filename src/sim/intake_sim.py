from wpimath.system.plant import DCMotor

from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems.intake import Intake


class IntakeSim:
    def __init__(self, intake: Intake):
        motor_model = DCMotor.krakenX60(1)
        gearing = 4.0
        moi = 0.01  # kg*m^2

        self._roller_top = FlywheelMotorSim(intake.motor_roller_top, motor_model, gearing, moi)
        self._roller_bottom = FlywheelMotorSim(intake.motor_roller_bottom, motor_model, gearing, moi)

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._roller_top.update(tm_diff)
        self._roller_bottom.update(tm_diff)
