from wpimath.system.plant import DCMotor

from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems.spindex import Spindex


class SpindexSim:
    def __init__(self, spindex: Spindex):
        self._motor = FlywheelMotorSim(
            spindex.motor_spindex, DCMotor.krakenX60(1), gearing=4.0, moi=0.005
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._motor.update(tm_diff)
