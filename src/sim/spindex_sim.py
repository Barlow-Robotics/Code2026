from constants import SpindexConstants
from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems.spindex import Spindex


class SpindexSim:
    def __init__(self, spindex: Spindex):
        self._motor = FlywheelMotorSim(
            spindex.motor_spindex,
            SpindexConstants.MOTOR,
            SpindexConstants.GEARING,
            SpindexConstants.MOI,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._motor.update(tm_diff)
