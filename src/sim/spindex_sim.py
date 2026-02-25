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
        self._feeder_motor_alt = FlywheelMotorSim(
            spindex.motor_feeder_alternating,
            SpindexConstants.MOTOR,
            SpindexConstants.GEARING,
            SpindexConstants.MOI,
        )
        self._feeder_motor_const = FlywheelMotorSim(
            spindex.motor_feeder_constant,
            SpindexConstants.MOTOR,
            SpindexConstants.GEARING,
            SpindexConstants.MOI,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._motor.update(tm_diff)
        self._feeder_motor_alt.update(tm_diff)
        self._feeder_motor_const.update(tm_diff)
