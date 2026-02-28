from constants import FeederConstants
from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems.feeder import Feeder


class FeederSim:
    def __init__(self, feeder: Feeder):
        self._feeder_motor_const = FlywheelMotorSim(
            feeder.motor_feeder_constant,
            FeederConstants.MOTOR,
            FeederConstants.GEARING,
            FeederConstants.MOI,
        )
        self._feeder_motor_alt = FlywheelMotorSim(
            feeder.motor_feeder_alternating,
            FeederConstants.MOTOR,
            FeederConstants.GEARING,
            FeederConstants.MOI,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._feeder_motor_const.update(tm_diff)
        self._feeder_motor_alt.update(tm_diff)
