from constants import ShooterConstants
from sim.flywheel_motor_sim import FlywheelMotorSimSparkFlex
from subsystems import Shooter


class ShooterSim:
    def __init__(self, shooter: Shooter):
        self._flywheel_leader = FlywheelMotorSimSparkFlex(
            shooter.flywheel_motor_left_leader,
            ShooterConstants.FLYWHEEL_MOTOR,
            ShooterConstants.FLYWHEEL_GEARING,
            ShooterConstants.FLYWHEEL_MOI,
        )
        # self._flywheel_follower = FlywheelMotorSimSparkFlex(
        #     shooter.flywheel_motor_right_follower,
        #     ShooterConstants.FLYWHEEL_MOTOR,
        #     ShooterConstants.FLYWHEEL_GEARING,
        #     ShooterConstants.FLYWHEEL_MOI,
        # )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._flywheel_leader.update(tm_diff)
        # self._flywheel_follower.update(tm_diff)
