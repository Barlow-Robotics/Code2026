from constants import ShooterConstants
from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems import Shooter


class ShooterSim:
    def __init__(self, shooter: Shooter):
        self._hood_motor = FlywheelMotorSim(
            shooter.hood_motor,
            ShooterConstants.HOOD_MOTOR,
            ShooterConstants.HOOD_GEARING,
            ShooterConstants.HOOD_MOI,
        )
        self._turret_motor = FlywheelMotorSim(
            shooter.turret_motor,
            ShooterConstants.TURRET_MOTOR,
            ShooterConstants.TURRET_GEARING,
            ShooterConstants.TURRET_MOI,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._hood_motor.update(tm_diff)
        self._turret_motor.update(tm_diff)
