from constants import TurretConstants
from sim.flywheel_motor_sim import FlywheelMotorSim
from subsystems import Turret


class TurretSim:
    def __init__(self, turret: Turret):
        self._hood_motor = FlywheelMotorSim(
            turret.hood_motor,
            TurretConstants.HOOD_MOTOR,
            TurretConstants.HOOD_GEARING,
            TurretConstants.HOOD_MOI,
        )
        self._turret_motor = FlywheelMotorSim(
            turret.turret_motor,
            TurretConstants.TURRET_MOTOR,
            TurretConstants.TURRET_GEARING,
            TurretConstants.TURRET_MOI,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._hood_motor.update(tm_diff)
        self._turret_motor.update(tm_diff)
