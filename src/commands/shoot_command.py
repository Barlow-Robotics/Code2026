from typing import TYPE_CHECKING
from commands2 import Command
from constants import ShooterConstants

if TYPE_CHECKING:
    from subsystems.shooter import Shooter
    from subsystems.feeder import Feeder
    from subsystems import Spindex

VELOCITY_TOLERANCE = 9.4


class ShootCommand(Command):
    def __init__(self, shooter: "Shooter", feeder: "Feeder", spindex: "Spindex"):
        super().__init__()
        self.shooter = shooter
        self.feeder = feeder
        self.spindex = spindex
        self._feeding = False
        self.addRequirements(shooter, feeder, spindex)

    def initialize(self):
        self._feeding = False
        self.shooter.set_velocity(ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT)

    def execute(self):
        if self._feeding:
            return
        current_rpm = self.shooter.get_current_rpm()
        if (
            abs(current_rpm - ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT)
            <= VELOCITY_TOLERANCE
        ):
            self.feeder.move()
            self.spindex.move()
            self._feeding = True

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.shooter.stop_flywheel()
        self.feeder.stop()
        self.spindex.stop()
