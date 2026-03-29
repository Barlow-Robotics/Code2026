from typing import TYPE_CHECKING
from commands2 import Command
from subsystems.drivetrain import Drivetrain

if TYPE_CHECKING:
    from subsystems import Spindex, Turret, Feeder, Shooter, Intake


class ReverseCommandSlow(Command):
    def __init__(
        self,
        feeder: "Feeder",
        spindex: "Spindex",
    ):
        super().__init__()
        self.feeder = feeder
        self.spindex = spindex
        self._feeding = False
        self.addRequirements(
            self.feeder, self.spindex
        )

    def initialize(self):
        self._feeding = False
        # self.target_velocity = SmartDashboard.getNumber(
        #     "CustomFloatVelocity", ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT
        # )

    def execute(self):
        # self.shooter.set_velocity(-self.target_velocity)
        self.feeder.reverse(-9 * 1/3)
        self.spindex.move(-9 * 1/3)
        # current_speeds = self.driveSub.get_speeds()
        # overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
        # self.intake.reverse_velocity(overall_velocity)

    def isFinished(self):
        return False

    def end(self, interrupted):
        # self.shooter.stop_flywheel()
        self.feeder.stop()
        self.spindex.stop()
        # self.intake.stop()
