from typing import TYPE_CHECKING
from commands2 import Command
from subsystems.drivetrain import Drivetrain

if TYPE_CHECKING:
    from subsystems import Spindex, Turret, Feeder, Shooter, Intake


class ReverseCommand(Command):
    def __init__(
        self,
        driveSub: "Drivetrain",
        shooter: "Shooter",
        feeder: "Feeder",
        spindex: "Spindex",
        turret: "Turret",
        intake: "Intake",
    ):
        super().__init__()
        self.driveSub = driveSub
        self.shooter = shooter
        self.feeder = feeder
        self.spindex = spindex
        self.turret = turret
        self.intake = intake
        self._feeding = False
        self.addRequirements(
            self.shooter, self.feeder, self.spindex, self.turret, self.intake
        )

    def initialize(self):
        self._feeding = False
        # self.target_velocity = SmartDashboard.getNumber(
        #     "CustomFloatVelocity", ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT
        # )

    def execute(self):
        # self.shooter.set_velocity(-self.target_velocity)
        self.feeder.move(-9 * 3)
        self.spindex.move(-9)
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
