from typing import TYPE_CHECKING
from commands2 import Command
from wpilib import SmartDashboard
from constants import ShooterConstants
from constants.field_constants import CustomPoints
from subsystems.drivetrain import Drivetrain

if TYPE_CHECKING:
    from subsystems import Spindex, Turret, Feeder, Shooter
from wpimath.geometry import Pose2d, Rotation2d, Translation3d
from pykit.logger import Logger as PyKitLogger


class ShootCommand(Command):
    def __init__(
        self,
        driveSub: "Drivetrain",
        shooter: "Shooter",
        feeder: "Feeder",
        spindex: "Spindex",
        turret: "Turret",
    ):
        super().__init__()
        self.driveSub = driveSub
        self.shooter = shooter
        self.feeder = feeder
        self.spindex = spindex
        self.turret = turret
        self._feeding = False
        self.addRequirements(self.shooter, self.feeder, self.spindex)

    def initialize(self):
        self._feeding = False
        self.static_actual_velocity_to_go = self.turret.actual_velocity_to_go
        PyKitLogger.recordOutput("ShootCommand/velocity_to_go", self.static_actual_velocity_to_go)  
        self.shooter.set_velocity(self.static_actual_velocity_to_go)

    def execute(self):
        if self.static_actual_velocity_to_go != self.turret.actual_velocity_to_go:
            self.static_actual_velocity_to_go = self.turret.actual_velocity_to_go
            PyKitLogger.recordOutput("ShootCommand/velocity_to_go", self.static_actual_velocity_to_go)  
            self.shooter.set_velocity(self.static_actual_velocity_to_go)

        if self._feeding:
            return
        current_velocity = self.shooter.get_current_velocity()
        if (
            abs(current_velocity - self.static_actual_velocity_to_go)
            <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE
        ):
            self.feeder.move(9)
            self.spindex.move(9*3)
            self._feeding = True

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.shooter.stop_flywheel()
        self.feeder.stop()
        self.spindex.stop()
        # self.turret.reset_target_hood_and_turret()
