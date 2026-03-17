from typing import TYPE_CHECKING
from commands2 import Command
from constants import ShooterConstants
from constants.field_constants import CustomPoints
from subsystems.drivetrain import Drivetrain

if TYPE_CHECKING:
    from subsystems import Spindex, Turret, Feeder, Shooter
from wpimath.geometry import Pose2d, Rotation2d, Translation3d


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
        self.addRequirements(self.shooter, self.feeder, self.spindex, self.turret)

    def initialize(self):
        self._feeding = False
        self.shooter.set_velocity(ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT)
        if not self.driveSub.allow_center_auto_align:
            self.turret.set_target_hood_and_turret()

    def execute(self):
        if not self.driveSub.allow_center_auto_align:
            self.turret.set_target_hood_and_turret()
        else:
            pose = self.driveSub.get_pose()
            target_pose = pose.nearest(
                [
                    Pose2d(
                        CustomPoints.TARGET_POSE_SHOOT.toTranslation2d(), Rotation2d(0)
                    ),
                    Pose2d(
                        CustomPoints.TARGET_POSE_SHOOT_OTHER_SIDE.toTranslation2d(),
                        Rotation2d(0),
                    ),
                ]
            )
            target_pose = Translation3d(target_pose.X(), target_pose.Y(), 0)
            self.turret.set_target_hood_and_turret(shooting_location=target_pose)

        if self._feeding:
            return
        current_velocity = self.shooter.get_current_velocity()
        if (
            abs(current_velocity - ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT)
            <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE
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
