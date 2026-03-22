from typing import TYPE_CHECKING
from commands2 import Command
from constants.field_constants import CustomPoints
from subsystems.drivetrain import Drivetrain

if TYPE_CHECKING:
    from subsystems import Turret
from wpimath.geometry import Pose2d, Rotation2d, Translation3d


class AutoAimCommand(Command):
    def __init__(self, driveSub: "Drivetrain", turret: "Turret"):
        super().__init__()
        self.driveSub = driveSub
        self.turret = turret
        self.addRequirements(self.turret)

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

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.turret.reset_target_hood_and_turret()