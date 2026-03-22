from typing import TYPE_CHECKING
from commands2 import Command
from wpilib import SmartDashboard
from constants import ShooterConstants
from constants.field_constants import CustomPoints
from constants.robot_constants import RobotFeatures
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
        self.target_velocity = SmartDashboard.getNumber(
            "CustomFloatVelocity", ShooterConstants.FLYWHEEL_VELOCITY_CONSTANT
        )
        self.shooter.set_velocity(self.target_velocity)

    def execute(self):
        if not self.driveSub.allow_center_auto_align:
            if RobotFeatures.HAS_TURRET:
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
            if RobotFeatures.HAS_TURRET:
                self.turret.set_target_hood_and_turret(shooting_location=target_pose)

        if self._feeding:
            return
        current_velocity = self.shooter.get_current_velocity()
        if (
            abs(current_velocity - self.target_velocity)
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
        if RobotFeatures.HAS_TURRET:
            self.turret.reset_target_hood_and_turret()