import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine
from commands import IntakePositionCommand
from constants import RobotFeatures

from commands2 import (
    SequentialCommandGroup,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    cmd,
)

if typing.TYPE_CHECKING:
    from core import RobotContainer

path_name = "HP_Intake_Center_Pieces"


class HP_Intake_Center_Pieces:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
        ]
        self.container = container

        if RobotFeatures.HAS_INTAKE:
            from subsystems import IntakePositions

            deploy_cmd = IntakePositionCommand(
                self.container.drivetrain,
                self.container.intake,
                IntakePositions.DEPLOYED,
                auto=True,
            ).withTimeout(3)
            home_cmd = IntakePositionCommand(
                self.container.drivetrain, self.container.intake, IntakePositions.HOME, auto=True
            ).withTimeout(3)
        else:
            deploy_cmd = cmd.none()
            home_cmd = cmd.none()

        self.command = SequentialCommandGroup(
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_and_aim_command(),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[1]),
                deploy_cmd,
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                home_cmd,
            ),
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_and_aim_command(),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
