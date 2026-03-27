import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import (
    SequentialCommandGroup,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
)

if typing.TYPE_CHECKING:
    from core import RobotContainer
from commands import IntakePositionCommand
from subsystems import IntakePositions

path_name = "N_Kenny_Path"


class Kenny_auto2:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)
        ]
        self.container = container

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
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                ).withTimeout(3),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.HOME,
                ).withTimeout(3),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[3]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[4]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                ).withTimeout(3),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[5]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.HOME,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_and_aim_command(),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
