import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import (
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    ParallelCommandGroup,
)

if typing.TYPE_CHECKING:
    from core import RobotContainer
from commands import IntakePositionCommand
from subsystems import IntakePositions

path_name = "Depot_Center_1L"


class HP_Center_L1:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(5)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            # self.container.shoot_command_factory().withTimeout(5),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[1]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
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
                SequentialCommandGroup(
                    AutoBuilder.followPath(self.paths[4]),
                    # AutoBuilder.followPath(self.paths[5]),
                    # self.container.drivetrain.hold_position_command(),
                ),
                self.container.shoot_command_factory().withTimeout(15),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
