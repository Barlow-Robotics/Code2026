import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import (
    ParallelDeadlineGroup,
    SequentialCommandGroup,
)

if typing.TYPE_CHECKING:
    from core import RobotContainer
from commands import IntakePositionCommand
from subsystems import IntakePositions

path_name = "HP_Center_1L_standalone"


class HP_Center_L1_standalone:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
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
                    auto=True,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[2]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.HOME,
                    auto=True,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_and_aim_command(),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
