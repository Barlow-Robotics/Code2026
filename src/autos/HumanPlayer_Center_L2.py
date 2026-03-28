import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import (
    ParallelDeadlineGroup,
    SequentialCommandGroup,
)

from commands.reverse_command_slow import ReverseCommandSlow

if typing.TYPE_CHECKING:
    from core import RobotContainer
from commands import IntakePositionCommand
from subsystems import IntakePositions

path_name = "HP_Center_2L"


class HP_Center_L2:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            # self.container.shoot_command_factory().withTimeout(5),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[0]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[1]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                    auto=True,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[2]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
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
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[3]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[4]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                    auto=True,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[5]),
                ReverseCommandSlow(self.container.feeder, self.container.spindex),
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
