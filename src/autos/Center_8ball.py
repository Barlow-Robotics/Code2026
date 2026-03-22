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

path_name = "Center_8Ball"


class Center_Auto:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            # self.container.shoot_command_factory().withTimeout(5),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_command(),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
