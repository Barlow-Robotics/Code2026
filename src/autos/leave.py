import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

if typing.TYPE_CHECKING:
    from core import RobotContainer


path_name = "Leave_Shoot"


class Leave_Shoot:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
