from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup


path_name = "Leave_Shoot"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)]

command = SequentialCommandGroup(
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
    )
)


auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())