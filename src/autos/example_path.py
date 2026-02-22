from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup


path_name = "kenny_path"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)]

command = SequentialCommandGroup(
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]),
        # Do something
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[2]),
        # Do Smothing
    ),
    # FeedOut(Robot.wrist).withTimeout(.2),
)


example_path_auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())
