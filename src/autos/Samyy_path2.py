import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

if typing.TYPE_CHECKING:
    from core import RobotContainer
from subsystems import IntakePositions

path_name = "N_BETTER_Samyy_Path"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)]

command = SequentialCommandGroup(
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]),
        # Start intake
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[2]),
        # Stop intake
    ),
     ParallelCommandGroup(
        AutoBuilder.followPath(paths[3]),
        # Shoot 'yer balls
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[4]),
        # Start intake
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[5]),
        # Stop intake
    ),
    # Shoot 'yer balls
)

N_BETTER_Samyy_Path = AutoRoutine(command, paths[0].getStartingHolonomicPose())