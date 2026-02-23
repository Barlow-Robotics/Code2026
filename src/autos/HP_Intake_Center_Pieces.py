from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from robot import Robot


path_name = "HP_Intake_Center_Pieces"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)]

command = SequentialCommandGroup(
    # SHOOT. 
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]),
        # start intake
        # Robot.container.intake.set_velocity_command
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[2]),
        # stop intake
    ),
    # shoot
)


HP_Intake_Center_Pieces_Auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())
