import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

if typing.TYPE_CHECKING:
    from core import RobotContainer
from subsystems import IntakePositions

path_name = "HP_Intake_Center_Pieces"


class HP_Intake_Center_Pieces:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            self.container.shoot_command_factory().withTimeout(0),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[1]),
                self.container.intake.goto_position_command_factory(IntakePositions.DEPLOYED),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                self.container.intake.goto_position_command_factory(IntakePositions.STOWED),
            ),
            self.container.shoot_command_factory().withTimeout(0),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
