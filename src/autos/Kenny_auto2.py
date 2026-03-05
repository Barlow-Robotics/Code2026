import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

if typing.TYPE_CHECKING:
    from core import RobotContainer
from subsystems import IntakePositions

path_name = "N_Kenny_Path"

class Kenny_auto2:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            self.container.shoot_command_factory().withTimeout(5),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[1]),
                self.container.intake.goto_position_command_factory(
                    IntakePositions.DEPLOYED
                ),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                self.container.intake.goto_position_command_factory(
                    IntakePositions.STOWED
                ),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[3]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[4]),
                self.container.intake.goto_position_command_factory(
                    IntakePositions.DEPLOYED
                ),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[5]),
                self.container.intake.goto_position_command_factory(
                    IntakePositions.STOWED
                ),
            ),
            self.container.shoot_command_factory().withTimeout(5),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
