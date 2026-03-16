import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, ParallelCommandGroup

if typing.TYPE_CHECKING:
    from core import RobotContainer
from commands import IntakePositionCommand
from subsystems import IntakePositions

path_name = "Depot_intake_path"

class Depot_intake_center_path:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
        ]
        self.container = container

        self.command = SequentialCommandGroup(
            self.container.shoot_command_factory().withTimeout(5),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[1]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.DEPLOYED
                ).withTimeout(3),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                IntakePositionCommand(
                    self.container.drivetrain,
                    self.container.intake,
                    IntakePositions.HOME
                ).withTimeout(3),
            ),
            self.container.shoot_command_factory().withTimeout(5),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
