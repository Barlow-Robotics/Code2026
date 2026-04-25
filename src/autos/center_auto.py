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

path_name = "NotRunning_Depot_Intake_path"


class Depot_Side_Balls:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
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
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[1]),
                IntakePositionCommand(
                    self.container.intake,
                    IntakePositions.DEPLOYED,
                    activate_rollers=True,
                ).withTimeout(5),
            ),
            ParallelDeadlineGroup(
                AutoBuilder.followPath(self.paths[2]),
                IntakePositionCommand(
                    self.container.intake,
                    IntakePositions.HOME,
                    activate_rollers=False,
                ).withTimeout(3),
            ),
            ParallelDeadlineGroup(
                self.container.shoot_command_factory().withTimeout(5),
                self.container.drivetrain.hold_position_command(),
            ),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
