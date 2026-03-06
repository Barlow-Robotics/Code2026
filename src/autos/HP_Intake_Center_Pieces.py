import typing
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from autos import AutoRoutine
from commands import IntakePositionCommand
from constants import RobotFeatures

from commands2 import SequentialCommandGroup, ParallelCommandGroup, cmd

if typing.TYPE_CHECKING:
    from core import RobotContainer

path_name = "HP_Intake_Center_Pieces"


class HP_Intake_Center_Pieces:
    def __init__(self, container: "RobotContainer"):
        self.paths = [
            PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(3)
        ]
        self.container = container

        if RobotFeatures.HAS_INTAKE:
            from subsystems import IntakePositions

            deploy_cmd = IntakePositionCommand(
                self.container.drivetrain,
                self.container.intake,
                IntakePositions.DEPLOYED,
            ).withTimeout(3)
            stow_cmd = IntakePositionCommand(
                self.container.drivetrain, self.container.intake, IntakePositions.STOWED
            ).withTimeout(3)
        else:
            deploy_cmd = cmd.none()
            stow_cmd = cmd.none()

        self.command = SequentialCommandGroup(
            self.container.shoot_command_factory().withTimeout(5),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[0]),
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[1]),
                deploy_cmd,
            ),
            ParallelCommandGroup(
                AutoBuilder.followPath(self.paths[2]),
                stow_cmd,
            ),
            self.container.shoot_command_factory().withTimeout(5),
        )

    def get_command(self):
        return AutoRoutine(self.command, self.paths[0].getStartingHolonomicPose())
