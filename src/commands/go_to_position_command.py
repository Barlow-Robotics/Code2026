from commands2.command import Command
import typing
from utils import IntakePositions

if typing.TYPE_CHECKING:
    from subsystems import Drivetrain, Intake


class IntakePositionCommand(Command):
    def __init__(
        self, drive_sub: "Drivetrain", intake_sub: "Intake", position: IntakePositions
    ):
        super().__init__()
        self.drive_sub = drive_sub
        self.intake_sub = intake_sub
        self.position = position
        self.addRequirements(
            intake_sub
        )  # BW: since im only controlling intake and just getting data from drive i only need to do requirements for intake

    def initialize(self):
        current_speeds = self.drive_sub.get_speeds()
        overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
        self.intake_sub.go_to_position(self.position, current_velocity=overall_velocity)

    def execute(self):
        if self.position == IntakePositions.DEPLOYED:
            current_speeds = self.drive_sub.get_speeds()
            overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
            self.intake_sub.set_velocity(overall_velocity)
        else:
            self.intake_sub.set_velocity(0)

    def isFinished(self):
        pass

    def end(self, interrupted):
        pass
