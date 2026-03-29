from commands2.command import Command
import typing
from constants.robot_constants import RobotFeatures
from utils import IntakePositions

if typing.TYPE_CHECKING:
    from subsystems import Drivetrain, Intake


class IntakePositionCommand(Command):
    def __init__(
        self,
        drive_sub: "Drivetrain",
        intake_sub: "Intake",
        position: IntakePositions,
        auto=False
    ):
        super().__init__()
        self.drive_sub = drive_sub
        self.intake_sub = intake_sub
        self.position = position
        self.auto = auto
        self.addRequirements(
            intake_sub
        )  # BW: since im only controlling intake and just getting data from drive i only need to do requirements for intake

    def initialize(self):
        # current_speeds = self.drive_sub.get_speeds()
        # overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
        if self.position != IntakePositions.HOME or not self.auto:
            self.intake_sub.go_to_position(self.position, 0)

    def execute(self):
        if self.auto:
            if (
                self.position == IntakePositions.DEPLOYED
                or self.position == IntakePositions.REVERSE
            ):
                current_speeds = self.drive_sub.get_speeds()
                overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
                self.intake_sub.set_velocity(overall_velocity)
            # if self.position == IntakePositions.HOME:
            #     self.intake_sub.stop()

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass
