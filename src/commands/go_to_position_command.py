from commands2.command import Command
import typing

from utils import IntakePositions

if typing.TYPE_CHECKING:
    from subsystems import Intake


class IntakePositionCommand(Command):
    """One-shot: requests an arm position (and optionally toggles the
    rollers), then immediately finishes. The intake subsystem's state
    machine drives the actual motion to completion in the background.

    The ``activate_rollers`` parameter is trinary:
        - ``True``  → start the rollers
        - ``False`` → stop the rollers
        - ``None``  → leave the rollers alone (default)
    """

    def __init__(
        self,
        intake_sub: "Intake",
        position: IntakePositions,
        activate_rollers: bool | None = None,
    ):
        super().__init__()
        self.intake_sub = intake_sub
        self.position = position
        self.activate_rollers = activate_rollers
        self.addRequirements(intake_sub)

    def initialize(self):
        if self.position == IntakePositions.HOME:
            self.intake_sub.retract()
        elif self.position == IntakePositions.DEPLOYED:
            self.intake_sub.deploy()

        if self.activate_rollers is True:
            self.intake_sub.activate_roller()
        elif self.activate_rollers is False:
            self.intake_sub.stop_rollers()

    def isFinished(self):
        return True
