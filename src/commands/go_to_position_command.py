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

    The ``arm_timeout`` is forwarded to ``intake.deploy()`` /
    ``intake.retract()``: the SM forces the transition out of
    DEPLOYING / GOING_HOME after that many seconds even if Motion Magic
    hasn't finished its profile (covers stuck-arm cases).
    """

    def __init__(
        self,
        intake_sub: "Intake",
        position: IntakePositions,
        activate_rollers: bool | None = None,
        arm_timeout: float | None = None,
    ):
        super().__init__()
        self.intake_sub = intake_sub
        self.position = position
        self.activate_rollers = activate_rollers
        self.arm_timeout = arm_timeout
        self.addRequirements(intake_sub)

    def initialize(self):
        if self.position == IntakePositions.HOME:
            self.intake_sub.retract(self.arm_timeout)
        elif self.position == IntakePositions.DEPLOYED:
            self.intake_sub.deploy(self.arm_timeout)

        if self.activate_rollers is True:
            self.intake_sub.activate_roller()
        elif self.activate_rollers is False:
            self.intake_sub.stop_rollers()

    def isFinished(self):
        return True
