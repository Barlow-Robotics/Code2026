from commands2.command import Command
import typing

from utils import IntakePositions

if typing.TYPE_CHECKING:
    from subsystems import Intake


class IntakePositionCommand(Command):
    """Requests an arm position, optionally controlling the rollers.

    The ``activate_rollers`` parameter is trinary:
        - ``True``  → start the rollers in initialize()
        - ``False`` → stop the rollers in initialize()
        - ``None``  → leave the rollers alone (default)

    The ``arm_timeout`` is forwarded to ``intake.deploy()`` /
    ``intake.retract()`` and tells the state machine to declare the
    move complete after that many seconds even if Motion Magic hasn't
    finished its profile (covers stuck-arm cases).

    Long-running: never finishes on its own; runs until interrupted by
    another command requiring the intake. ``end()`` is intentionally a
    no-op — roller state changes only happen on explicit request.
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
        return False
