from typing import TYPE_CHECKING
from commands2 import Command

if TYPE_CHECKING:
    from subsystems.feeder import Feeder
    from subsystems import Spindex


class ThrowFeederCommand(Command):
    def __init__(self, feeder: "Feeder", spindex: "Spindex"):
        super().__init__()
        self.feeder = feeder
        self.spindex = spindex
        self._feeding = False
        self.addRequirements(feeder, spindex)

    def initialize(self):
        self._feeding = False

    def execute(self):
        if self._feeding:
            return
        self.feeder.move(9, invert=True)
        self.spindex.move(9 * 3)
        self._feeding = True

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.feeder.stop()
        self.spindex.stop()
