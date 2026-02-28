from typing import TYPE_CHECKING
from commands2 import Command

if TYPE_CHECKING:
    from subsystems import Spindex
    from subsystems.feeder import Feeder


class MoveSpindexAndFeederCommand(Command):
    def __init__(
        self, spindex: "Spindex", feeder: "Feeder", velocity: float = 1, invert=False
    ):
        super().__init__()
        self.spindex = spindex
        self.feeder = feeder
        self.velocity = velocity
        self.invert = invert
        self.addRequirements(spindex, feeder)

    def initialize(self):
        self.spindex.move(self.velocity)
        self.feeder.move(self.velocity, self.invert)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.spindex.stop()
        self.feeder.stop()
