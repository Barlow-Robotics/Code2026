from commands2 import Command
from subsystems.turret import Turret


class HoodSequenceCommand(Command):
    STAGING_ANGLE_DEG = 25.0
    STAGING_TOLERANCE_DEG = 1.0
    TARGET_TOLERANCE_DEG = 0.5

    def __init__(self, turret: Turret, target_angle_deg: float):
        super().__init__()
        self.turret = turret
        self.target_angle_deg = target_angle_deg
        self._at_staging = False
        self.addRequirements(turret)

    def initialize(self):
        self._at_staging = False
        self.turret.set_angle_hood(self.STAGING_ANGLE_DEG)

    def execute(self):
        actual = self.turret.get_actual_hood_angle_cancoder()

        if not self._at_staging:
            if abs(actual - self.STAGING_ANGLE_DEG) <= self.STAGING_TOLERANCE_DEG:
                self._at_staging = True
                self.turret.set_angle_hood(self.target_angle_deg)
        
    def isFinished(self) -> bool:
        if not self._at_staging:
            return False
        actual = self.turret.get_actual_hood_angle_cancoder()
        return abs(actual - self.target_angle_deg) <= self.TARGET_TOLERANCE_DEG

    def end(self, interrupted: bool):
        if interrupted:
            self.turret.set_angle_hood(0)