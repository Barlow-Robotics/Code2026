from commands2 import Command
from pathplannerlib.config import dataclass
from wpimath.geometry import Pose2d
from utils import get_red_pose


@dataclass
class AutoRoutine:
    def __init__(self, command: Command, start_pose: Pose2d):
        self.command = command
        self.blue_pose = start_pose
        self.red_pose = get_red_pose(start_pose)
