from commands2 import Command
from wpimath.geometry import Pose2d
from utils import get_red_pose


class AutoRoutine(Command):
    def __init__(self, command: Command, start_pose: Pose2d):
        super().__init__()
        self.command = command
        self.blue_pose = start_pose
        self.red_pose = get_red_pose(start_pose)
