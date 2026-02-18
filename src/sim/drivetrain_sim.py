from wpilib import RobotController

from subsystems.drivetrain import Drivetrain


class DrivetrainSim:
    def __init__(self, drivetrain: Drivetrain):
        self._drivetrain = drivetrain

    def update_sim(self, now: float, tm_diff: float) -> None:
        self._drivetrain.update_sim_state(tm_diff, RobotController.getBatteryVoltage())
