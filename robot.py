# TODO: insert robot code here
import commands2
import wpilib

from controller import Controller
from subsystems import Intake


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.intakeSubsystem = Intake()
        self.controller = Controller(self.intakeSubsystem)
        self.scheduler = commands2.CommandScheduler.getInstance()

    def robotPeriodic(self):
        try:
            self.scheduler.run()
        except Exception as e:
            print(e)
            # self.nt.getTable("errors").putString("command scheduler", str(e))

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self):
        self.controller.setupTeleop()
        

    def teleopPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
