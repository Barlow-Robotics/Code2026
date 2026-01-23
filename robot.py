# TODO: insert robot code here
import commands2
import wpilib

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""

        self.scheduler = commands2.CommandScheduler.getInstance()

    def robotPeriodic(self):
        try:
            print("I am a robot")
            self.scheduler.run()
        except Exception as e:
            self.log.error(e)
            self.nt.getTable("errors").putString("command scheduler", str(e))

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)