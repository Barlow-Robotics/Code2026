#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from datetime import datetime
from phoenix6.swerve import Pose2d
import wpilib
import commands2
import typing

from core import RobotContainer
from phoenix6 import HootAutoReplay
from wpilib import DriverStation
from utils import configure_pykit

if typing.TYPE_CHECKING:
    from autos import AutoRoutine

from phoenix6 import SignalLogger

from pykit.loggedrobot import LoggedRobot


class Robot(LoggedRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def __init__(self):
        super().__init__()
        SignalLogger.start()
        self.useTiming = configure_pykit(
            type(self).__name__, datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        )

    def robotInit(self) -> None:
        """Robot initialization function"""
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.autonomousCommand: typing.Optional[commands2.Command] = None

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

        # log and replay timestamp and joystick data
        self._time_and_joystick_replay = (
            HootAutoReplay().with_timestamp_replay().with_joystick_replay()
        )

    def robotPeriodic(self) -> None:
        self._time_and_joystick_replay.update()

        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand: "AutoRoutine" = (
            self.container.auto_selection.getSelected()
        )
        starting_pose: Pose2d = (
            self.autonomousCommand.red_pose
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else self.autonomousCommand.blue_pose
        )
        self.container.drivetrain.reset_odometry_auto(starting_pose)
        if self.autonomousCommand:
            commands2.CommandScheduler.getInstance().schedule(
                self.autonomousCommand.command
            )

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            commands2.CommandScheduler.getInstance().cancel(
                self.autonomousCommand.command
            )

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(Robot)
