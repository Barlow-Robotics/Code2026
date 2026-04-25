#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from phoenix6.swerve import Pose2d
import wpilib
import commands2
import typing

from utils import (
    configure_pykit,
)  # DO NOT MOVE IMPORT, it needs to be called before any PyKit loggers are created

from core import RobotContainer
from wpilib import DriverStation, SmartDashboard
from constants import RobotFeatures
from utils.profiler import LoopTimer

if typing.TYPE_CHECKING:
    from autos import AutoRoutine

from pykit.loggedrobot import LoggedRobot

LoggedRobot.default_period = 0.04  # seconds slowed down to from 0.02 -> 0.04 to reduce


class Robot(
    LoggedRobot
):  # BW: Don't worry LoggedRobot does the same thing as TimedRobot, just with logging.
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def __init__(self):

        super().__init__()
        # if RobotBase.isReal():
        #     SignalLogger.set_path("/home/lvuser/logs")
        # else:
        #     SignalLogger.set_path("logs")
        self.useTiming = configure_pykit(type(self).__name__)
        # SignalLogger.start()

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
        # if RobotFeatures.TESTING:
        #     self._time_and_joystick_replay = (
        #         HootAutoReplay().with_timestamp_replay().with_joystick_replay()
        #     )

        # --- Profiler setup (sim-only by default) ---
        if RobotFeatures.HAS_CPROFILE:
            from utils.profiler import PeriodicProfiler

            self._cprofile = PeriodicProfiler()
        else:
            self._cprofile = None
        self._scheduler_timer = LoopTimer("Scheduler")
        if RobotFeatures.TESTING and RobotFeatures.SmartDashboardTuning:
            SmartDashboard.putNumber("hood_angle", 25)
            SmartDashboard.putNumber("hood_angle_final", 0)

    def robotPeriodic(self) -> None:
        # self._time_and_joystick_replay.update()

        self._scheduler_timer.start()
        commands2.CommandScheduler.getInstance().run()
        if RobotFeatures.LOGGING_ROBOT:
            self.container.update_scoring_mechanism()
        self._scheduler_timer.stop()

    def disabledInit(self) -> None:
        # self.container.turret.
        # .set_position(0, 0.2)
        """This function is called once each time the robot enters Disabled mode."""
        if self._cprofile:
            self._cprofile.disable()

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
        if RobotFeatures.TESTING:
            self.container.reinitialize_subsystems()

        if self._cprofile:
            self._cprofile.enable()

        self.container.turret.set_angle_hood(0)
        self.container.intake.stop_rollers()
        self.container.feeder.stop()
        self.container.shooter.stop_flywheel()
        self.container.spindex.stop()

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
