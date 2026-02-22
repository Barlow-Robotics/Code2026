#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import cmd

from constants import TunerConstants, DriveConstants
from utils import SwerveTelemetry

from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from subsystems import Vision, Intake, Spindex
from core.controller import Controller


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        DriverStation.silenceJoystickConnectionWarning(True)

        # Swerve drive requests
        self.brake = swerve.requests.SwerveDriveBrake()
        self.point = swerve.requests.PointWheelsAt()

        # Subsystems
        self.drivetrain = TunerConstants.create_drivetrain()
        self.vision = Vision(drive_sub=self.drivetrain)
        #self.intake = Intake()
        #self.spindex = Spindex()

        # Telemetry
        self._swerve_telemetry = SwerveTelemetry(
            DriveConstants.MAX_TRANSLATIONAL_VELOCITY
        )
        self.drivetrain.register_telemetry(
            lambda state: self._swerve_telemetry.telemeterize(state)
        )

        # Controller bindings
        self.controller = Controller(self)
