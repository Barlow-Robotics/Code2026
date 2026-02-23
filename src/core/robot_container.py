#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from constants import TunerConstants, DriveConstants, AutoConstants
from utils import SwerveTelemetry

from phoenix6 import swerve
from subsystems import Vision
from core.controller import Controller
from wpilib import DriverStation, SendableChooser
from utils import should_flip
import wpilib


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
        # self.shooter = Shooter(driveSub=self.drivetrain)
        # self.intake = Intake()
        # self.spindex = Spindex()

        # Telemetry
        self._swerve_telemetry = SwerveTelemetry(
            DriveConstants.MAX_TRANSLATIONAL_VELOCITY
        )
        self.drivetrain.register_telemetry(
            lambda state: self._swerve_telemetry.telemeterize(state)
        )
        AutoBuilder.configure(
            self.drivetrain.get_pose,
            self.drivetrain.reset_odometry_auto,
            self.drivetrain.get_speeds,
            lambda spds, ffs: self.drivetrain.set_robot_centric_velocities(spds),
            PPHolonomicDriveController(
                AutoConstants.auto_translation_pid,
                AutoConstants.auto_rotation_pid,
                AutoConstants.period,
            ),
            RobotConfig.fromGUISettings(),
            should_flip,
            self.drivetrain,
        )

        # Controller bindings
        self.controller = Controller(self)
        self.configure_autos()

    def configure_autos(self):
        import autos

        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("kenny path", autos.example_path_auto)

        self.auto_selection.addOption("Leave", autos.leave_auto)

        # allow us to choose our auto in Smart Dashboard
        wpilib.SmartDashboard.putData("Auto", self.auto_selection)
