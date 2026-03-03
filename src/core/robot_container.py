#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from commands.shoot_command import ShootCommand
from constants import TunerConstants, DriveConstants, AutoConstants
from utils import SwerveTelemetry

from phoenix6 import swerve

from core.controller import Controller
from wpilib import DriverStation, RobotBase, SendableChooser
from utils import should_flip
import wpilib
from pykit.logger import Logger as PyKitLogger
from autos import HP_Intake_Center_Pieces, Leave_Shoot


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
        from subsystems import Intake

        # self.drivetrain = TunerConstants.create_drivetrain()

        self.intake = Intake()
        # self.shooter = Shooter()
        # self.turret = Turret(driveSub=self.drivetrain)
        # self.intake = Intake()
        # self.spindex = Spindex()
        # self.feeder = Feeder()
        # self.vision = Vision(drive_sub=self.drivetrain)

        # if RobotBase.isReal() is False:


        # Telemetry
        # self._swerve_telemetry = SwerveTelemetry(
        #     DriveConstants.MAX_TRANSLATIONAL_VELOCITY
        # )
        # self.drivetrain.register_telemetry(
        #     lambda state: self._swerve_telemetry.telemeterize(state)
        # )
        # AutoBuilder.configure(
        #     self.drivetrain.get_pose,
        #     self.drivetrain.reset_odometry_auto,
        #     self.drivetrain.get_speeds,
        #     lambda spds, ffs: self.drivetrain.set_robot_centric_velocities(spds),
        #     PPHolonomicDriveController(
        #         AutoConstants.auto_translation_pid,
        #         AutoConstants.auto_rotation_pid,
        #         AutoConstants.period,
        #     ),
        #     RobotConfig.fromGUISettings(),
        #     should_flip,
        #     self.drivetrain,
        # )

        # Controller bindings
        self.controller = Controller(self)
        # self.create_commands()
        # self.configure_autos()

    def configure_autos(self):

        self.auto_selection = SendableChooser()

        self.auto_selection.setDefaultOption(
            "HP_Intake_Center_Pieces", HP_Intake_Center_Pieces(self).get_command()
        )

        self.auto_selection.addOption("Leave", Leave_Shoot(self).get_command())

        # allow us to choose our auto in Smart Dashboard
        wpilib.SmartDashboard.putData("Auto", self.auto_selection)
        prefix = "Auto"
        PyKitLogger.recordOutput(f"{prefix}/option_0", "HP_Intake_Center_Pieces")
        PyKitLogger.recordOutput(f"{prefix}/option_1", "kenny path")
        PyKitLogger.recordOutput(f"{prefix}/option_2", "Leave")
        PyKitLogger.recordOutput(f"{prefix}/default", "HP_Intake_Center_Pieces")

        prefix = prefix + "/PathPlanner"
        PathPlannerLogging.setLogCurrentPoseCallback(
            lambda pose: PyKitLogger.recordOutput(f"{prefix}/current_pose", pose)
        )
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda pose: PyKitLogger.recordOutput(f"{prefix}/target_pose", pose)
        )
        PathPlannerLogging.setLogActivePathCallback(
            lambda poses: PyKitLogger.recordOutput(f"{prefix}/active_path", poses)
        )

    def create_commands(self):
        self.shoot_command_factory = lambda: ShootCommand(
            self.shooter, self.feeder, self.spindex
        )
