#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from __future__ import annotations
from commands2 import cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from commands.shoot_command import ShootCommand
from constants import TunerConstants, DriveConstants, AutoConstants, RobotFeatures
from utils import SwerveTelemetry

from phoenix6 import swerve

from core.controller import Controller
from wpilib import (
    Color8Bit,
    DriverStation,
    Mechanism2d,
    SendableChooser,
    SmartDashboard,
)
from utils import should_flip
import wpilib
from pykit.logger import Logger as PyKitLogger
from autos import (
    Leave_Shoot,
    HP_Center_L2,
    HP_Center_L1,
    HP_Center_L1_standalone,
    HP_Center_L2_AutoAlign,
    Simple,
)


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        RobotFeatures.configure()
        DriverStation.silenceJoystickConnectionWarning(True)

        # Swerve drive requests
        self.brake = swerve.requests.SwerveDriveBrake()
        self.point = swerve.requests.PointWheelsAt()

        # Subsystems
        self.drivetrain = TunerConstants.create_drivetrain()

        self.vision = None
        self.shooter = None
        self.turret = None
        self.intake = None
        self.spindex = None
        self.feeder = None

        from subsystems import Vision, Shooter, Turret, Intake, Spindex, Feeder

        if RobotFeatures.HAS_VISION:
            self.vision = Vision(drive_sub=self.drivetrain)
        if RobotFeatures.HAS_SHOOTER:
            self.shooter = Shooter()
        if RobotFeatures.HAS_TURRET:
            self.turret = Turret(driveSub=self.drivetrain)
        if RobotFeatures.HAS_INTAKE:
            self.intake = Intake()
        if RobotFeatures.HAS_SPINDEX:
            self.spindex = Spindex()
        if RobotFeatures.HAS_FEEDER:
            self.feeder = Feeder()

        # Combined shooter/feeder/spindex mechanism
        if RobotFeatures.LOGGING:
            self._setup_scoring_mechanism()

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
        self.create_commands()
        self.controller = Controller(self)
        self.configure_autos()

    def reinitialize_subsystems(self):
        from subsystems import Turret, Intake

        # if RobotFeatures.HAS_VISION:
        #     self.vision = Vision(drive_sub=self.drivetrain)
        # if RobotFeatures.HAS_SHOOTER:
        #     self.shooter = Shooter()
        if RobotFeatures.HAS_TURRET:
            self.turret = Turret(driveSub=self.drivetrain, init2=True)
        self.controller = Controller(self)
        if RobotFeatures.HAS_INTAKE:
            self.intake = Intake()
        # if RobotFeatures.HAS_SPINDEX:
        #     self.spindex = Spindex()
        # if RobotFeatures.HAS_FEEDER:
        #     self.feeder = Feeder()

    def configure_autos(self):

        self.auto_selection = SendableChooser()

        if (
            RobotFeatures.HAS_INTAKE
            and RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
            and RobotFeatures.HAS_TURRET
        ):
            self.auto_selection.setDefaultOption(
                "HP_Center_L2",
                HP_Center_L2(self).get_command(),
            )
            self.auto_selection.addOption("Leave", Leave_Shoot(self).get_command())
        else:
            self.auto_selection.setDefaultOption(
                "Leave", Leave_Shoot(self).get_command()
            )

        self.auto_selection.addOption("HP_Center_L1", HP_Center_L1(self).get_command())
        self.auto_selection.addOption(
            "HP_Center_L1_Standalone", HP_Center_L1_standalone(self).get_command()
        )
        self.auto_selection.addOption("HP_Center_L2", HP_Center_L2(self).get_command())
        self.auto_selection.addOption(
            "HP_Center_L2_AutoAlign", HP_Center_L2_AutoAlign(self).get_command()
        )

        
        self.auto_selection.addOption("Simple", Simple(self).get_command())
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
        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
            and RobotFeatures.HAS_TURRET
        ):
            self.shoot_command_factory = lambda: ShootCommand(
                self.drivetrain, self.shooter, self.feeder, self.spindex, self.turret
            )
        else:
            self.shoot_command_factory = lambda: cmd.none()

    def _setup_scoring_mechanism(self):
        if not (self.spindex and self.feeder and self.shooter):
            return

        self.scoring_mechanism = Mechanism2d(3, 2)

        # Spindex at (0.5, 0.5)
        spindex_root = self.scoring_mechanism.getRoot("SpindexRoot", 0.5, 0.5)
        self._spindex_ligament = spindex_root.appendLigament(
            "Spindex", 0.3, 0, 6, Color8Bit(255, 255, 0)
        )

        # Feeder at (1.5, 0.5)
        feeder_root = self.scoring_mechanism.getRoot("FeederRoot", 1.5, 0.5)
        self._feeder_ligament = feeder_root.appendLigament(
            "Feeder", 0.3, 0, 6, Color8Bit(0, 200, 0)
        )

        # Shooter at (2.5, 1.5)
        shooter_root = self.scoring_mechanism.getRoot("ShooterRoot", 2.5, 1.5)
        self._shooter_ligament = shooter_root.appendLigament(
            "Shooter", 0.4, 0, 8, Color8Bit(255, 50, 50)
        )

        SmartDashboard.putData("Scoring/Mechanism2d", self.scoring_mechanism)

        self._spindex_angle = 0.0
        self._feeder_angle = 0.0
        self._shooter_angle = 0.0

    def update_scoring_mechanism(self):
        if not hasattr(self, "scoring_mechanism"):
            return
        # Spin each ligament based on current motor velocity
        spindex_vel = float(self.spindex.motor_spindex.get_velocity().value)
        feeder_vel = float(self.feeder.motor_feeder_constant.get_velocity().value)
        shooter_vel = self.shooter.get_current_rpm()

        # Scale velocities to visible rotation rates (degrees per cycle)
        self._spindex_angle += spindex_vel * 3.6
        self._feeder_angle += feeder_vel * 3.6
        self._shooter_angle += shooter_vel * 0.12

        self._spindex_ligament.setAngle(self._spindex_angle % 360)
        self._feeder_ligament.setAngle(self._feeder_angle % 360)
        self._shooter_ligament.setAngle(self._shooter_angle % 360)
