#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import cmd

from utils import TunerConstants, Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from subsystems import Vision, Intake, Spindex


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self.max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Swerve drive requests
        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self.max_speed * 0.1)
            .with_rotational_deadband(
                self.max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self.brake = swerve.requests.SwerveDriveBrake()
        self.point = swerve.requests.PointWheelsAt()

        # Subsystems
        self.drivetrain = TunerConstants.create_drivetrain()
        self.vision = Vision(drive_sub=self.drivetrain)
        self.intake = Intake()
        self.spindex = Spindex()

        # Telemetry
        self._logger = Telemetry(self.max_speed)
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Simple drive forward auton
        idle = swerve.requests.Idle()
        return cmd.sequence(
            # Reset our field centric heading to match the robot
            # facing away from our alliance station wall (0 deg).
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
            ),
            # Then slowly drive forward (away from us) for 5 seconds.
            self.drivetrain.apply_request(
                lambda: (
                    self.drive.with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            ).withTimeout(5.0),
            # Finally idle for the rest of auton
            self.drivetrain.apply_request(lambda: idle),
        )
