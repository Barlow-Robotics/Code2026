from __future__ import annotations

from typing import TYPE_CHECKING

from commands2.button import CommandXboxController, CommandJoystick, Trigger

from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from subsystems import IntakePositions

if TYPE_CHECKING:
    from core import RobotContainer


class Controller:
    def __init__(self, container: RobotContainer):
        self.container = container
        self._joystick = CommandXboxController(0)
        self._driver = CommandJoystick(1)
        self._configured = False

    def setupTeleop(self):
        if self._configured:
            return
        self._configured = True

        container = self.container

        # Drivetrain default command (field-centric drive)
        # Note that X is forward and Y is left per WPILib convention.
        container.drivetrain.setDefaultCommand(
            container.drivetrain.apply_request(
                lambda: (
                    container.drive.with_velocity_x(
                        -self._joystick.getLeftY() * container.max_speed
                    )
                    .with_velocity_y(
                        -self._joystick.getLeftX() * container.max_speed
                    )
                    .with_rotational_rate(
                        -self._joystick.getRightX() * container.max_angular_rate
                    )
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            container.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Drivetrain button bindings (xbox controller)
        self._joystick.a().whileTrue(
            container.drivetrain.apply_request(lambda: container.brake)
        )
        self._joystick.b().whileTrue(
            container.drivetrain.apply_request(
                lambda: container.point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Intake and spindex bindings (driver joystick)
        self._driver.button(2).onTrue(container.intake.set_velocity_command)
        self._driver.button(3).onTrue(container.intake.stop_command)
        self._driver.button(4).onTrue(
            container.intake.goto_position_cmmand[IntakePositions.DEPLOYED]
        )
        self._driver.button(5).onTrue(
            container.intake.goto_position_cmmand[IntakePositions.STOWED]
        )
        self._driver.button(6).onTrue(
            container.intake.goto_position_cmmand[IntakePositions.HOME]
        )
        self._driver.button(7).onTrue(container.spindex.set_velocity_command)
        self._driver.button(8).onTrue(container.spindex.stop_velocity_command)
