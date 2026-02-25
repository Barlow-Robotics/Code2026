from __future__ import annotations

from typing import TYPE_CHECKING

from commands2 import cmd
from commands2.button import CommandXboxController, CommandJoystick, Trigger

from commands2.sysid import SysIdRoutine
import wpilib
from phoenix6 import swerve
from wpilib import DriverStation, RobotBase
from wpimath.geometry import Rotation2d
from constants import DriveConstants

if TYPE_CHECKING:
    from core import RobotContainer


class Controller:
    _JOYSTICK_PORT = 0
    _DRIVER_PORT = 1

    def __init__(self, container: RobotContainer):
        self._joystick = CommandXboxController(self._JOYSTICK_PORT)
        self._driver = CommandJoystick(self._DRIVER_PORT)

        # Drivetrain default command (field-centric drive)
        # Note that X is forward and Y is left per WPILib convention.
        container.drivetrain.setDefaultCommand(
            container.drivetrain.apply_request(
                lambda: (
                    container.drivetrain.movement.with_velocity_x(
                        -self._joystick.getLeftY()
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                    )
                    .with_velocity_y(
                        -self._joystick.getLeftX()
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                    )
                    .with_rotational_rate(
                        -self._joystick.getRightX()
                        * DriveConstants.MAX_ANGULAR_VELOCITY
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
        # self._driver.button(2).onTrue(container.intake.set_velocity_command)

        if RobotBase.isReal() is False:
            self._joystick.button(1).onTrue(
                container.spindex.sysIdDynamicSpindex(SysIdRoutine.Direction.kForward)
            )
            self._joystick.button(2).onTrue(
                container.spindex.sysIdQuasistaticSpindex(
                    SysIdRoutine.Direction.kReverse
                )
            )
            self._joystick.button(3).onTrue(
                container.spindex.sysIdDynamicSpindex(SysIdRoutine.Direction.kForward)
            )
            self._joystick.button(4).onTrue(
                container.spindex.sysIdDynamicSpindex(SysIdRoutine.Direction.kReverse)
            )
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))

            # self._driver.button(2).onTrue(container.vision.auto_align_command)

            # self._driver.button(3).onTrue(container.intake.stop_command)
            # self._driver.button(4).onTrue(
            #     container.intake.goto_position_cmmand[IntakePositions.DEPLOYED]
            # )
            # self._driver.button(5).onTrue(
            #     container.intake.goto_position_cmmand[IntakePositions.STOWED]
            # )
            # self._driver.button(6).onTrue(
            #     container.intake.goto_position_cmmand[IntakePositions.HOME]
            # )

        # Periodically warn about missing controllers during teleop
        Trigger(DriverStation.isTeleopEnabled).whileTrue(
            cmd.sequence(
                cmd.waitSeconds(5),
                cmd.runOnce(self._log_missing_connections),
            ).repeatedly()
        )

    def _log_missing_connections(self):
        if not DriverStation.isJoystickConnected(self._JOYSTICK_PORT):
            wpilib.reportWarning(
                f"Xbox controller not connected on port {self._JOYSTICK_PORT}"
            )
        if not DriverStation.isJoystickConnected(self._DRIVER_PORT):
            wpilib.reportWarning(
                f"Driver joystick not connected on port {self._DRIVER_PORT}"
            )
