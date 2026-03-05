from __future__ import annotations

from typing import TYPE_CHECKING

from commands2 import cmd
from commands2.button import CommandXboxController, CommandJoystick, Trigger

import wpilib
from phoenix6 import swerve
from wpilib import DriverStation, RobotBase
from wpimath.geometry import Rotation2d
from constants import DriveConstants, Constants
from subsystems.intake import IntakePositions

if TYPE_CHECKING:
    from core import RobotContainer


class Controller:
    _OPERATOR_PORT = 1
    _DRIVER_PORT = 0

    def __init__(self, container: "RobotContainer"):
        self._operator = CommandXboxController(self._OPERATOR_PORT)
        self._driver = CommandJoystick(self._DRIVER_PORT)
        if RobotBase.isReal() and Constants.robotTesting:
            # self._driver.button(4).onTrue(
            #     container.intake.goto_position_command_factory(IntakePositions.DEPLOYED)
            # )
            container.drivetrain.setDefaultCommand(
                container.drivetrain.apply_request(
                    lambda: (
                        container.drivetrain.movement.with_velocity_x(
                            self._driver.getRawAxis(1)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        )
                        .with_velocity_y(
                            self._driver.getRawAxis(0)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        )
                        .with_rotational_rate(
                            -self._driver.getRawAxis(2)
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
            self._operator.a().whileTrue(
                container.drivetrain.apply_request(lambda: container.brake)
            )
            self._operator.b().whileTrue(
                container.drivetrain.apply_request(
                    lambda: container.point.with_module_direction(
                        Rotation2d(
                            -self._operator.getLeftY(), -self._operator.getLeftX()
                        )
                    )
                )
            )
            self._driver.button(3).onTrue(container.vision.auto_align_command)
            self._driver.button(12).onTrue(lambda: container.drivetrain.reset_gyro())

        else:
            # Drivetrain default command (field-centric drive)
            # Note that X is forward and Y is left per WPILib convention.
            container.drivetrain.setDefaultCommand(
                container.drivetrain.apply_request(
                    lambda: (
                        container.drivetrain.movement.with_velocity_x(
                            self._driver.getRawAxis(1)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        )
                        .with_velocity_y(
                            self._driver.getRawAxis(0)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        )
                        .with_rotational_rate(
                            -self._driver.getRawAxis(2)
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
            self._operator.a().whileTrue(
                container.drivetrain.apply_request(lambda: container.brake)
            )
            self._operator.b().whileTrue(
                container.drivetrain.apply_request(
                    lambda: container.point.with_module_direction(
                        Rotation2d(
                            -self._operator.getLeftY(), -self._operator.getLeftX()
                        )
                    )
                )
            )

            # Intake and spindex bindings (driver joystick)
            self._driver.button(2).onTrue(container.intake.set_velocity_command)
            self._driver.button(3).onTrue(container.vision.auto_align_command)
            self._driver.button(4).onTrue(container.shooter.start_flywheel_command)

            # if RobotBase.isReal() is False:
            # self._driver.button(1).onTrue(
            #     container.shooter.sysIdDynamicTurret(SysIdRoutine.Direction.kForward)
            # )
            # self._driver.button(2).onTrue(
            #     container.shooter.sysIdDynamicTurret(SysIdRoutine.Direction.kReverse)
            # )

            # self._driver.button(3).onTrue(
            #     container.shooter.sysIdQuasistaticTurret(
            #         SysIdRoutine.Direction.kForward
            #     )
            # )
            # self._driver.button(4).onTrue(
            #     container.shooter.sysIdQuasistaticTurret(
            #         SysIdRoutine.Direction.kReverse
            #     )
            # )

            # self._operator.button(3).onTrue(
            #     container.shooter.sysIdDynamicHood(SysIdRoutine.Direction.kForward)
            # )
            # self._operator.button(4).onTrue(
            #     container.shooter.sysIdDynamicHood(SysIdRoutine.Direction.kReverse)
            # )

            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            # self._joystick.button(4).onTrue(container.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
            # self._driver.button(1).onTrue(container.vision.auto_align_command)

            # self._driver.button(3).onTrue(container.intake.stop_command)
            # self._driver.button(1).onTrue(
            #     container.intake.goto_position_command[IntakePositions.DEPLOYED]
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
        if not DriverStation.isJoystickConnected(self._OPERATOR_PORT):
            wpilib.reportWarning(
                f"Xbox controller not connected on port {self._OPERATOR_PORT}"
            )
        if not DriverStation.isJoystickConnected(self._DRIVER_PORT):
            wpilib.reportWarning(
                f"Driver joystick not connected on port {self._DRIVER_PORT}"
            )
