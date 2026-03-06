from __future__ import annotations

from typing import TYPE_CHECKING

from commands2 import cmd
from commands2.button import CommandXboxController, CommandJoystick, Trigger

import wpilib
from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from commands import IntakePositionCommand
from constants import DriveConstants, RobotFeatures
from subsystems.intake import IntakePositions

if TYPE_CHECKING:
    from core import RobotContainer


class Controller:
    _OPERATOR_PORT = 1
    _DRIVER_PORT = 0

    def __init__(self, container: "RobotContainer"):
        self._operator = CommandXboxController(self._OPERATOR_PORT)
        self._driver = CommandJoystick(self._DRIVER_PORT)

        # Drivetrain default command (field-centric drive)
        # Note that X is forward and Y is left per WPILib convention.
        container.drivetrain.setDefaultCommand(
            container.drivetrain.apply_request(
                lambda: (
                    container.drivetrain.movement.with_velocity_x(
                        -self._driver.getRawAxis(1)
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        * (((self._driver.getThrottle() + 1) * (1 - 0.4)) / 2 + 0.4)
                    )
                    .with_velocity_y(
                        -self._driver.getRawAxis(0)
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        * (((self._driver.getThrottle() + 1) * (1 - 0.4)) / 2 + 0.4)
                    )
                    .with_rotational_rate(
                        -self._driver.getRawAxis(2)
                        * DriveConstants.MAX_ANGULAR_VELOCITY
                        * (((self._driver.getThrottle() + 1) * (1 - 0.4)) / 2 + 0.4)
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
                    Rotation2d(-self._operator.getLeftY(), -self._operator.getLeftX())
                )
            )
        )

        # Subsystem button bindings (driver joystick)
        if RobotFeatures.HAS_INTAKE:
            current_speeds = container.drivetrain.get_speeds()
            overall_velocity = (current_speeds.vx**2 + current_speeds.vy**2) ** 0.5
            self._driver.button(2).onTrue(
                container.intake.go_to_velocity_command_factory(overall_velocity)
            )
            self._driver.button(4).onTrue(
                IntakePositionCommand(
                    container.drivetrain, container.intake, IntakePositions.DEPLOYED
                )
            ).onFalse(
                IntakePositionCommand(
                    container.drivetrain, container.intake, IntakePositions.STOWED
                )
            )
        if RobotFeatures.HAS_VISION:
            self._driver.button(3).onTrue(container.vision.auto_align_command)
        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
        ):
            self._driver.button(5).whileTrue(container.shoot_command_factory())

        self._driver.button(12).onTrue(cmd.runOnce(container.drivetrain.reset_gyro))

        # SysId bindings (uncomment as needed)
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
