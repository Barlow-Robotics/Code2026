from __future__ import annotations

from enum import Enum
import math
from typing import TYPE_CHECKING

from commands2 import cmd
from commands2.button import CommandXboxController, CommandJoystick, Trigger

import wpilib
from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from commands import IntakePositionCommand, ShootCommand
from commands.throw_feeder_command import ThrowFeederCommand
from constants import DriveConstants, RobotFeatures
from subsystems.intake import IntakePositions

if TYPE_CHECKING:
    from core import RobotContainer


class StartingState(Enum):
    SLOW = 0.5
    NORMAL = 1
    FAST = 2


class Controller:
    _OPERATOR_PORT = 1
    _DRIVER_PORT = 0

    def __init__(self, container: "RobotContainer"):
        self._operator = CommandXboxController(self._OPERATOR_PORT)
        self._driver = CommandXboxController(self._DRIVER_PORT)
        self._current_state = StartingState.NORMAL.value
        # Drivetrain default command (field-centric drive)
        # Note that X is forward and Y is left per WPILib convention.
        container.drivetrain.setDefaultCommand(
            container.drivetrain.apply_request(
                lambda: (
                    container.drivetrain.movement.with_velocity_x(
                        -self._driver.getRawAxis(1)
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        * self._current_state
                    )
                    .with_velocity_y(
                        -self._driver.getRawAxis(0)
                        * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                        * self._current_state
                    )
                    .with_rotational_rate(
                        -self._driver.getRawAxis(
                            2 if type(self._driver) is CommandJoystick else 4
                        )
                        * DriveConstants.MAX_ANGULAR_VELOCITY
                        * self._current_state
                    )
                )
            )
        )

        if RobotFeatures.HAS_VISION:
            for controller in [self._driver, self._operator]:
                controller.x().whileTrue(
                    container.drivetrain.apply_request(
                        lambda: container.vision.auto_align_modify_request(
                            container.drivetrain.movement.with_velocity_x(
                                -self._driver.getRawAxis(1)
                                * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                                * self._current_state
                            )
                            .with_velocity_y(
                                -self._driver.getRawAxis(0)
                                * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                                * self._current_state
                            )
                            .with_rotational_rate(
                                -self._driver.getRawAxis(
                                    2 if type(self._driver) is CommandJoystick else 4
                                )
                                * DriveConstants.MAX_ANGULAR_VELOCITY
                                * self._current_state
                            )
                        )
                    )
                )

        for controller in [self._driver, self._operator]:
            controller.rightBumper().onTrue(
                cmd.runOnce(
                    lambda: (
                        self.update_state(sub=False)
                        if round(self._current_state, 2) < 1.0
                        else print("HI2")
                    )
                )
            )

            controller.leftBumper().onTrue(
                cmd.runOnce(
                    lambda: (
                        self.update_state(sub=True)
                        if round(self._current_state, 2) > 0.5
                        else print("hi")
                    )
                )
            )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            container.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Subsystem button bindings
        if RobotFeatures.HAS_INTAKE:
            for controller in [self._driver, self._operator]:
                controller.leftStick().onTrue(
                    IntakePositionCommand(
                        drive_sub=container.drivetrain,
                        intake_sub=container.intake,
                        position=IntakePositions.DEPLOYED,
                    )
                )

                controller.a().onTrue(
                    IntakePositionCommand(
                        drive_sub=container.drivetrain,
                        intake_sub=container.intake,
                        position=IntakePositions.HOME,
                    )
                )

                controller.y().onTrue(
                    IntakePositionCommand(
                        drive_sub=container.drivetrain,
                        intake_sub=container.intake,
                        position=IntakePositions.DEPLOYED,
                    )
                )

        for controller in [self._driver, self._operator]:
            controller.button(7).onTrue(cmd.runOnce(container.drivetrain.reset_gyro))

        if RobotFeatures.HAS_VISION:
            for controller in [self._driver, self._operator]:
                controller.button(6).onTrue(
                    cmd.runOnce(
                        lambda: container.vision.disable_the_vision(
                            not container.vision.disabled_vision
                        )
                    )
                )

        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
        ):
            for controller in [self._driver, self._operator]:
                controller.rightTrigger().whileTrue(
                    ShootCommand(
                        container.drivetrain,
                        container.shooter,
                        container.feeder,
                        container.spindex,
                        container.turret,
                    )
                )

        if RobotFeatures.HAS_FEEDER and RobotFeatures.HAS_SPINDEX:
            for controller in [self._driver, self._operator]:
                controller.b().whileTrue(
                    ThrowFeederCommand(container.feeder, container.spindex)
                )

        # Periodically warn about missing controllers during teleop
        Trigger(DriverStation.isTeleopEnabled).whileTrue(
            cmd.sequence(
                cmd.waitSeconds(5),
                cmd.runOnce(self._log_missing_connections),
            ).repeatedly()
        )

        # if RobotFeatures.TESTING:

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

    def update_state(self, sub=False):
        print(self._current_state)
        if sub:
            self._current_state -= 0.1
        else:
            self._current_state += 0.1

    def _log_missing_connections(self):
        if not DriverStation.isJoystickConnected(self._OPERATOR_PORT):
            wpilib.reportWarning(
                f"Xbox controller not connected on port {self._OPERATOR_PORT}"
            )
        if not DriverStation.isJoystickConnected(self._DRIVER_PORT):
            wpilib.reportWarning(
                f"Driver joystick not connected on port {self._DRIVER_PORT}"
            )

    def _joystick_to_rotation(self) -> Rotation2d | None:
        x = -self._operator.getLeftX()
        y = -self._operator.getLeftY()
        if math.hypot(x, y) < 0.1:
            return Rotation2d(0)
        return Rotation2d(y, x)
