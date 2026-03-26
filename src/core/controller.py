from __future__ import annotations

from enum import Enum
import math
from typing import TYPE_CHECKING

from commands2 import cmd
from commands2.button import CommandXboxController, CommandJoystick, Trigger

import wpilib
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from commands import IntakePositionCommand, ShootCommand, ReverseCommand
from commands.throw_feeder_command import ThrowFeederCommand
from constants import DriveConstants, RobotFeatures
from subsystems.intake import IntakePositions
from pykit.logger import Logger as PyKitLogger

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
        self._test_controller = CommandXboxController(3)

        self._current_state = StartingState.NORMAL.value
        container.drivetrain.setDefaultCommand(
            container.drivetrain.apply_request(
                lambda: (
                    (
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
                    if not container.drivetrain.changeAngTelop
                    else (
                        container.drivetrain.facing_angle.with_velocity_x(
                            -self._driver.getRawAxis(1)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                            * self._current_state
                        )
                        .with_velocity_y(
                            -self._driver.getRawAxis(0)
                            * DriveConstants.MAX_TRANSLATIONAL_VELOCITY
                            * self._current_state
                        )
                        .with_target_direction(
                            container.drivetrain.target_field_heading
                        )
                    )
                )
            )
        )   

        for controller in [self._driver]:
            controller.rightBumper().onTrue(
                cmd.runOnce(
                    lambda: (
                        self.update_driver(sub=False)
                    )
                )
            )

            controller.leftBumper().onTrue(
                cmd.runOnce(
                    lambda: (
                        self.update_driver(sub=True)
                    )
                )
            )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            container.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        if RobotFeatures.TESTING:
            self._test_controller.leftBumper().onTrue(
                cmd.runOnce(lambda: container.intake.set_velocity(0))
            ).onFalse(cmd.runOnce(lambda: container.intake.stop()))

            self._test_controller.rightTrigger().onTrue(
                cmd.runOnce(
                    lambda: container.intake.go_to_position(IntakePositions.DEPLOYED)
                )
            )
            self._test_controller.leftTrigger().onTrue(
                cmd.runOnce(
                    lambda: container.intake.go_to_position(IntakePositions.HOME)
                )
            )


            # self._test_controller.y().onTrue(
            #     cmd.runOnce(lambda: container.feeder.move(9))
            # ).onFalse(cmd.runOnce(lambda: container.feeder.stop()))

            # self._test_controller.a().onTrue(
            #     cmd.runOnce(lambda: container.spindex.move(9))
            # ).onFalse(cmd.runOnce(lambda: container.spindex.stop()))

            # self._test_controller.rightTrigger().onTrue(
            #     cmd.runOnce(lambda: container.shooter.set_velocity(11))
            # ).onFalse(cmd.runOnce(lambda: container.shooter.set_velocity(0)))

            # self._test_controller.leftTrigger().onTrue(
            #     cmd.runOnce(
            #         lambda: container.turret.set_angle_hood(
            #             SmartDashboard.getNumber("hood_angle", 10)
            #         )
            #     )
            # ).onFalse(cmd.runOnce(lambda: container.turret.set_angle_hood(0)))

            # self._test_controller.x().whileTrue(
            #     ShootCommand(
            #         container.drivetrain,
            #         container.shooter,
            #         container.feeder,
            #         container.spindex,
            #         container.turret
            #     )
            # )

            # self._test_controller.b().onTrue(
            #     cmd.runOnce(lambda: container.intake.reset_zero())
            # )
            # self._test_controller.button(1).whileTrue(
            #     cmd.run(
            #         lambda: container.turret.enableAutoAim(True, vision_on=not container.vision.disabled_vision)
            #     )
            # ).onFalse(
            #     cmd.runOnce(
            #         lambda: container.turret.enableAutoAim(False, vision_on=not container.vision.disabled_vision)
            #     )
            # )
            # self._test_controller.button(2).onTrue(
            #     cmd.runOnce(
            #         lambda: container.vision.disable_the_vision(
            #             not container.vision.disabled_vision
            #         )
            #     )
            # )
        # Subsystem button bindings
        if RobotFeatures.HAS_INTAKE:
            for controller in [self._operator]:
                # Y → Deploy intake
                controller.y().onTrue(
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
                # A → Retract intake
                controller.rightTrigger().whileTrue(
                    cmd.runOnce(lambda: container.intake.set_velocity(1))
                ).whileFalse(cmd.runOnce(lambda: container.intake.stop()))

                controller.leftTrigger().whileTrue(
                    cmd.runOnce(lambda: container.intake.set_velocity(-1))
                ).whileFalse(
                    cmd.runOnce(lambda: container.intake.stop())
                )
            


        
        for controller in [self._driver]:
            controller.button(8).onTrue(cmd.runOnce(container.drivetrain.reset_gyro))

        if RobotFeatures.HAS_VISION:
            for controller in [self._driver]:
                controller.button(7).onTrue(
                    cmd.runOnce(
                        lambda: container.vision.disable_the_vision(
                            not container.vision.disabled_vision
                        )
                    )
                )

                controller.leftTrigger().whileTrue(
                    cmd.run(
                        lambda: container.turret.enableAutoAim(True, vision_on=not container.vision.disabled_vision)
                    )
                ).onFalse(
                    cmd.runOnce(
                        lambda: container.turret.enableAutoAim(False, vision_on=not container.vision.disabled_vision)
                    )
                )


        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
        ):
            for controller in [self._driver]:
                # X → Fire / Shoot

                controller.rightTrigger().whileTrue(
                    ShootCommand(
                        container.drivetrain,
                        container.shooter,
                        container.feeder,
                        container.spindex,
                        container.turret
                    )
                )
                controller.b().whileTrue(
                    ThrowFeederCommand(container.feeder, container.spindex)
                )

        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
            and RobotFeatures.HAS_INTAKE
        ):
            for controller in [self._operator]:
                controller.leftBumper().whileTrue( # reverse intake
                    ReverseCommand(container.drivetrain, container.shooter, container.feeder, container.spindex, container.turret, container.intake)
                )
        if RobotFeatures.HAS_TURRET:
            for controller in [self._driver, self._operator]:
                # Left trigger → Aim
                pass
                # controller.povDown().onTrue(
                #     cmd.runOnce(
                #         lambda: (
                #             container.turret.set_angle_hood(
                #                 container.turret.hood_cancoder.get_position() - 10
                #             )
                #             if container.vision.disabled_vision
                #             else container.turret.set_angle_hood(
                #                 container.turret.hood_cancoder.get_position()
                #             )
                #         )
                #     )
                # )
                # controller.povUp().onTrue(
                #     cmd.runOnce(
                #         lambda: (
                #             container.turret.set_angle_hood(
                #                 container.turret.hood_cancoder.get_position() + 10
                #             )
                #             if container.vision.disabled_vision
                #             else container.turret.set_angle_hood(
                #                 container.turret.hood_cancoder.get_position()
                #             )
                #         )
                #     )
                # )
        # Periodically warn about missing controllers during teleop
        # Trigger(DriverStation.isTeleopEnabled).whileTrue(
        #     cmd.sequence(
        #         cmd.waitSeconds(5),
        #         cmd.runOnce(self._log_missing_connections),
        #     ).repeatedly()
        # )

    def update_state(self, sub=False):        
        if sub:
            self._current_state -= 0.5
        else:
            self._current_state += 0.5


    def update_driver(self, sub: bool):
        if sub:
            if round(self._current_state, 2) > 0.5:
                self.update_state(sub=sub)
        elif not sub:
            if round(self._current_state, 2) < 1.0:
                self.update_state(sub=sub)

    # def _log_missing_connections(self):
    #     if not DriverStation.isJoystickConnected(self._OPERATOR_PORT):
    #         wpilib.reportWarning(
    #             f"Xbox controller not connected on port {self._OPERATOR_PORT}"
    #         )
    #     if not DriverStation.isJoystickConnected(self._DRIVER_PORT):
    #         wpilib.reportWarning(
    #             f"Driver joystick not connected on port {self._DRIVER_PORT}"
    #         )

    def _joystick_to_rotation(self) -> Rotation2d | None:
        x = -self._operator.getLeftX()
        y = -self._operator.getLeftY()
        if math.hypot(x, y) < 0.1:
            return Rotation2d(0)
        return Rotation2d(y, x)
