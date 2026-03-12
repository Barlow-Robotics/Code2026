from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

from commands2 import cmd
import commands2
from commands2.button import CommandXboxController, CommandJoystick, Trigger

import wpilib
from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from commands import IntakePositionCommand, ShootCommand
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
                        -self._driver.getRawAxis(2 if type(self._driver) == CommandJoystick else 4)
                        * DriveConstants.MAX_ANGULAR_VELOCITY
                        * self._current_state
                    )
                )
            )
        )
        
        self._driver.rightBumper().onTrue(
            cmd.runOnce(lambda: self.update_state(sub=False) if round(self._current_state, 2) < 1.0 else print("HI2"))
        )
        
        
        self._driver.leftBumper().onTrue(
            cmd.runOnce(lambda: self.update_state(sub=True) if round(self._current_state, 2) > 0.5 else print("hi"))
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
            self._driver.leftStick().onTrue( # NEED TO FIX
                IntakePositionCommand(
                    drive_sub=container.drivetrain, intake_sub=container.intake, position=IntakePositions.DEPLOYED,
                )
            )
            
            self._driver.b().onTrue(
                IntakePositionCommand(
                    drive_sub=container.drivetrain, intake_sub=container.intake, position=IntakePositions.DEPLOYED, move=False
                )
            )
            self._driver.a().onTrue(
                IntakePositionCommand(
                    drive_sub=container.drivetrain, intake_sub=container.intake, position=IntakePositions.HOME, move=False
                )
            )
        
            
        if RobotFeatures.HAS_VISION:
            # NEED TO ADD AUTO-ALIGN
            self._driver.x().onTrue(container.vision.auto_align_command)
            # self._driver.button(6).onTrue(
            #     cmd.run(container.vision.position_to_pose_align)
            #     .until(lambda: container.vision.is_aligned(tolerance_degrees=0.7))
            #     .withTimeout(1.0)
            # )
        if (
            RobotFeatures.HAS_SHOOTER
            and RobotFeatures.HAS_FEEDER
            and RobotFeatures.HAS_SPINDEX
        ):
            self._driver.rightTrigger().whileTrue(ShootCommand(container.shooter, container.feeder, container.spindex))
        if RobotFeatures.HAS_SPINDEX:
            self._operator.leftTrigger().onTrue(
                cmd.runOnce(
                    lambda: container.feeder.move(1, False),
                )
            )
            self._operator.leftBumper().onTrue(
                cmd.runOnce(
                    lambda: container.feeder.move(1, True),
                )
            )
            self._operator.rightBumper().onTrue(
                cmd.runOnce(
                    lambda: container.feeder.stop(),
                )
            )

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
