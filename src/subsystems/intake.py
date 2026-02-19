from dataclasses import dataclass

from phoenix6 import controls
from phoenix6.hardware import TalonFX
import commands2

from utils import TalonConfig, Logger
from constants import MotorIDs, IntakeConstants
from commands2 import cmd
from enum import Enum


@dataclass
class IntakeTelemetry:
    velocity_motor_bottom: float
    velocity_motor_top: float
    target_velocity: float


class IntakePositions(Enum):
    HOME = 0
    STOWED = 1
    DEPLOYED = 2


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.log = Logger("Intake")
        INTAKE_CONFIG_ARM = TalonConfig(
            kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True
        )
        INTAKE_CONFIG_HEAD = TalonConfig(
            kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True
        )

        INTAKE_CONFIG_ROLLER_TOP = TalonConfig(
            kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True
        )
        INTAKE_CONFIG_ROLLER_BOTTOM = TalonConfig(
            kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True
        )

        self.motor_head: TalonFX = TalonFX(
            MotorIDs.motor_id_head,
        )
        self.motor_arm: TalonFX = TalonFX(
            MotorIDs.motor_id_arm,
        )

        self.motor_roller_top: TalonFX = TalonFX(
            MotorIDs.motor_id_roller_top,
        )

        self.motor_roller_bottom: TalonFX = TalonFX(
            MotorIDs.motor_id_roller_bottom,
        )

        INTAKE_CONFIG_ARM._apply_settings(self.motor_arm, inverted=False)
        INTAKE_CONFIG_HEAD._apply_settings(self.motor_head, inverted=False)
        INTAKE_CONFIG_ROLLER_TOP._apply_settings(self.motor_roller_top, inverted=True)
        INTAKE_CONFIG_ROLLER_BOTTOM._apply_settings(
            self.motor_roller_bottom, inverted=False
        )

        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(
            0, enable_foc=MotorIDs.foc_active
        )
        self._motion_magic_position_voltage = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.target_velocity = -1
        self.set_velocity_command = cmd.runOnce(self.set_velocity)
        self.stop_command = cmd.runOnce(self.stop)

        self.goto_position_cmmand = {
            pos: cmd.runOnce(lambda: self.go_to_position(pos))
            for pos in IntakePositions
        }

    def init(self):
        pass

    _POSITION_MAP = {
        IntakePositions.HOME: (
            IntakeConstants.ARM_HOME_ROTATIONS,
            IntakeConstants.HEAD_HOME_ROTATIONS,
        ),
        IntakePositions.STOWED: (
            IntakeConstants.ARM_STOWED_ROTATIONS,
            IntakeConstants.HEAD_STOWED_ROTATIONS,
        ),
        IntakePositions.DEPLOYED: (
            IntakeConstants.ARM_DEPLOYED_ROTATIONS,
            IntakeConstants.HEAD_DEPLOYED_ROTATIONS,
        ),
    }

    def go_to_position(self, position: IntakePositions):
        arm_rot, head_rot = self._POSITION_MAP[position]
        self.motor_arm.set_control(
            self._motion_magic_position_voltage.with_position(arm_rot)
        )
        self.motor_head.set_control(
            self._motion_magic_position_voltage.with_position(head_rot)
        )

    def set_velocity(self, velocity: float = 1):  # ft/sec
        # Convert ft/sec to motor rotations/sec
        velocity *= (
            12 * IntakeConstants.ROLLER_GEARING
        ) / IntakeConstants.ROLLER_CIRCUMFERENCE

        # print("AAAx")
        self.motor_roller_top.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

        self.target_velocity = velocity

    def stop(self):
        self.motor_roller_top.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

    def periodic(self):
        self.log.publish(
            IntakeTelemetry(
                velocity_motor_bottom=float(
                    self.motor_roller_bottom.get_velocity().value
                ),
                velocity_motor_top=float(self.motor_roller_top.get_velocity().value),
                target_velocity=float(self.target_velocity),
            )
        )
