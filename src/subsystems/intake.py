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
    supply_current_top: float
    supply_current_bottom: float
    stator_current_top: float
    stator_current_bottom: float
    motor_voltage_top: float
    motor_voltage_bottom: float
    device_temp_top: float
    device_temp_bottom: float
    arm_position: float
    head_position: float
    arm_supply_current: float
    head_supply_current: float
    arm_stator_current: float
    head_stator_current: float


@dataclass
class IntakeCommandTelemetry:
    target_position: str
    commanded_velocity_ft_per_sec: float
    converted_velocity_rps: float
    stop_requested: bool


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

        self.target_velocity = -1.0
        self._target_position_name = "NONE"
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0
        self._stop_requested = False

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
        self._target_position_name = position.name
        arm_rot, head_rot = self._POSITION_MAP[position]
        self.motor_arm.set_control(
            self._motion_magic_position_voltage.with_position(arm_rot)
        )
        self.motor_head.set_control(
            self._motion_magic_position_voltage.with_position(head_rot)
        )

    def set_velocity(self, velocity: float = 1):  # ft/sec
        self._commanded_velocity_ft_per_sec = float(velocity)
        self._stop_requested = False

        # Convert ft/sec to motor rotations/sec
        velocity *= (
            12 * IntakeConstants.ROLLER_GEARING
        ) / IntakeConstants.ROLLER_CIRCUMFERENCE

        self._converted_velocity_rps = float(velocity)

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
        self._stop_requested = True
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0

        self.motor_roller_top.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

    def periodic(self):
        self.log.publish(
            IntakeTelemetry(
                velocity_motor_bottom=float(self.motor_roller_bottom.get_velocity().value),
                velocity_motor_top=float(self.motor_roller_top.get_velocity().value),
                target_velocity=float(self.target_velocity),
                supply_current_top=float(self.motor_roller_top.get_supply_current().value),
                supply_current_bottom=float(self.motor_roller_bottom.get_supply_current().value),
                stator_current_top=float(self.motor_roller_top.get_stator_current().value),
                stator_current_bottom=float(self.motor_roller_bottom.get_stator_current().value),
                motor_voltage_top=float(self.motor_roller_top.get_motor_voltage().value),
                motor_voltage_bottom=float(self.motor_roller_bottom.get_motor_voltage().value),
                device_temp_top=float(self.motor_roller_top.get_device_temp().value),
                device_temp_bottom=float(self.motor_roller_bottom.get_device_temp().value),
                arm_position=float(self.motor_arm.get_position().value),
                head_position=float(self.motor_head.get_position().value),
                arm_supply_current=float(self.motor_arm.get_supply_current().value),
                head_supply_current=float(self.motor_head.get_supply_current().value),
                arm_stator_current=float(self.motor_arm.get_stator_current().value),
                head_stator_current=float(self.motor_head.get_stator_current().value),
            )
        )

        self.log.publish(
            IntakeCommandTelemetry(
                target_position=self._target_position_name,
                commanded_velocity_ft_per_sec=self._commanded_velocity_ft_per_sec,
                converted_velocity_rps=self._converted_velocity_rps,
                stop_requested=self._stop_requested,
            )
        )