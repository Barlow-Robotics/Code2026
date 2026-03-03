from dataclasses import dataclass
import dataclasses

from phoenix6 import controls
from phoenix6.hardware import TalonFX
import commands2
from pykit.logger import Logger as PyKitLogger
import wpilib

from utils import TalonConfig, generateSysIdProfile
from constants import MotorIDs, IntakeConstants
from commands2 import cmd
from enum import Enum

from commands2.sysid import SysIdRoutine

from wpilib import (
    Mechanism2d,
    Color8Bit,
    RobotBase,
)


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

        self.mechanism = Mechanism2d(4, 3)  # wider canvas to show horizontal reach

        # Position root toward right-center of robot, low mount point
        root = self.mechanism.getRoot("IntakeRoot", 2.5, 0.3)

        # Arm starts angled upward, adjust starting angle to match stowed position
        self.arm_ligament = root.appendLigament(
            "Arm",
            1.2,
            75,
            8,
            Color8Bit(0, 200, 0),  # green like your CAD
        )

        # Head/roller assembly at end of arm, perpendicular
        self.head_ligament = self.arm_ligament.appendLigament(
            "Head",
            0.4,
            -90,
            10,
            Color8Bit(0, 150, 0),  # darker green, wider = roller
        )
        wpilib.SmartDashboard.putData("Intake/Mechanism2d", self.mechanism)

        INTAKE_CONFIG_ARM = TalonConfig(
            kP=0.11,
            kI=0,
            kD=0,
            kF=0,
            kA=0,
            brake_mode=True,
        )
        INTAKE_CONFIG_HEAD = TalonConfig(
            kP=0.11,
            kI=0,
            kD=0,
            kF=0,
            kA=0,
            brake_mode=True,
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

        self._motion_magic_velocity_voltage_roller_top = (
            controls.MotionMagicVelocityVoltage(0, enable_foc=MotorIDs.foc_active)
        )

        self._motion_magic_velocity_voltage_roller_bottom = (
            controls.MotionMagicVelocityVoltage(0, enable_foc=MotorIDs.foc_active)
        )

        self._motion_magic_position_voltage_arm = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )
        self._motion_magic_position_voltage_head = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.target_velocity = -1.0
        self._target_position_name = "NONE"
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0
        self._stop_requested = False

        self.set_velocity_command = cmd.runOnce(self.set_velocity)
        self.stop_command = cmd.runOnce(self.stop)

        # self.goto_position_command = {
        #     pos: cmd.runOnce(lambda p=pos: self.go_to_position(p))
        #     for pos in IntakePositions
        # }
        self.goto_position_command_factory = lambda pos: cmd.runOnce(lambda: self.go_to_position(pos))

        self._POSITION_MAP = {
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

        self.sys_id_routine_arm = generateSysIdProfile(self, self.motor_arm, name="Arm")
        self.sys_id_routine_head = generateSysIdProfile(
            self, self.motor_head, name="Head"
        )
        self.sys_id_routine_roller_top = generateSysIdProfile(
            self, self.motor_roller_top, name="Roller_Top"
        )
        self.sys_id_routine_roller_bottom = generateSysIdProfile(
            self, self.motor_roller_bottom, name="Roller_Bottom"
        )

    def go_to_position(self, position: IntakePositions):
        self._target_position_name = position.name
        arm_rot, head_rot = self._POSITION_MAP[position]
        print(arm_rot, head_rot)

        self.motor_arm.set_control(
            self._motion_magic_position_voltage_arm.with_position(arm_rot)
        )

        self.motor_head.set_control(
            self._motion_magic_position_voltage_head.with_position(head_rot)
        )
        if position == IntakePositions.DEPLOYED:
            print("Deploying intake, setting rollers to velocity")
            self.set_velocity(IntakeConstants.INTAKE_VELOCITY_CONSTANT)
        else:
            print("Stowing intake, stopping rollers")
            self.stop()

    def set_velocity(self, velocity: float = IntakeConstants.INTAKE_VELOCITY_CONSTANT):
        self._commanded_velocity_ft_per_sec = float(velocity)
        self._stop_requested = False

        # Convert ft/sec to motor rotations/sec
        velocity *= (
            12 * IntakeConstants.ROLLER_GEARING
        ) / IntakeConstants.ROLLER_CIRCUMFERENCE

        self._converted_velocity_rps = float(velocity)

        self.motor_roller_top.set_control(
            self._motion_magic_velocity_voltage_roller_top.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage_roller_bottom.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

        self.target_velocity = velocity

    def stop(self):
        self._stop_requested = True
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0
        self.target_velocity = 0.0

        self.motor_roller_top.set_control(
            self._motion_magic_velocity_voltage_roller_top.with_velocity(
                0
            ).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage_roller_bottom.with_velocity(
                0
            ).with_acceleration(0.1)
        )

    def _log_dataclass(self, prefix: str, data: object):
        """Recursively log a dataclass to PyKitLogger using prefix."""
        for field in dataclasses.fields(data):
            value = getattr(data, field.name)

            if value is None:
                continue

            key = f"{prefix}/{field.name}"

            if dataclasses.is_dataclass(value):
                self._log_dataclass(key, value)
            else:
                PyKitLogger.recordOutput(key, value)

    def periodic(self):
        intake_prefix = "Intake/Telemetry"
        command_prefix = "Intake/Command"
        if not RobotBase.isReal():
            arm_degrees = self.motor_arm.get_position().value * 360
            head_degrees = self.motor_head.get_position().value * 360

            self.arm_ligament.setAngle(
                75 + arm_degrees
            )  # offset by your starting angle
            self.head_ligament.setAngle(-90 + head_degrees)
            # SmartDashboard.putData("Intake/Mechanism2d", self.mechanism)

        self._log_dataclass(
            intake_prefix,
            IntakeTelemetry(
                velocity_motor_bottom=float(
                    self.motor_roller_bottom.get_velocity().value
                ),
                velocity_motor_top=float(self.motor_roller_top.get_velocity().value),
                target_velocity=float(self.target_velocity),
                supply_current_top=float(
                    self.motor_roller_top.get_supply_current().value
                ),
                supply_current_bottom=float(
                    self.motor_roller_bottom.get_supply_current().value
                ),
                stator_current_top=float(
                    self.motor_roller_top.get_stator_current().value
                ),
                stator_current_bottom=float(
                    self.motor_roller_bottom.get_stator_current().value
                ),
                motor_voltage_top=float(
                    self.motor_roller_top.get_motor_voltage().value
                ),
                motor_voltage_bottom=float(
                    self.motor_roller_bottom.get_motor_voltage().value
                ),
                device_temp_top=float(self.motor_roller_top.get_device_temp().value),
                device_temp_bottom=float(
                    self.motor_roller_bottom.get_device_temp().value
                ),
                arm_position=float(self.motor_arm.get_position().value),
                head_position=float(self.motor_head.get_position().value),
                arm_supply_current=float(self.motor_arm.get_supply_current().value),
                head_supply_current=float(self.motor_head.get_supply_current().value),
                arm_stator_current=float(self.motor_arm.get_stator_current().value),
                head_stator_current=float(self.motor_head.get_stator_current().value),
            ),
        )

        self._log_dataclass(
            command_prefix,
            IntakeCommandTelemetry(
                target_position=self._target_position_name,
                commanded_velocity_ft_per_sec=self._commanded_velocity_ft_per_sec,
                converted_velocity_rps=self._converted_velocity_rps,
                stop_requested=self._stop_requested,
            ),
        )

    def sysIdQuasistaticArm(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_arm.quasistatic(direction)

    def sysIdDynamicArm(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_arm.dynamic(direction)

    def sysIdQuasistaticHead(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_head.quasistatic(direction)

    def sysIdDynamicHead(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_head.dynamic(direction)

    def sysIdQuasistaticRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller_top.quasistatic(direction)

    def sysIdDynamicRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller_top.dynamic(direction)

    def sysIdQuasistaticRollerBottom(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller_bottom.quasistatic(direction)

    def sysIdDynamicRollerBottom(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller_bottom.dynamic(direction)
