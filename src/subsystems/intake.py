from dataclasses import dataclass
import dataclasses

import commands2
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from phoenix6.hardware import TalonFX
import wpilib
from wpilib import RobotBase, Mechanism2d, Color8Bit
from pykit.logger import Logger as PyKitLogger

from utils.profiler import LoopTimer
from constants import IntakeConstants, MotorIDs
from utils import TalonConfigFX, generateSysIdProfile, IntakePositions


@dataclass
class IntakeCommandTelemetry:
    target_position: float
    commanded_velocity_ft_per_sec: float
    converted_velocity_rps: float
    stop_requested: bool


@dataclass
class IntakeTelemetry:
    arm_position: float
    arm_target_position: float
    # arm_supply_current: float
    # arm_stator_current: float
    roller_velocity: float
    # roller_supply_current: float
    # roller_stator_current: float
    # roller_motor_voltage: float
    # roller_device_temp: float
    target_velocity: float


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Intake")
        self.target_velocity = 0.0
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0
        self._stop_requested = False
        self._target_position_name = "NONE"
        self._target_position = IntakePositions.HOME
        self.target_rot: float = -1.0

        # Mechanism2d
        self.mechanism = Mechanism2d(4, 3)
        root = self.mechanism.getRoot("IntakeRoot", 2.5, 0.3)
        self.arm_ligament = root.appendLigament("Arm", 1.2, 75, 8, Color8Bit(0, 200, 0))
        self.head_ligament = self.arm_ligament.appendLigament(
            "Head", 0.4, -90, 10, Color8Bit(0, 150, 0)
        )
        wpilib.SmartDashboard.putData("Intake/Mechanism2d", self.mechanism)

        self.motor_roller: TalonFX = TalonFX(MotorIDs.motor_id_roller, "*")
        self.motor_arm_leader: TalonFX = TalonFX(MotorIDs.motor_id_arm_leader, "*")
        self.motor_arm_follower: TalonFX = TalonFX(MotorIDs.motor_id_arm_follower, "*")

        INTAKE_ROLLER = TalonConfigFX(
            kP=0.11,
            kI=0,
            kD=0,
            kV=0,
            brake_mode=False,
            gear_ratio=IntakeConstants.ROLLER_GEARING,
        )
        INTAKE_CONFIG_ARM = TalonConfigFX(
            kP=0.11,
            kI=0,
            kD=0,
            kV=0,
            brake_mode=True,
            gear_ratio=IntakeConstants.ARM_GEARING,
        )

        INTAKE_ROLLER._apply_settings(self.motor_roller, inverted=True)
        INTAKE_CONFIG_ARM._apply_settings(self.motor_arm_leader, inverted=False)
        INTAKE_CONFIG_ARM._apply_settings(self.motor_arm_follower, inverted=True)

        self._motion_magic_velocity_voltage_roller = (
            controls.MotionMagicVelocityVoltage(0, enable_foc=MotorIDs.foc_active)
        )
        self._motion_magic_position_voltage_arm_leader = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )
        self._motion_magic_position_voltage_arm_follower = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.sys_id_routine_roller = generateSysIdProfile(
            self, self.motor_roller, name="Roller"
        )

        self._POSITION_MAP = {
            IntakePositions.HOME: IntakeConstants.ARM_HOME_ROTATIONS,
            IntakePositions.STOWED: IntakeConstants.ARM_STOWED_ROTATIONS,
            IntakePositions.DEPLOYED: IntakeConstants.ARM_DEPLOYED_ROTATIONS,
        }

        self.go_to_velocity_command_factory = lambda velocity: cmd.runOnce(
            lambda: self.set_velocity(velocity)
        )

    def set_velocity(self, current_velocity: float):
        if current_velocity < 1:
            current_velocity = 1
        commanded_velocity = current_velocity * IntakeConstants.INTAKE_VELOCITY_CONSTANT
        self._commanded_velocity_ft_per_sec = float(commanded_velocity)
        self._stop_requested = False

        commanded_velocity /= IntakeConstants.ROLLER_CIRCUMFERENCE_M
        self._converted_velocity_rps = float(commanded_velocity)

        self.motor_roller.set_control(
            self._motion_magic_velocity_voltage_roller.with_velocity(
                commanded_velocity
            ).with_acceleration(0.1)
        )
        self.target_velocity = commanded_velocity

    def stop(self):
        self._stop_requested = True
        self._commanded_velocity_ft_per_sec = 0.0
        self._converted_velocity_rps = 0.0
        self.target_velocity = 0.0

        self.motor_roller.set_control(
            self._motion_magic_velocity_voltage_roller.with_velocity(
                0
            ).with_acceleration(0.1)
        )

    def get_target_position(self):
        return self._target_position

    def go_to_position(self, position: IntakePositions, current_velocity: float = 0.0):
        self._target_position_name = position.name
        self._target_position = position
        arm_rot = self._POSITION_MAP[position]

        self.motor_arm_leader.set_control(
            self._motion_magic_position_voltage_arm_leader.with_position(arm_rot)
        )
        self.motor_arm_follower.set_control(
            self._motion_magic_position_voltage_arm_follower.with_position(arm_rot)
        )

        if position == IntakePositions.DEPLOYED:
            self.set_velocity(current_velocity)
        else:
            self.stop()
        self.target_rot: float = arm_rot

    def _log_dataclass(self, prefix: str, data: object):
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
        self._loop_timer.start()

        if not RobotBase.isReal():
            arm_degrees = self.motor_arm_leader.get_position().value * 360
            self.arm_ligament.setAngle(75 + arm_degrees)

        self._log_dataclass(
            "Intake/Telemetry",
            IntakeTelemetry(
                arm_position=float(self.motor_arm_leader.get_position().value),
                arm_target_position=float(self.target_rot),
                # arm_supply_current=float(self.motor_arm.get_supply_current().value),
                # arm_stator_current=float(self.motor_arm.get_stator_current().value),
                roller_velocity=float(self.motor_roller.get_velocity().value),
                # roller_supply_current=float(self.motor_roller.get_supply_current().value),
                # roller_stator_current=float(self.motor_roller.get_stator_current().value),
                # roller_motor_voltage=float(self.motor_roller.get_motor_voltage().value),
                # roller_device_temp=float(self.motor_roller.get_device_temp().value),
                target_velocity=float(self.target_velocity),
            ),
        )

        self._log_dataclass(
            "Intake/Command",
            IntakeCommandTelemetry(
                target_position=self._target_position_name,
                commanded_velocity_ft_per_sec=self._commanded_velocity_ft_per_sec,
                converted_velocity_rps=self._converted_velocity_rps,
                stop_requested=self._stop_requested,
            ),
        )

        self._loop_timer.stop()

    def sysIdQuasistaticRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.quasistatic(direction)

    def sysIdDynamicRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.dynamic(direction)
