from dataclasses import dataclass
import dataclasses

import commands2
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from phoenix6.hardware import TalonFX
import wpilib
from wpilib import RobotBase, Mechanism2d, Color8Bit, SmartDashboard
from pykit.logger import Logger as PyKitLogger

from constants.robot_constants import RobotFeatures
from utils.profiler import LoopTimer
from constants import IntakeConstants, MotorIDs
from utils import TalonConfigFX, generateSysIdProfile, IntakePositions


@dataclass
class IntakeCommandTelemetry:
    target_position: str

    commanded_velocity_ft_per_sec: float
    converted_velocity_rps: float
    stop_requested: bool


@dataclass
class IntakeTelemetry:
    arm_position: float
    # arm_position_follower: float
    arm_target_position: float
    # follower_voltage: float
    # leader_voltage: float
    # # arm_supply_current: float
    # arm_stator_current: float
    roller_velocity: float
    # roller_supply_current: float
    # roller_stator_current: float
    # roller_motor_voltage: float
    # roller_device_temp: float
    target_velocity: float


class Intake(commands2.Subsystem):
    def __init__(self, init2=False):
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
        if RobotFeatures.LOGGING_ROBOT:
            self.mechanism = Mechanism2d(4, 3)
            root = self.mechanism.getRoot("IntakeRoot", 2.5, 0.3)
            self.arm_ligament = root.appendLigament(
                "Arm", 1.2, 75, 8, Color8Bit(0, 200, 0)
            )
            self.head_ligament = self.arm_ligament.appendLigament(
                "Head", 0.4, -90, 10, Color8Bit(0, 150, 0)
            )
            wpilib.SmartDashboard.putData("Intake/Mechanism2d", self.mechanism)

        self.motor_roller: TalonFX = TalonFX(MotorIDs.motor_id_roller, "*")
        self.motor_arm_leader: TalonFX = TalonFX(MotorIDs.motor_id_arm_leader_left, "*")
        # self.motor_arm_follower: TalonFX = TalonFX(MotorIDs.motor_id_arm_follower_right, "*")
        if not init2:
            SmartDashboard.putNumber("kP_Roller", 0.0)
            SmartDashboard.putNumber("kI_Roller", 0.0)
            SmartDashboard.putNumber("kD_Roller", 0.0)
            SmartDashboard.putNumber("kV_Roller", 0.01)
            SmartDashboard.putNumber("kG_Roller", 0.00)
            SmartDashboard.putNumber("kS_Roller", 0.00)

            SmartDashboard.putNumber("kP_IntakeArm", 0.0)
            SmartDashboard.putNumber("kI_IntakeArm", 0.0)
            SmartDashboard.putNumber("kD_IntakeArm", 0.0)
            SmartDashboard.putNumber("kV_IntakeArm", 0.00)
            SmartDashboard.putNumber("kG_IntakeArm", 0.00)
            SmartDashboard.putNumber("kS_IntakeArm", 0.00)
            SmartDashboard.putNumber("arm_rot", 4.0)
            SmartDashboard.putNumber("roller_speed", 4.0)

        INTAKE_ROLLER = TalonConfigFX(
            kP=SmartDashboard.getNumber("kP_Roller", 0.0),
            kI=SmartDashboard.getNumber("kI_Roller", 0.0),
            kD=SmartDashboard.getNumber("kD_Roller", 0.0),
            kV=SmartDashboard.getNumber("kV_Roller", 0.00),
            kG=SmartDashboard.getNumber("kG_Roller", 0.00),
            kS=SmartDashboard.getNumber("kS_Roller", 0.00),
            brake_mode=False,
            gear_ratio=IntakeConstants.ROLLER_GEARING,
        )

        INTAKE_CONFIG_ARM_RIGHT = TalonConfigFX(
            kP=SmartDashboard.getNumber("kP_IntakeArm", 0.0),
            kI=SmartDashboard.getNumber("kI_IntakeArm", 0.0),
            kD=SmartDashboard.getNumber("kD_IntakeArm", 0.0),
            kV=SmartDashboard.getNumber("kV_IntakeArm", 0.00),
            kG=SmartDashboard.getNumber("kG_IntakeArm", 0.00),
            kS=SmartDashboard.getNumber("kS_IntakeArm", 0.00),
            brake_mode=True,
            gear_ratio=IntakeConstants.ARM_GEARING,
        )

        INTAKE_ROLLER._apply_settings(self.motor_roller, inverted=False)
        INTAKE_CONFIG_ARM_RIGHT._apply_settings(self.motor_arm_leader, inverted=False)

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
            IntakePositions.REVERSE: IntakeConstants.ARM_DEPLOYED_ROTATIONS,
            IntakePositions.DEPLOYED: IntakeConstants.ARM_DEPLOYED_ROTATIONS,
        }

        self.go_to_velocity_command_factory = lambda velocity: cmd.runOnce(
            lambda: self.set_velocity(velocity)
        )

    def reinitMotors(self):
        INTAKE_ROLLER = TalonConfigFX(
            kP=SmartDashboard.getNumber("kP_Roller", 0.0),
            kI=SmartDashboard.getNumber("kI_Roller", 0.0),
            kD=SmartDashboard.getNumber("kD_Roller", 0.0),
            kV=SmartDashboard.getNumber("kV_Roller", 0.00),
            kG=SmartDashboard.getNumber("kG_Roller", 0.00),
            kS=SmartDashboard.getNumber("kS_Roller", 0.00),
            brake_mode=False,
            gear_ratio=IntakeConstants.ROLLER_GEARING,
        )

        INTAKE_CONFIG_ARM_RIGHT = TalonConfigFX(
            kP=SmartDashboard.getNumber("kP_IntakeArm", 0.0),
            kI=SmartDashboard.getNumber("kI_IntakeArm", 0.0),
            kD=SmartDashboard.getNumber("kD_IntakeArm", 0.0),
            kV=SmartDashboard.getNumber("kV_IntakeArm", 0.00),
            kG=SmartDashboard.getNumber("kG_IntakeArm", 0.00),
            kS=SmartDashboard.getNumber("kS_IntakeArm", 0.00),
            brake_mode=True,
            gear_ratio=IntakeConstants.ARM_GEARING,
        )

        INTAKE_ROLLER._apply_settings(self.motor_roller, inverted=False)
        INTAKE_CONFIG_ARM_RIGHT._apply_settings(self.motor_arm_leader, inverted=False)

    def set_velocity(self, current_velocity: float):
        if current_velocity < 1:
            current_velocity = 1
        commanded_velocity = SmartDashboard.getNumber("roller_speed", 4.0)
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

    def reverse_velocity(self, current_velocity: float):
        if current_velocity < 1:
            current_velocity = 1
        commanded_velocity = SmartDashboard.getNumber("roller_speed", 4.0)
        self._commanded_velocity_ft_per_sec = float(commanded_velocity)
        self._stop_requested = False

        commanded_velocity /= IntakeConstants.ROLLER_CIRCUMFERENCE_M
        self._converted_velocity_rps = float(commanded_velocity)

        self.motor_roller.set_control(
            self._motion_magic_velocity_voltage_roller.with_velocity(
                -commanded_velocity
            ).with_acceleration(0.1)
        )
        self.target_velocity = -commanded_velocity

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
        if position == IntakePositions.HOME:
            self.motor_arm_leader.set_control(
                self._motion_magic_position_voltage_arm_leader.with_position(0)
            )
        else:
            self.motor_arm_leader.set_control(
                self._motion_magic_position_voltage_arm_leader.with_position(
                    SmartDashboard.getNumber("arm_rot", 4.0)
                )
            )

        # self.motor_arm_follower.set_control(
        #     self._motion_magic_position_voltage_arm_follower.with_position(arm_rot)
        # )

        # if position == IntakePositions.DEPLOYED:
        #     print("SET VELOCITY")
        #     self.set_velocity(current_velocity)
        # else:
        #     self.stop()
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

        if not RobotBase.isReal() and RobotFeatures.LOGGING_ROBOT:
            arm_degrees = self.motor_arm_leader.get_position().value * 360
            self.arm_ligament.setAngle(75 + arm_degrees)
        if RobotFeatures.LOGGING_INTAKE:
            PyKitLogger.recordOutput(
                "Intake/Actual_Arm_Position",
                float(self.motor_arm_leader.get_position().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Target_Arm_Position", float(self.target_rot)
            )
            PyKitLogger.recordOutput(
                "Intake/Actual_Roller_Velocity",
                float(self.motor_roller.get_velocity().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Target_Roller_Velocity", float(self.target_velocity)
            )
            PyKitLogger.recordOutput(
                "Intake/Actual_Arm_Position",
                float(self.motor_arm_leader.get_position().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Target_Arm_State", self._target_position_name
            )
            PyKitLogger.recordOutput(
                "Intake/Arm_Volt",
                float(self.motor_arm_leader.get_motor_voltage().value),
            )

        self._loop_timer.stop()

    def reset_zero(self):
        self.motor_arm_leader.set_position(0)
        # self.motor_arm_follower.set_position(0)

    def sysIdQuasistaticRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.quasistatic(direction)

    def sysIdDynamicRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.dynamic(direction)
