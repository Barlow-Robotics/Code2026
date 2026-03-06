from enum import Enum
import commands2
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from phoenix6 import controls

from constants import IntakeConstants
from utils import TalonConfig, generateSysIdProfile
from constants import MotorIDs
from phoenix6.hardware import TalonFX


class IntakePositions(Enum):
    HOME = 0
    STOWED = 1
    DEPLOYED = 2


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.motor_roller: TalonFX = TalonFX(
            MotorIDs.motor_id_roller_top,
        )
        self.motor_arm: TalonFX = TalonFX(
            MotorIDs.motor_id_arm,
        )

        INTAKE_ROLLER = TalonConfig(
            kP=0.11,
            kI=0,
            kD=0,
            kF=0,
            kA=0,
            brake_mode=True,
            gear_ratio=IntakeConstants.ROLLER_GEARING,
        )
        INTAKE_CONFIG_ARM = TalonConfig(
            kP=0.11,
            kI=0,
            kD=0,
            kF=0,
            kA=0,
            brake_mode=True,
            gear_ratio=IntakeConstants.ARM_GEARING,
        )

        INTAKE_ROLLER._apply_settings(self.motor_roller, inverted=True)
        INTAKE_CONFIG_ARM._apply_settings(self.motor_arm, inverted=False)

        self._motion_magic_velocity_voltage_roller = (
            controls.MotionMagicVelocityVoltage(0, enable_foc=MotorIDs.foc_active)
        )

        self._motion_magic_position_voltage_arm = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.sys_id_routine_roller = generateSysIdProfile(
            self, self.motor_roller, name="Roller"
        )
        self._POSITION_MAP = {
            IntakePositions.HOME: (IntakeConstants.ARM_HOME_ROTATIONS,),
            IntakePositions.STOWED: (IntakeConstants.ARM_STOWED_ROTATIONS,),
            IntakePositions.DEPLOYED: (IntakeConstants.ARM_DEPLOYED_ROTATIONS,),
        }
        self.goto_position_command_factory = lambda pos: cmd.runOnce(
            lambda: self.go_to_position(pos)
        )
        
        self.go_to_velocity_command_factory = lambda velocity: cmd.runOnce(
            lambda: self.set_velocity(velocity)
        )



    def set_velocity(self, current_velocity: float):
        # 2.6 * current_velocity for speed of roller
        if current_velocity < 1:
            current_velocity = 1
        commanded_velocity = current_velocity * IntakeConstants.INTAKE_VELOCITY_CONSTANT
        self._commanded_velocity_ft_per_sec = float(commanded_velocity)
        self._stop_requested = False

        # Convert m/sec to motor rotations/sec
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

    def go_to_position(self, position: IntakePositions):
        self._target_position_name = position.name
        arm_rot = self._POSITION_MAP[position]
        print(arm_rot)

        self.motor_arm.set_control(
            self._motion_magic_position_voltage_arm.with_position(arm_rot)
        )

    def sysIdQuasistaticRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.quasistatic(direction)

    def sysIdDynamicRollerTop(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_roller.dynamic(direction)
