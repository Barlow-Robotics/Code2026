from phoenix6.hardware import TalonFX
from phoenix6 import controls
import commands2
from utils import TalonConfigFX
from constants import MotorIDs
from pykit.logger import Logger as PyKitLogger
from commands2.sysid import SysIdRoutine
from utils import generateSysIdProfile
from utils.profiler import LoopTimer


class Spindex(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Spindex")
        SPINDEX_CONFIG = TalonConfigFX(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)

        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.motor_spindex: TalonFX = TalonFX(
            MotorIDs.motor_id_motor_spindex, "*"
        )

        SPINDEX_CONFIG._apply_settings(self.motor_spindex, inverted=False)

        self.target_velocity_spindex = -1

        self.sys_id_routine_spindex = generateSysIdProfile(
            self, self.motor_spindex, name="Spindex"
        )

    def move(self, velocity: float = 1):
        """
        Args:
            velocity (float): rotations per second. Defaults to 1.
        """
        self.target_velocity_spindex = velocity
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

    def stop(self):
        self.target_velocity_spindex = 0
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

    def periodic(self):
        self._loop_timer.start()
        self.log_motor(self.motor_spindex, "Spindex", self.target_velocity_spindex)
        self._loop_timer.stop()

    def log_motor(self, motor: TalonFX, prefix: str, target_velocity: float):
        PyKitLogger.recordOutput(f"{prefix}/target_velocity", target_velocity)
        PyKitLogger.recordOutput(
            f"{prefix}/current_velocity", float(motor.get_velocity().value)
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_supply_current",
            float(motor.get_supply_current().value),
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_stator_current",
            float(motor.get_stator_current().value),
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_supply_voltage",
            float(motor.get_supply_voltage().value),
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_motor_voltage",
            float(motor.get_motor_voltage().value),
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_device_temp",
            float(motor.get_device_temp().value),
        )

    def sysIdQuasistaticSpindex(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_spindex.quasistatic(direction)

    def sysIdDynamicSpindex(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_spindex.dynamic(direction)
