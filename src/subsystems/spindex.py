from phoenix6.hardware import TalonFX
from phoenix6 import controls
from commands2 import cmd
import commands2
from utils import TalonConfig
from utils import MotorIDs

class Spindex(commands2.Subsystem):
    def __init__(self):
        SPINDEX_CONFIG = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)

        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.motor_spindex: TalonFX = TalonFX(
            MotorIDs.motor_id_motor_spindex,
        )

        SPINDEX_CONFIG._apply_settings(self.motor_spindex, inverted=False)

        self.set_velocity_command = cmd.runOnce(self.move_spindex)
        self.stop_velocity_command = cmd.runOnce(self.stop)

    def move_spindex(self, velocity: float = 1):
        """
        Args:
            velocity (float): roations per second. Defaults to 1.
        """
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

    def stop(self):
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )
