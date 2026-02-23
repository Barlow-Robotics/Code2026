from dataclasses import dataclass

from phoenix6.hardware import TalonFX
from phoenix6 import controls
from commands2 import cmd
import commands2
from utils import TalonConfig, Logger
from constants import MotorIDs


@dataclass
class SpindexTelemetry:
    velocity: float
    supply_current: float
    stator_current: float
    supply_voltage: float
    motor_voltage: float
    device_temp: float
    is_inverted: bool


class Spindex(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.log = Logger("Spindex")
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
        self.setVelocity = -1
    def move_spindex(self, velocity: float = 1):
        """
        Args:
            velocity (float): roations per second. Defaults to 1.
        """
        self.setVelocity = velocity
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                velocity
            ).with_acceleration(0.1)
        )

    def stop(self):
        self.setVelocity = 0
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )

    def periodic(self):
        self.log.put("target_velocity", self.setVelocity)
        self.log.put("stop_requested", self.stop_requested)
        self.log.publish(
            SpindexTelemetry(
                velocity=float(self.motor_spindex.get_velocity().value),
                supply_current=float(self.motor_spindex.get_supply_current().value),
                stator_current=float(self.motor_spindex.get_stator_current().value),
                supply_voltage=float(self.motor_spindex.get_supply_voltage().value),
                motor_voltage=float(self.motor_spindex.get_motor_voltage().value),
                device_temp=float(self.motor_spindex.get_device_temp().value),
                is_inverted=bool(self.motor_spindex.get_applied_rotor_polarity().value),
            )
        )