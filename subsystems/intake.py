import ntcore
from phoenix6 import controls 
from phoenix6.hardware import TalonFX
import commands2

from utils.TalonConfig import TalonConfig
from commands2 import button, cmd
import math
from phoenix6.signals.spn_enums import *


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        INTAKE_CONFIG_ARM = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)
        INTAKE_CONFIG_HEAD = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)
        
        INTAKE_CONFIG_ROLLER_TOP = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)
        INTAKE_CONFIG_ROLLER_BOTTOM = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)

        foc_active = False

        motor_id_arm = 51
        motor_id_head = 52
        motor_id_roller_top = 53
        motor_id_roller_bottom = 54
        
        self.motor_head: TalonFX = TalonFX(
            motor_id_head,
        )
        self.motor_arm: TalonFX = TalonFX(
            motor_id_arm,
        )

        self.motor_roller_top: TalonFX = TalonFX(
            motor_id_roller_top,
        )

        self.motor_roller_bottom: TalonFX = TalonFX(
            motor_id_roller_bottom,
        )

        INTAKE_CONFIG_ARM._apply_settings(self.motor_arm, inverted=False)
        INTAKE_CONFIG_HEAD._apply_settings(self.motor_head, inverted=False)
        INTAKE_CONFIG_ROLLER_TOP._apply_settings(self.motor_roller_top, inverted=True)
        INTAKE_CONFIG_ROLLER_BOTTOM._apply_settings(self.motor_roller_bottom, inverted=False)

        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(0, enable_foc=False)
        self.target_velocity = -1
        self.set_velocity_command = cmd.runOnce(self.set_velocity)
        self.stop_command = cmd.runOnce(self.stop)
        
        
        
    def init(self):
        pass

    def set_velocity(self, velocity: float = 1): # ft/sec
        # speed: rotations per seconds
        # sets inital velocity of speed rotations / sec
        # 25 ft / sec 
        # 4pi inches / sec
        # speed = 1 roation per seocnd
        # need to turn vel -> roations per second
        
        circumfrence = 1.374*math.pi # inches # 1.374*math.pi
        gear_ratio = 4

        velocity*=(12*gear_ratio)/circumfrence
        
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
            self._motion_magic_velocity_voltage.with_velocity(
                0
            ).with_acceleration(0.1)
        )

        self.motor_roller_bottom.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                0
            ).with_acceleration(0.1)
        )
        
    def update_table(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        table.getEntry("Velocity_Motor_Bottom").setDouble(float(self.motor_roller_bottom.get_velocity().value))
        table.getEntry("Velocity_Motor_Top").setDouble(float(self.motor_roller_top.get_velocity().value))
        table.getEntry("Target_Velocity").setDouble(float(self.target_velocity))
        #v            # 4pi inches / sec

    def periodic(self):
        self.update_table()
