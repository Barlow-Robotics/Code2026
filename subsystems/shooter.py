import ntcore
from phoenix6 import controls 
from phoenix6.hardware import TalonFX
import commands2

from utils.TalonConfig import TalonConfig
from commands2 import button, cmd

class Shooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        INTAKE_CONFIG = TalonConfig(kP=0.11, kI=0, kD=0, kF=0, kA=0, brake_mode=True)
        foc_active = False
        motor_id = 10
        self.motor: TalonFX = TalonFX(
            motor_id,
        )
        INTAKE_CONFIG._apply_settings(self.motor, inverted=False)
        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(0, enable_foc=False)
        self.target_velocity = -1
        self.set_velocity_command = cmd.runOnce(self.set_velocity)

    def init(self):
        pass

    def set_velocity(self, speed: float = 1):
        # speed: rotations per seconds
        # sets inital velocity of speed rotations / sec
        # 25 ft / sec 
        # 4pi inches / sec
        print("AAAx")
        self.motor.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                speed
            ).with_acceleration(0.1)
        )
        self.target_velocity = speed
        print("B")
        
    def stop(self):
        self.motor.set_control(
            self._motion_magic_velocity_voltage.with_velocity(
                0
            ).with_acceleration(0)
        )
        
    def update_table(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        table.getEntry("Velocity").setDouble(float(self.motor.get_velocity().value))
        table.getEntry("TargetVelocity").setDouble(float(self.target_velocity))
        #v            # 4pi inches / sec

    def periodic(self):
        self.update_table()
