from toolkit.motors.ctre_motors import TalonFX

from toolkit.subsystem import Subsystem
from phoenix6.hardware import CANcoder
import ntcore
from toolkit.motors.ctre_motors import TalonConfig

import math


class Intake(Subsystem):
    def __init__(self):
        config_talon = TalonConfig(0, 0, 0, 0, 0, brake_mode=True)
        foc_active = False
        motor_id = 10
        super().__init__()
        self.motor: TalonFX = TalonFX(
            motor_id,
            foc_active,
            inverted=False,
            config=config_talon,
        )

    def init(self):
        self.motor.init()

    def set_velocity(self, speed: float):
        self.motor.set_target_velocity(speed)
