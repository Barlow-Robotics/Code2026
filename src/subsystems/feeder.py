from phoenix6.hardware import TalonFXS
from phoenix6 import controls
import commands2
from utils import TalonConfig
from constants import MotorIDs
from pykit.logger import Logger as PyKitLogger
from commands2.sysid import SysIdRoutine
from utils import generateSysIdProfile
from utils.profiler import LoopTimer


class Feeder(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Feeder")
        FEEDER_CONFIG_CONSTANT = TalonConfig(
            kP=0.1, kI=0, kD=0, kF=4.0, kA=0, brake_mode=True
        )
        FEEDER_CONFIG_ALTERNATING = TalonConfig(
            kP=0.1, kI=0, kD=0, kF=4.0, kA=0, brake_mode=True
        )

        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.motor_feeder_constant: TalonFXS = TalonFXS( 
            MotorIDs.motor_id_motor_feeder_constant, "*"
        )
        self.motor_feeder_alternating: TalonFXS = TalonFXS(
            MotorIDs.motor_id_motor_feeder_alternating, "*"
        )

        FEEDER_CONFIG_CONSTANT._apply_settings(
            self.motor_feeder_constant, inverted=True
        )
        FEEDER_CONFIG_ALTERNATING._apply_settings(
            self.motor_feeder_alternating, inverted=True
        )
        self._duty_cycle_out = controls.DutyCycleOut(0, 
                                                     enable_foc=MotorIDs.foc_active)
        self._velocity_voltage = controls.VelocityVoltage(0,
                                                          enable_foc=MotorIDs.foc_active)



        self.target_velocity_feeder_constant = -1.0
        self.target_velocity_feeder_alternating = -1.0

        self.sys_id_routine_feeder_constant = generateSysIdProfile(
            self, self.motor_feeder_constant, name="Feeder_Constant"
        )
        self.sys_id_routine_feeder_alternating = generateSysIdProfile(
            self, self.motor_feeder_alternating, name="Feeder_Alternating"
        )
        

    def move(self, velocity: float = 0.17, invert=False):
        """
        Args:
            velocity (float): rotations per second. Defaults to 1.
            invert (bool): if True, reverses the alternating motor.
        """
        self.target_velocity_feeder_constant = velocity
        self.target_velocity_feeder_alternating = velocity
        print("MOVINNGHGGGG")
        # self.motor_feeder_constant.set_control(
        #     self._duty_cycle_out.with_output(velocity)
        # )
        # if invert:
        #     self.target_velocity_feeder_alternating = -velocity
        #     self.motor_feeder_alternating.set_control(
        #         self._duty_cycle_out.with_output(-velocity)
        #     )
        # else:
        #     self.target_velocity_feeder_alternating = velocity
        #     self.motor_feeder_alternating.set_control(
        #         self._duty_cycle_out.with_output(velocity)
        #     )

            # self.motor_feeder_alternating.set_control(
            #     self._motion_magic_velocity_voltage.with_velocity(
            #         velocity
            #     ).with_acceleration(2)
            # )
    
        # 0.094 * speed

        # self.motor_feeder_constant.set(0.2)
        velocity = 31.25 # rotations/sec
        self.motor_feeder_constant.set_control(
            self._velocity_voltage.with_velocity(
                velocity
            )
        )
        if invert:
            self.target_velocity_feeder_alternating = -velocity
            self.motor_feeder_alternating.set_control(
                self._velocity_voltage.with_velocity(
                    -velocity
                )
            )
        else:
            self.motor_feeder_alternating.set_control(
                self._velocity_voltage.with_velocity(
                    velocity
                )
            )

    def stop(self):
        self.target_velocity_feeder_constant = 0
        self.target_velocity_feeder_alternating = 0
        self.motor_feeder_constant.set_control(
            self._velocity_voltage.with_velocity(0)
        )
        self.motor_feeder_alternating.set_control(
            self._velocity_voltage.with_velocity(0)
        )

    def periodic(self):
        self._loop_timer.start()
        self.log_motor(
            self.motor_feeder_constant,
            "Feeder_Constant",
            self.target_velocity_feeder_constant,
        )
        self.log_motor(
            self.motor_feeder_alternating,
            "Feeder_Alternating",
            self.target_velocity_feeder_alternating,
        )
        self._loop_timer.stop()

    def log_motor(self, motor: TalonFXS, prefix: str, target_velocity: float):
        PyKitLogger.recordOutput(f"{prefix}/target_velocity", target_velocity)
        PyKitLogger.recordOutput(
            f"{prefix}/current_RPS", float(motor.get_velocity().value)
        )
        PyKitLogger.recordOutput(
            f"{prefix}/current_RPM", float(motor.get_velocity().value * 60)
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

    def sysIdQuasistaticFeederConstant(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_feeder_constant.quasistatic(direction)

    def sysIdDynamicFeederConstant(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_feeder_constant.dynamic(direction)

    def sysIdQuasistaticFeederAlternating(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_feeder_alternating.quasistatic(direction)

    def sysIdDynamicFeederAlternating(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_feeder_alternating.dynamic(direction)
