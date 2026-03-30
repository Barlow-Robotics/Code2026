from phoenix6.hardware import TalonFX
from phoenix6 import controls
import commands2
from wpilib import SmartDashboard
from constants.robot_constants import RobotFeatures
from utils import TalonConfigFX
from constants import MotorIDs
from pykit.logger import Logger as PyKitLogger
from commands2.sysid import SysIdRoutine
from utils import generateSysIdProfile
from utils.profiler import LoopTimer
from constants import SpindexConstants


class Spindex(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Spindex")
        self.kP_spindex = 0.4
        self.kI_spindex = 0
        self.kD_spindex = 0.0005
        self.kV_spindex = 0.3
        SmartDashboard.putNumber("Spindex kP", self.kP_spindex)
        SmartDashboard.putNumber("Spindex kI", self.kI_spindex)
        SmartDashboard.putNumber("Spindex kD", self.kD_spindex)
        SmartDashboard.putNumber("Spindex kV", self.kV_spindex)
        self.motor_spindex: TalonFX = TalonFX(MotorIDs.motor_id_motor_spindex, "*")
        
        self.initMotors()        


        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(
            0, enable_foc=MotorIDs.foc_active
        )


        self.target_velocity_spindex = -1.0

        self.sys_id_routine_spindex = generateSysIdProfile(
            self, self.motor_spindex, name="Spindex"
        )

    def move(self, velocity: float = 1.0):
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
        self.target_velocity_spindex = 0.0
        self.motor_spindex.set_control(
            self._motion_magic_velocity_voltage.with_velocity(0).with_acceleration(0.1)
        )


    def initMotors(self):
        if RobotFeatures.SmartDashboardTuning:
            self.kP_spindex = SmartDashboard.getNumber("Spindex kP", 0.16)
            self.kI_spindex = SmartDashboard.getNumber("Spindex kI", 0)
            self.kD_spindex = SmartDashboard.getNumber("Spindex kD", 0)
            self.kV_spindex = SmartDashboard.getNumber("Spindex kV", 0.16)


        SPINDEX_CONFIG = TalonConfigFX(
            kP=self.kP_spindex,
            kI=self.kI_spindex,
            kD=self.kD_spindex,
            kV=self.kV_spindex,
            brake_mode=True,
            gear_ratio=SpindexConstants.GEARING,
        )
        SPINDEX_CONFIG._apply_settings(self.motor_spindex, inverted=True)

    def periodic(self):
        self._loop_timer.start()
        self.log_motor(self.motor_spindex, "Spindex", self.target_velocity_spindex)
        self._loop_timer.stop()

    def log_motor(self, motor: TalonFX, prefix: str, target_velocity: float):
        if RobotFeatures.LOW_LOGGING:
            PyKitLogger.recordOutput(f"{prefix}/target_RPS", float(target_velocity))
            PyKitLogger.recordOutput(
                f"{prefix}/current_RPS", float(motor.get_velocity().value)
            )

        if RobotFeatures.LOGGING_SPINDEX:
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
