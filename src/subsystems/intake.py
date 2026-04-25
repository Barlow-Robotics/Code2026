import commands2
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from phoenix6.hardware import TalonFX
import wpilib
from wpilib import DriverStation, RobotBase, Mechanism2d, Color8Bit, SmartDashboard
from pykit.logger import Logger as PyKitLogger

from constants.robot_constants import RobotFeatures
from utils.profiler import LoopTimer
from constants import IntakeConstants, MotorIDs
from utils import TalonConfigFX, generateSysIdProfile, IntakePositions


class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._loop_timer = LoopTimer("Intake")
        self.target_velocity = 0.0
        self._target_position_name = "NONE"
        self._target_position = IntakePositions.HOME
        self.target_rot: float = 0.0

        # Mechanism2d
        if RobotFeatures.LOGGING_ROBOT:
            self.mechanism = Mechanism2d(4, 3)
            root = self.mechanism.getRoot("IntakeRoot", 0.5, 2.5)
            # Arm extends forward-and-down at ~20° below horizontal.
            # Length is set dynamically in periodic() based on motor position.
            self.arm_ligament = root.appendLigament(
                "Arm", 0.2, -20, 8, Color8Bit(0, 200, 0)
            )
            self.roller_ligament = self.arm_ligament.appendLigament(
                "Roller", 0.2, 0, 10, Color8Bit(0, 150, 0)
            )
            wpilib.SmartDashboard.putData("Intake/Mechanism2d", self.mechanism)

        self.motor_roller: TalonFX = TalonFX(MotorIDs.motor_id_roller, "*")
        self.motor_arm_leader: TalonFX = TalonFX(MotorIDs.motor_id_arm_leader_left, "*")
        # self.motor_arm_follower: TalonFX = TalonFX(MotorIDs.motor_id_arm_follower_right, "*")
        # Roller gains are tuned around the backwards ROLLER_GEARING value
        # (see IntakeConstants). Post-flip values (use together with
        # ROLLER_GEARING = 58/30) are listed in commented-out form.
        self.kP_roller = 0.01
        # self.kP_roller = 0.037
        self.kI_roller = 0.0
        self.kD_roller = 0.0
        self.kV_roller = 0.06
        # self.kV_roller = 0.224
        self.kG_roller = 0.00
        self.kS_roller = 0.00
        self.cruise_velocity_roller = 20
        # self.cruise_velocity_roller = 5.35
        self.cruise_accl_roller = 600
        # self.cruise_accl_roller = 160
        self.kP_arm = 0.8  # 0.8
        self.kI_arm = 0.0
        self.kD_arm = 0.0
        self.kV_arm = 0.63
        self.kG_arm = -0.11  # -0.11
        self.kS_arm = 0.00
        self.cruise_accl_arm = 20
        self.cruise_velocity_arm = 10
        self.arm_distance = 3.28

        self.stator_current_limit_arm = 40
        self.supply_current_limit_arm = 40

        if RobotFeatures.SmartDashboardTuning:
            SmartDashboard.putNumber("stator_current_limit_arm", 40)
            SmartDashboard.putNumber("supply_current_limit_arm", 40)

            SmartDashboard.putNumber("kP_Roller", self.kP_roller)
            SmartDashboard.putNumber("kI_Roller", self.kI_roller)
            SmartDashboard.putNumber("kD_Roller", self.kD_roller)
            SmartDashboard.putNumber("kV_Roller", self.kV_roller)
            SmartDashboard.putNumber("kG_Roller", self.kG_roller)
            SmartDashboard.putNumber("kS_Roller", self.kS_roller)

            SmartDashboard.putNumber("kP_IntakeArm", self.kP_arm)
            SmartDashboard.putNumber("kI_IntakeArm", self.kI_arm)
            SmartDashboard.putNumber("kD_IntakeArm", self.kD_arm)
            SmartDashboard.putNumber("kV_IntakeArm", self.kV_arm)
            SmartDashboard.putNumber("kG_IntakeArm", self.kG_arm)
            SmartDashboard.putNumber("kS_IntakeArm", self.kS_arm)
            SmartDashboard.putNumber("arm_rot", self.arm_distance)
            SmartDashboard.putNumber("cruise_velocity_arm", self.cruise_velocity_arm)
            SmartDashboard.putNumber("cruise_accl_arm", self.cruise_accl_arm)
            SmartDashboard.putNumber(
                "cruise_velocity_roller", self.cruise_velocity_roller
            )
            SmartDashboard.putNumber("cruise_accl_roller", self.cruise_accl_roller)

        self.reinitMotors()

        self._motion_magic_velocity_voltage_roller = (
            controls.MotionMagicVelocityVoltage(0, enable_foc=MotorIDs.foc_enabled())
        )
        self._motion_magic_position_voltage_arm_leader = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_enabled()
        )
        self._motion_magic_position_voltage_arm_follower = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_enabled()
        )

        self.sys_id_routine_roller = generateSysIdProfile(
            self, self.motor_roller, name="Roller"
        )

        self._POSITION_MAP = {
            IntakePositions.HOME: IntakeConstants.ARM_HOME_ROTATIONS,
            IntakePositions.REVERSE: IntakeConstants.ARM_DEPLOYED_ROTATIONS,
            IntakePositions.DEPLOYED: IntakeConstants.ARM_DEPLOYED_ROTATIONS,
        }

    def reinitMotors(self):
        if RobotFeatures.SmartDashboardTuning:
            INTAKE_ROLLER = TalonConfigFX(
                kP=SmartDashboard.getNumber("kP_Roller", 0.0),
                kI=SmartDashboard.getNumber("kI_Roller", 0.0),
                kD=SmartDashboard.getNumber("kD_Roller", 0.0),
                kV=SmartDashboard.getNumber("kV_Roller", 0.00),
                kG=SmartDashboard.getNumber("kG_Roller", 0.00),
                kS=SmartDashboard.getNumber("kS_Roller", 0.00),
                brake_mode=False,
                gear_ratio=IntakeConstants.ROLLER_GEARING,
                motion_magic_cruise_velocity=SmartDashboard.getNumber(
                    "cruise_velocity_roller", 20
                ),
                motion_magic_acceleration=SmartDashboard.getNumber(
                    "cruise_accl_roller", 600
                ),
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
                motion_magic_cruise_velocity=SmartDashboard.getNumber(
                    "cruise_velocity_arm", 1
                ),
                motion_magic_acceleration=SmartDashboard.getNumber(
                    "cruise_accl_arm", 2
                ),
                current_limit=SmartDashboard.getNumber("stator_current_limit_arm", 40),
                supply_current_limit=SmartDashboard.getNumber(
                    "supply_current_limit_arm", 40
                ),
            )
        else:
            INTAKE_ROLLER = TalonConfigFX(
                kP=self.kP_roller,
                kI=self.kI_roller,
                kD=self.kD_roller,
                kV=self.kV_roller,
                kG=self.kG_roller,
                kS=self.kS_roller,
                brake_mode=False,
                gear_ratio=IntakeConstants.ROLLER_GEARING,
                motion_magic_cruise_velocity=self.cruise_velocity_roller,
                motion_magic_acceleration=self.cruise_accl_roller,
            )
            INTAKE_CONFIG_ARM_RIGHT = TalonConfigFX(
                kP=self.kP_arm,
                kI=self.kI_arm,
                kD=self.kD_arm,
                kV=self.kV_arm,
                kG=self.kG_arm,
                kS=self.kS_arm,
                brake_mode=True,
                gear_ratio=IntakeConstants.ARM_GEARING,
                motion_magic_cruise_velocity=self.cruise_velocity_arm,
                motion_magic_acceleration=self.cruise_accl_arm,
                current_limit=self.stator_current_limit_arm,
                supply_current_limit=self.supply_current_limit_arm,
            )

        INTAKE_ROLLER._apply_settings(self.motor_roller, inverted=False)
        INTAKE_CONFIG_ARM_RIGHT._apply_settings(self.motor_arm_leader, inverted=False)

    def activate_roller(self):
        rps = (
            IntakeConstants.ROLLER_RPS_INTAKE_AUTO
            if DriverStation.isAutonomous()
            else IntakeConstants.ROLLER_RPS_INTAKE
        )
        self.target_velocity = rps
        self.motor_roller.set_control(
            self._motion_magic_velocity_voltage_roller.with_velocity(
                rps
            ).with_acceleration(0.1)
        )

    def reverse_roller(self):
        rps = IntakeConstants.ROLLER_RPS_REVERSE
        self.target_velocity = rps
        self.motor_roller.set_control(
            self._motion_magic_velocity_voltage_roller.with_velocity(
                rps
            ).with_acceleration(0.1)
        )

    def stop(self):
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
        if position == IntakePositions.HOME:
            target_position = 0
            self.motor_arm_leader.set_control(
                self._motion_magic_position_voltage_arm_leader.with_position(
                    target_position
                )
            )
        else:
            target_position = self.arm_distance
            self.motor_arm_leader.set_control(
                self._motion_magic_position_voltage_arm_leader.with_position(
                    target_position
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
        self.target_rot: float = target_position

    def periodic(self):
        self._loop_timer.start()

        if not RobotBase.isReal() and RobotFeatures.LOGGING_ROBOT:
            position_rot = self.motor_arm_leader.get_position().value
            self.arm_ligament.setLength(0.2 + position_rot * 0.3)
            roller_rot = self.motor_roller.get_position().value
            self.roller_ligament.setAngle((roller_rot * 360.0) % 360.0)
        if RobotFeatures.LOW_LOGGING:
            PyKitLogger.recordOutput(
                "Intake/Actual_Arm_Position",
                float(self.motor_arm_leader.get_position().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Actual_Arm_Error",
                float(self.motor_arm_leader.get_closed_loop_error().value),
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

        if RobotFeatures.LOGGING_INTAKE:
            PyKitLogger.recordOutput(
                "Intake/Stator_Current_Arm",
                float(self.motor_arm_leader.get_stator_current().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Closed_Loop_Refrence_Arm",
                float(self.motor_arm_leader.get_closed_loop_reference().value),
            )
            PyKitLogger.recordOutput(
                "Intake/Closed_Loop_Refrence_Slope_Arm",
                float(self.motor_arm_leader.get_closed_loop_reference_slope().value),
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
