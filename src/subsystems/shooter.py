from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from wpilib import RobotBase
from wpimath.units import volts
from constants.robot_constants import MotorIDs
from subsystems import Drivetrain
from constants import Hub
import math
from constants import ShooterConstants
from constants import SI
from rev import (
    FeedbackSensor,
    SparkBaseConfig,
    SparkFlex,
    SparkFlexConfig,
    ResetMode,
    PersistMode,
)
from pykit.logger import Logger as PyKitLogger

from utils.talon_config import TalonConfig
from phoenix6.hardware import TalonFX
from commands2 import cmd
from utils import generateSysIdProfile
# 4 motors
# 2 control flywheel to shoot ball - neo vortex moter: SparkFlex, SparkFlexExternalEncoder ------ MotionMagicVelocityVoltage
# One controls hood angle - CTR minion motor -  MotionMagicVoltage
# One controls turret rotation - CTR minion  motor: TalonFX - MotionMagicVoltage

# auto aim, set both hood angle and turret rotation


class Shooter(Subsystem):
    def __init__(self, driveSub: Drivetrain):
        super().__init__()
        self.driveSub = driveSub

        HOOD_MOTOR_CONFIG = TalonConfig(
            kP=0.2, kI=0.0, kD=0, kF=0, kA=0.5, brake_mode=True
        )
        if not RobotBase.isReal():
            HOOD_MOTOR_CONFIG = TalonConfig(
                kP=10, kI=0, kD=6, kF=0, kA=0, brake_mode=True
            )

        TURRET_MOTOR_CONFIG = TalonConfig(
            kP=0.2, kI=0, kD=0, kF=0, kA=0, brake_mode=True
        )

        self.hood_motor = TalonFX(MotorIDs.motor_id_hood)
        self.turret_motor = TalonFX(MotorIDs.motor_id_turret)

        self.flywheel_motor_left_leader = SparkFlex(
            MotorIDs.motor_id_flywheel_left, type=SparkFlex.MotorType.kBrushless
        )
        self.flywheel_motor_right_follower = SparkFlex(
            MotorIDs.motor_id_flywheel_right, type=SparkFlex.MotorType.kBrushless
        )

        HOOD_MOTOR_CONFIG._apply_settings(self.hood_motor, inverted=False)
        TURRET_MOTOR_CONFIG._apply_settings(self.turret_motor, inverted=False)

        self._motion_magic_position_voltage = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )
        leader_config = SparkFlexConfig()
        leader_config.setIdleMode(
            leader_config.IdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        leader_config.smartCurrentLimit(80, freeLimit=5700)  # set to 5700 for max

        leader_config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            0.0001, 0.0, 0.0
        ).velocityFF(0.000175).outputRange(-1, 1)

        leader_config.closedLoop.maxMotion.maxVelocity(5700).maxAcceleration(
            10000
        ).allowedClosedLoopError(10)

        self.flywheel_motor_left_leader.configure(
            leader_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        follower_config = SparkFlexConfig()
        follower_config.setIdleMode(
            follower_config.IdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        follower_config.smartCurrentLimit(80, freeLimit=5700)  # set to 5700 for max

        follower_config.follow(self.flywheel_motor_left_leader, False)

        self.flywheel_motor_right_follower.configure(
            follower_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.flywheel_target_velocity = 0.0
        self.target_hood_angle = 0.0
        self.target_turret_yaw = 0.0

        self.start_flywheel_command = cmd.runOnce(
            lambda: self.setRPM(ShooterConstants.FLYWHEEL_RPM_CONSTANT)
        )

        self.sys_id_routine_hood = generateSysIdProfile(
            self, self.hood_motor, name="Hood_Motor"
        )
        self.sys_id_routine_turret = generateSysIdProfile(
            self, self.turret_motor, name="Turret_Motor"
        )

    def set_angle_hood(self, angle_deg: float):
        self.target_hood_angle = angle_deg
        self.hood_motor.set_control(
            self._motion_magic_position_voltage.with_position(
                SI.degrees_to_rotations * angle_deg
            )
        )
        
    def setRPM(self, targetRPM: float):
        self.flywheel_target_velocity = targetRPM
        self.flywheel_motor_left_leader.getClosedLoopController().setReference(
            targetRPM, SparkFlex.ControlType.kMAXMotionVelocityControl
        )

    def get_current_rpm(self) -> float:
        return self.flywheel_motor_left_leader.getEncoder().getVelocity()

    def stop_flywheel(self):
        self.flywheel_motor_left_leader.set(0)

    def set_angle_turret(self, angle_deg: float):
        self.target_turret_yaw = angle_deg
        self.turret_motor.set_control(
            self._motion_magic_position_voltage.with_position(
                SI.degrees_to_rotations * angle_deg
            )
        )

    def set_target_hood_and_turret(self):
        # self._optimal_angle_calc(ShooterConstants.SHOOTER_SET_VELOCITY_CONSTANT)
        v_fixed, hood_angle_deg, turret_yaw_deg = self._optimal_angle_calc(
            ShooterConstants.SHOOTER_SET_VELOCITY_CONSTANT
        )
        if v_fixed < 0:
            return -1, -1, -1
        self.set_angle_hood(hood_angle_deg)
        self.set_angle_turret(turret_yaw_deg)
        return v_fixed, hood_angle_deg, turret_yaw_deg

    def periodic(self):
        self.set_target_hood_and_turret()
        PyKitLogger.recordOutput(
            "Shooter/target_hood_angle", float(self.target_hood_angle)
        )
        PyKitLogger.recordOutput(
            "Shooter/target_turret_yaw", float(self.target_turret_yaw)
        )
        PyKitLogger.recordOutput(
            "Shooter/hood_motor_position",
            float(self.hood_motor.get_position().value) * SI.rotations_to_degrees,
        )
        PyKitLogger.recordOutput(
            "Shooter/turret_motor_position",
            float(self.turret_motor.get_position().value) * SI.rotations_to_degrees,
        )

        PyKitLogger.recordOutput(
            "Shooter/hood_motor_voltage",
            float(self.hood_motor.get_motor_voltage().value_as_double),
        )
        PyKitLogger.recordOutput(
            "Shooter/turret_motor_voltage",
            float(self.turret_motor.get_motor_voltage().value_as_double),
        )
        PyKitLogger.recordOutput(
            "Shooter/flywheel_motor_left_velocity", float(self.get_current_rpm())
        )
        PyKitLogger.recordOutput(
            "Shooter/flywheel_motor_left_target_velocity",
            float(self.flywheel_target_velocity),
        )

    def _optimal_angle_calc(
        self, v_fixed: float = ShooterConstants.SHOOTER_SET_VELOCITY_CONSTANT
    ):
        """
        Args:
            v_fixed: Should be ~10m/s this season, but can be tuned for optimal performance

        Returns:
            turret_yaw (degree) is robot-relative, so 0 degrees is straight ahead of your robot's front. Where Positive = left, Negative = right
            hood_angle (degree), 0° = shooting horizontally, 90° = shooting straight up
            v_fixed (m/s) is the fixed velocity you input, returned for convenience
        """

        robot_pose = self.driveSub.get_pose()
        robot_speeds = self.driveSub.get_speeds()

        if robot_pose is None:
            PyKitLogger.recordOutput("Shooter/calc_valid", False)
            PyKitLogger.recordOutput(
                "Shooter/calc_failure_reason", "robot_pose_is_none"
            )
            return -1, -1, -1

        G = 9.81
        hub_pose = Hub.TOP_CENTER_POINT

        dx = hub_pose.X() - robot_pose.X()
        dy = hub_pose.Y() - robot_pose.Y()
        dz = hub_pose.Z() - ShooterConstants.SHOOTER_HEIGHT_FOR_FUEL_M

        PyKitLogger.recordOutput("Shooter/initial_dx", float(dx))
        PyKitLogger.recordOutput("Shooter/initial_dy", float(dy))
        PyKitLogger.recordOutput("Shooter/initial_dz", float(dz))
        PyKitLogger.recordOutput("Shooter/v_fixed", float(v_fixed))

        discriminant = 0.0
        hood_angle = 0.0
        tof = 0.0
        r = 0.0

        for i in range(3):
            r = math.sqrt(dx * dx + dy * dy)

            A = (G * r**2) / (2 * v_fixed**2)
            B = -r
            C = dz + A

            discriminant = B**2 - 4 * A * C

            PyKitLogger.recordOutput(f"Shooter/Iterations/iter_{i}_r", float(r))
            PyKitLogger.recordOutput(
                f"Shooter/Iterations/iter_{i}_discriminant", float(discriminant)
            )

            if discriminant < 0:
                PyKitLogger.recordOutput("Shooter/calc_valid", False)
                PyKitLogger.recordOutput(
                    "Shooter/calc_failure_reason",
                    "target_unreachable_discriminant_negative",
                )
                PyKitLogger.recordOutput("Shooter/discriminant", float(discriminant))
                PyKitLogger.recordOutput("Shooter/horizontal_distance", float(r))
                PyKitLogger.recordOutput(
                    f"Shooter/Iterations/iter_{i}_hood_angle_deg", 0.0
                )
                PyKitLogger.recordOutput(f"Shooter/Iterations/iter_{i}_tof", 0.0)
                return -1, -1, -1

            tan_theta = (-B - math.sqrt(discriminant)) / (2 * A)
            hood_angle = math.atan(tan_theta)
            tof = r / (math.cos(hood_angle) * v_fixed)

            PyKitLogger.recordOutput(
                f"Shooter/Iterations/iter_{i}_hood_angle_deg",
                float(hood_angle * SI.radians_to_degrees),
            )
            PyKitLogger.recordOutput(f"Shooter/Iterations/iter_{i}_tof", float(tof))

            dx = hub_pose.X() - robot_pose.X() - robot_speeds.vx * tof
            dy = hub_pose.Y() - robot_pose.Y() - robot_speeds.vy * tof

        field_angle = math.atan2(dy, dx)
        turret_yaw = field_angle - robot_pose.rotation().radians()
        turret_yaw = (turret_yaw + math.pi) % (2 * math.pi) - math.pi
        hood_angle_deg = hood_angle * SI.radians_to_degrees
        turret_yaw_deg = turret_yaw * SI.radians_to_degrees

        PyKitLogger.recordOutput("Shooter/calc_valid", True)
        PyKitLogger.recordOutput("Shooter/calc_failure_reason", "")
        PyKitLogger.recordOutput("Shooter/hood_angle", float(hood_angle_deg))
        PyKitLogger.recordOutput("Shooter/turret_yaw", float(turret_yaw_deg))
        PyKitLogger.recordOutput("Shooter/hub_dx", float(hub_pose.X() - robot_pose.X()))
        PyKitLogger.recordOutput("Shooter/hub_dy", float(hub_pose.Y() - robot_pose.Y()))
        PyKitLogger.recordOutput(
            "Shooter/hub_dz",
            float(hub_pose.Z() - ShooterConstants.SHOOTER_HEIGHT_FOR_FUEL_M),
        )
        PyKitLogger.recordOutput("Shooter/horizontal_distance", float(r))
        PyKitLogger.recordOutput("Shooter/time_of_flight", float(tof))
        PyKitLogger.recordOutput("Shooter/discriminant", float(discriminant))

        return v_fixed, hood_angle_deg, turret_yaw_deg

    @staticmethod
    def find_v_fixed():
        G = 9.81
        height_needed = 1.8288
        height_of_shooter = 0.5224
        dz = height_needed - height_of_shooter

        r_min = 0.6
        r_max = 5.549110139617

        results = []
        for v_tenth in range(10, 150):
            v = v_tenth * 0.1
            valid = True
            for r in [r_min, r_max]:
                A = (G * r**2) / (2 * v**2)
                B = -r
                C = dz + A
                if B**2 - 4 * A * C < 0:
                    valid = False
                    break
            if valid:
                results.append(v)

        if results:
            v_min = results[0]
            print(f"Minimum v_fixed: {v_min:.1f} m/s ({v_min * 3.28084:.1f} ft/s)")
            print(
                f"Recommended v_fixed: {v_min * 1.15:.1f} m/s ({v_min * 1.15 * 3.28084:.1f} ft/s) (15% margin)"
            )

    def sysIdQuasistaticHood(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_hood.quasistatic(direction)

    def sysIdDynamicHood(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_hood.dynamic(direction)

    def sysIdQuasistaticTurret(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_turret.quasistatic(direction)

    def sysIdDynamicTurret(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine_turret.dynamic(direction)

# 1. slywheel motor spins up to speed. 
# both at same time
# 2. spindex runs const
# 3. feeder starts only once shooter is at speed. 


# eject comamnd for spindex/feeder


# shoot to other side of field when -1 -1 -1 