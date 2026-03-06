from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from wpilib import DriverStation, Mechanism2d, Color8Bit, RobotBase, SmartDashboard
from wpimath.geometry import Translation3d
from constants.robot_constants import MotorIDs
from subsystems import Drivetrain
from constants import Hub
import math
from constants import TurretConstants
from constants import SI
from pykit.logger import Logger as PyKitLogger

from utils.talon_config import TalonConfig
from phoenix6.hardware import TalonFX
from utils import generateSysIdProfile, get_red_pose


class Turret(Subsystem):
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

        if not RobotBase.isReal():
            TURRET_MOTOR_CONFIG = TalonConfig(
                kP=0.85,
                kI=0,
                kD=0.6,
                kF=0.0,
                kA=0.03,
                kV=0.11,
                brake_mode=True,
                motion_magic_cruise_velocity=8,
                motion_magic_acceleration=30,
            )
        else:
            TURRET_MOTOR_CONFIG = TalonConfig(
                kP=0.2, kI=0, kD=0, kF=0, kA=0, brake_mode=True
            )

        self.hood_motor = TalonFX(MotorIDs.motor_id_hood)
        self.turret_motor = TalonFX(MotorIDs.motor_id_turret)

        HOOD_MOTOR_CONFIG._apply_settings(self.hood_motor, inverted=False)
        TURRET_MOTOR_CONFIG._apply_settings(self.turret_motor, inverted=False)

        self._motion_magic_position_voltage_hood = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self._motion_magic_position_voltage_turret = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.target_hood_angle = 0.0
        self.target_turret_yaw = 0.0

        # Mechanism2d — top-down view: base = robot heading, turret = yaw, length = hood
        self.mechanism = Mechanism2d(3, 3)
        turret_root = self.mechanism.getRoot("TurretRoot", 1.5, 1.5)

        # Robot base — rotates with robot heading (0° = right)
        self.base_ligament = turret_root.appendLigament(
            "Base", 0.4, 0, 4, Color8Bit(100, 100, 100)
        )

        # Turret — rotates relative to base with turret yaw, length scales with hood angle
        self.turret_ligament = self.base_ligament.appendLigament(
            "Turret", 1.0, 0, 8, Color8Bit(0, 150, 255)
        )

        SmartDashboard.putData("Turret/Mechanism2d", self.mechanism)

        self.sys_id_routine_hood = generateSysIdProfile(
            self, self.hood_motor, name="Hood_Motor"
        )
        self.sys_id_routine_turret = generateSysIdProfile(
            self, self.turret_motor, name="Turret_Motor"
        )

    def set_angle_hood(self, angle_deg: float):
        self.target_hood_angle = angle_deg
        self.hood_motor.set_control(
            self._motion_magic_position_voltage_hood.with_position(
                SI.degrees_to_rotations * angle_deg
            )
        )

    def set_angle_turret(self, angle_deg: float):
        self.target_turret_yaw = angle_deg
        self.turret_motor.set_control(
            self._motion_magic_position_voltage_turret.with_position(
                SI.degrees_to_rotations * angle_deg * TurretConstants.TURRET_GEARING
            )
        )

    def set_target_hood_and_turret(self):
        v_fixed, hood_angle_deg, turret_yaw_deg = self._optimal_angle_calc(
            TurretConstants.SHOOTER_SET_VELOCITY_CONSTANT
        )
        if v_fixed < 0:
            return -1, -1, -1
        self.set_angle_hood(hood_angle_deg)
        self.set_angle_turret(turret_yaw_deg)
        return v_fixed, hood_angle_deg, turret_yaw_deg

    def periodic(self):
        self.set_target_hood_and_turret()

        actual_turret_yaw = (
            float(
                self.turret_motor.get_position().value / TurretConstants.TURRET_GEARING
            )
            * SI.rotations_to_degrees
        )
        actual_hood_angle = (
            float(self.hood_motor.get_position().value) * SI.rotations_to_degrees
        )

        # Update Mechanism2d
        robot_pose = self.driveSub.get_pose()
        if robot_pose is not None:
            self.base_ligament.setAngle(robot_pose.rotation().degrees())
        self.turret_ligament.setAngle(actual_turret_yaw)
        # Hood angle controls turret length — 0° = min length, 90° = max length
        hood_fraction = max(0, min(actual_hood_angle, 90)) / 90.0
        self.turret_ligament.setLength(0.3 + hood_fraction * 0.9)

        PyKitLogger.recordOutput(
            "Turret/target_hood_angle", float(self.target_hood_angle)
        )
        PyKitLogger.recordOutput(
            "Turret/target_turret_yaw", float(self.target_turret_yaw)
        )
        PyKitLogger.recordOutput(
            "Turret/actual_hood_angle",
            float(self.hood_motor.get_position().value) * SI.rotations_to_degrees,
        )
        PyKitLogger.recordOutput(
            "Turret/actual_turret_yaw",
            float(
                self.turret_motor.get_position().value / TurretConstants.TURRET_GEARING
            )
            * SI.rotations_to_degrees,
        )
        PyKitLogger.recordOutput(
            "Turret/hood_motor_voltage",
            float(self.hood_motor.get_motor_voltage().value_as_double),
        )
        PyKitLogger.recordOutput(
            "Turret/turret_motor_voltage",
            float(self.turret_motor.get_motor_voltage().value_as_double),
        )
        PyKitLogger.recordOutput(
            "Turret/turret_motor_current",
            float(self.turret_motor.get_stator_current().value_as_double),
        )

    def _optimal_angle_calc(
        self, v_fixed: float = TurretConstants.SHOOTER_SET_VELOCITY_CONSTANT
    ):
        """
        Args:
            v_fixed: Should be ~10m/s this season, but can be tuned for optimal performance

        Returns:
            turret_yaw (degree) is robot-relative, so 0 degrees is straight ahead of your robot's front. Where Positive = left, Negative = right
            hood_angle (degree), 0 = shooting horizontally, 90 = shooting straight up
            v_fixed (m/s) is the fixed velocity you input, returned for convenience
        """

        robot_pose = self.driveSub.get_pose()
        robot_speeds = self.driveSub.get_speeds()

        if robot_pose is None:
            PyKitLogger.recordOutput("Turret/calc_valid", False)
            PyKitLogger.recordOutput("Turret/calc_failure_reason", "robot_pose_is_none")
            return -1, -1, -1

        G = 9.81
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_pose = get_red_pose(Hub.TOP_CENTER_POINT.toTranslation2d())
            hub_pose = Translation3d(
                hub_pose.X(), hub_pose.Y(), Hub.TOP_CENTER_POINT.Z()
            )
        else:
            hub_pose = Hub.TOP_CENTER_POINT
        if hub_pose is None:
            PyKitLogger.recordOutput("Turret/calc_valid", False)
            PyKitLogger.recordOutput("Turret/calc_failure_reason", "hub_pose_is_none")
            return -1, -1, -1
        dx = hub_pose.X() - robot_pose.X()
        dy = hub_pose.Y() - robot_pose.Y()
        dz = hub_pose.Z() - TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M

        PyKitLogger.recordOutput("Turret/initial_dx", float(dx))
        PyKitLogger.recordOutput("Turret/initial_dy", float(dy))
        PyKitLogger.recordOutput("Turret/initial_dz", float(dz))
        PyKitLogger.recordOutput("Turret/v_fixed", float(v_fixed))

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

            PyKitLogger.recordOutput(f"Turret/Iterations/iter_{i}_r", float(r))
            PyKitLogger.recordOutput(
                f"Turret/Iterations/iter_{i}_discriminant", float(discriminant)
            )

            if discriminant < 0:
                PyKitLogger.recordOutput("Turret/calc_valid", False)
                PyKitLogger.recordOutput(
                    "Turret/calc_failure_reason",
                    "target_unreachable_discriminant_negative",
                )
                PyKitLogger.recordOutput("Turret/discriminant", float(discriminant))
                PyKitLogger.recordOutput("Turret/horizontal_distance", float(r))
                PyKitLogger.recordOutput(
                    f"Turret/Iterations/iter_{i}_hood_angle_deg", 0.0
                )
                PyKitLogger.recordOutput(f"Turret/Iterations/iter_{i}_tof", 0.0)
                return -1, -1, -1

            tan_theta = (-B - math.sqrt(discriminant)) / (2 * A)
            hood_angle = math.atan(tan_theta)
            tof = r / (math.cos(hood_angle) * v_fixed)

            PyKitLogger.recordOutput(
                f"Turret/Iterations/iter_{i}_hood_angle_deg",
                float(hood_angle * SI.radians_to_degrees),
            )
            PyKitLogger.recordOutput(f"Turret/Iterations/iter_{i}_tof", float(tof))

            dx = hub_pose.X() - robot_pose.X() - robot_speeds.vx * tof
            dy = hub_pose.Y() - robot_pose.Y() - robot_speeds.vy * tof

        field_angle = math.atan2(dy, dx)
        turret_yaw = field_angle - robot_pose.rotation().radians()
        turret_yaw = (turret_yaw + math.pi) % (2 * math.pi) - math.pi
        hood_angle_deg = hood_angle * SI.radians_to_degrees
        turret_yaw_deg = turret_yaw * SI.radians_to_degrees

        PyKitLogger.recordOutput("Turret/calc_valid", True)
        PyKitLogger.recordOutput("Turret/calc_failure_reason", "")
        PyKitLogger.recordOutput("Turret/target_hood_angle", float(hood_angle_deg))
        PyKitLogger.recordOutput("Turret/target_turret_yaw", float(turret_yaw_deg))
        PyKitLogger.recordOutput("Turret/hub_dx", float(hub_pose.X() - robot_pose.X()))
        PyKitLogger.recordOutput("Turret/hub_dy", float(hub_pose.Y() - robot_pose.Y()))
        PyKitLogger.recordOutput(
            "Turret/hub_dz",
            float(hub_pose.Z() - TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M),
        )
        PyKitLogger.recordOutput("Turret/horizontal_distance", float(r))
        PyKitLogger.recordOutput("Turret/time_of_flight", float(tof))
        PyKitLogger.recordOutput("Turret/discriminant", float(discriminant))

        return v_fixed, hood_angle_deg, turret_yaw_deg

    @staticmethod
    def find_v_fixed():
        G = 9.80665
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
