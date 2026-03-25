from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import controls
from wpilib import (
    DriverStation,
    Mechanism2d,
    Color8Bit,
    RobotBase,
    SmartDashboard,
)
from wpimath.geometry import Translation3d, Pose2d
from constants.field_constants import CustomPoints
from constants.robot_constants import MotorIDs, RobotFeatures
from subsystems import Drivetrain
from constants import Hub
import math
from constants import TurretConstants
from constants import SI
from utils.telemetry import Logger
from utils import TalonConfigFXS
from phoenix6.hardware import TalonFXS
from utils import generateSysIdProfile, get_red_pose
from utils.profiler import LoopTimer
from wpimath.geometry import Rotation2d
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.signals import SensorDirectionValue
from phoenix6.configs import FeedbackConfigs
from phoenix6.signals import FeedbackSensorSourceValue

Logger = Logger("Turret")


class Turret(Subsystem):
    def __init__(self, driveSub: Drivetrain, init2: bool = False):
        super().__init__()
        self._loop_timer = LoopTimer("Turret")
        self.driveSub = driveSub
        self.actual_velocity_to_go = 15.5
        if not init2:
            SmartDashboard.putNumber("kV", 1.8)
            SmartDashboard.putNumber("kD", 1.5)
            SmartDashboard.putNumber("kI", 0.0)
            SmartDashboard.putNumber("kP", 0.0)
            SmartDashboard.putNumber("kS", 0.43)
            SmartDashboard.putNumber("kG", 0.0)

            SmartDashboard.putNumber("motion_magic_cruise_velocity", 0.0)

        HOOD_MOTOR_CONFIG = TalonConfigFXS(
            kP=SmartDashboard.getNumber("kP", 0.0),
            kI=SmartDashboard.getNumber("kI", 0.0),
            kD=SmartDashboard.getNumber("kD", 1.5),
            kV=SmartDashboard.getNumber("kV", 1.8),
            kS=SmartDashboard.getNumber("kS", 0.43),
            kG=SmartDashboard.getNumber("kG", 0.0),
            brake_mode=True,
            # gear_ratio=TurretConstants.HOOD_GEARING,
        )
        if not RobotBase.isReal():
            HOOD_MOTOR_CONFIG = TalonConfigFXS(
                kP=10,
                kI=0,
                kD=6,
                kV=0,
                brake_mode=True,
                # gear_ratio=TurretConstants.HOOD_GEARING,
                motion_magic_cruise_velocity=0.1,
            )

        # if not RobotBase.isReal():
        #     TURRET_MOTOR_CONFIG = TalonConfigFXS(
        #         kP=0.85,
        #         kI=0,
        #         kD=0.6,
        #         kV=0,
        #         brake_mode=True,
        #         motion_magic_cruise_velocity=8,
        #         motion_magic_acceleration=30,
        #         gear_ratio=TurretConstants.TURRET_GEARING,
        #     )
        # else:
        #     TURRET_MOTOR_CONFIG = TalonConfigFXS(
        #         kP=0.2,
        #         kI=0,
        #         kD=0,
        #         kV=0,
        #         brake_mode=True,
        #         gear_ratio=TurretConstants.TURRET_GEARING,
        #     )

        self.hood_motor = TalonFXS(MotorIDs.motor_id_hood, "*")
        # self.turret_motor = TalonFX(MotorIDs.motor_id_turret, "*")
        self.hood_cancoder = CANcoder(MotorIDs.cancoder_id_hood, "*")

        hood_cancoder_config = CANcoderConfiguration()
        hood_cancoder_config.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
        )

        # hood_cancoder_config.magnet_sensor.magnet_offset = +66 * SI.degrees_to_rotations
        self.hood_cancoder.configurator.apply(hood_cancoder_config, 0.2)
        # hood_cancoder_config.magnet_sensor.magnet_offset = -self.hood_cancoder.get_absolute_position().value
        # self.hood_cancoder.configurator.apply(hood_cancoder_config, 0.4)
        if not init2:
            self.hood_cancoder.set_position(0, 0.2)

        feedback_cfg = FeedbackConfigs()

        feedback_cfg.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        feedback_cfg.feedback_remote_sensor_id = MotorIDs.cancoder_id_hood

        feedback_cfg.sensor_to_mechanism_ratio = TurretConstants.HOOD_MECHANICAL_RATIO
        feedback_cfg.rotor_to_sensor_ratio = TurretConstants.HOOD_GEARING
        HOOD_MOTOR_CONFIG._apply_settings(self.hood_motor, inverted=True)
        self.hood_motor.configurator.apply(feedback_cfg)

        # TURRET_MOTOR_CONFIG._apply_settings(self.turret_motor, inverted=False)

        self._motion_magic_position_voltage_hood = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self._motion_magic_position_voltage_turret = controls.MotionMagicVoltage(
            0, enable_foc=MotorIDs.foc_active
        )

        self.target_hood_angle = 0.0
        self.target_turret_yaw = 0.0
        self.turret_yaw_deg = Rotation2d(0)
        SmartDashboard.putNumber("Turret/SHOOTER_SET_VELOCITY_CONSTANT", 0)
        # Mechanism2d — top-down view: base = robot heading, turret = yaw, length = hood
        if RobotFeatures.LOGGING_ROBOT:
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
        # self.sys_id_routine_turret = generateSysIdProfile(
        #     self, self.turret_motor, name="Turret_Motor"
        # )

        # self.left_turret_limit_switch = DigitalInput(0)
        # self.right_turret_limit_switch = DigitalInput(1)
        self.target_robot_heading: Rotation2d | None = None

    def set_angle_hood(self, angle_deg: float):
        self.target_hood_angle = angle_deg
        self.hood_motor.set_control(
            self._motion_magic_position_voltage_hood.with_position(
                SI.degrees_to_rotations * angle_deg
            )
        )

    # def left_limit_switch_on(self):
    #     return not self.left_turret_limit_switch.get()

    # def right_limit_switch_on(self):
    #     return not self.right_turret_limit_switch.get()

    def set_angle_turret(self, angle_deg: float):
        if not RobotFeatures.HAS_TURRET_ANGLE:
            current_robot_yaw_deg = self.driveSub.get_rotation().degrees()
            self.driveSub.target_field_heading = Rotation2d.fromDegrees(
                current_robot_yaw_deg - angle_deg
            )
            self.driveSub.changeAng = True
            self.driveSub.changeAngTelop = False
            

            # print("SET_ANGLE_TURRET")
        else:
            self.target_turret_yaw = angle_deg
            if self.left_limit_switch_on() and angle_deg < self.get_actual_turret_yaw():
                print("Left limit switch triggered, preventing further left movement")
            elif (
                self.right_limit_switch_on()
                and angle_deg > self.get_actual_turret_yaw()
            ):
                print("Right limit switch triggered, preventing further right movement")
            else:
                self.turret_motor.set_control(
                    self._motion_magic_position_voltage_turret.with_position(
                        SI.degrees_to_rotations * angle_deg
                    )
                )

    def clear_robot_heading_lock(self):
        self.driveSub.changeAng = False
        self.driveSub.target_field_heading = Rotation2d(0)

    def reset_target_hood_and_turret(self):
        self.set_angle_hood(0)
        self.clear_robot_heading_lock()
        return 0, 0, 0

    def get_actual_hood_angle_cancoder(self) -> float:
        """Returns the absolute hood angle in degrees from the CANcoder."""
        return (
            float(self.hood_cancoder.get_position().value)
            / TurretConstants.HOOD_MECHANICAL_RATIO
            * SI.rotations_to_degrees
        )

    def set_target_hood_and_turret(self, shooting_location=Hub.TOP_CENTER_POINT):
        self.turret_yaw_deg = Rotation2d(0)
        v_fixed, hood_angle_deg, turret_yaw_deg = self._optimal_angle_calc(
            TurretConstants.SHOOTER_SET_VELOCITY_CONSTANT, shooting_location
        )
        # print(v_fixed, hood_angle_deg, self.turret_yaw_deg)
        if v_fixed < 0:
            return -1, -1, 0
        else:
            pass
        self.turret_yaw_deg = turret_yaw_deg
        print(v_fixed, hood_angle_deg, self.turret_yaw_deg)
        # if hood_angle_deg > 80:
        #     hood_angle_deg = 80
        # elif hood_angle_deg < -80:
        #     hood_angle_deg = -80
        self.set_angle_hood(hood_angle_deg)
        self.set_angle_turret(self.turret_yaw_deg)
        return v_fixed, hood_angle_deg, self.turret_yaw_deg

    # def get_actual_turret_yaw(self):
    #     return float(self.turret_motor.get_position().value) * SI.rotations_to_degrees

    def periodic(self):
        self._loop_timer.start()

        # actual_turret_yaw = (
        #     float(self.turret_motor.get_position().value) * SI.rotations_to_degrees
        # )
        actual_hood_angle = (
            float(self.hood_motor.get_position().value) * SI.rotations_to_degrees
        )

        # Update Mechanism2d
        if RobotFeatures.LOGGING_ROBOT:
            robot_pose = self.driveSub.get_pose()
            if robot_pose is not None:
                self.base_ligament.setAngle(robot_pose.rotation().degrees())
            # self.turret_ligament.setAngle(actual_turret_yaw)
            # Hood angle controls turret length — 0° = min length, 90° = max length
            hood_fraction = max(0, min(actual_hood_angle, 90)) / 90.0
            self.turret_ligament.setLength(0.3 + hood_fraction * 0.9)

        Logger.recordOutput(
            "Turret/target_hood_angle", float(self.target_hood_angle)
        )
        Logger.recordOutput(
            "Turret/actual_hood_angle_motor",
            float(self.hood_motor.get_position().value) * SI.rotations_to_degrees,
        )
        Logger.recordOutput(
            "Turret/actual_hood_angle_cancoder",
            self.get_actual_hood_angle_cancoder(),
        )
        if self.turret_yaw_deg != Rotation2d(0):
            Logger.recordOutput(
                "Turret/turret_yaw_deg", float(self.turret_yaw_deg)
            )
            self.turret_yaw_deg = Rotation2d(0)
        if RobotFeatures.LOGGING_TURRET:
            Logger.recordOutput(
                "Turret/hood_motor_voltage",
                float(self.hood_motor.get_motor_voltage().value_as_double),
            )

        self._loop_timer.stop()

    def _optimal_angle_calc(
        self,
        v_fixed: float = TurretConstants.SHOOTER_SET_VELOCITY_CONSTANT,
        shooting_location=Hub.TOP_CENTER_POINT,
    ):
        """
        Args:
            v_fixed: Should be ~10m/s this season, but can be tuned for optimal performance

        Returns:
            v_fixed (m/s): the fixed velocity input, returned for convenience.
            hood_angle (degree): in UNIT CIRCLE convention. Rest = 140°, full travel = 110°.
                Physically: 140° = 50° above horizontal, 110° = 20° above horizontal.
            turret_yaw (degree): robot-relative. 0° = straight ahead, -left, +right. Clamped to [-90, 90]
        """

        # --- Physical hood limits (elevation from horizontal) ---
        # Unit circle 140° -> 90° offset -> 50° elevation (rest/start)
        # Unit circle 110° -> 90° offset -> 20° elevation (max travel)
        # HOOD_UNIT_CIRCLE_REST = 140.0  # degrees, rest position
        # HOOD_UNIT_CIRCLE_FULL = 110.0  # degrees, full travel
        HOOD_ELEV_MAX_DEG = 45.0  # elevation at rest   (140 - 90)
        HOOD_ELEV_MIN_DEG = 20.0  # elevation at full   (110 - 90)

        YAW_MIN_DEG = -180.0
        YAW_MAX_DEG = 180.0

        robot_pose = self.driveSub.get_pose()
        robot_speeds = self.driveSub.get_speeds()

        if robot_pose is None and RobotFeatures.LOGGING_TURRET:
            Logger.recordOutput("Turret/calc_valid", False)
            Logger.recordOutput("Turret/calc_failure_reason", "robot_pose_is_none")
            return -1, -1, 0

        G = 9.81
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            hub_pose = get_red_pose(shooting_location.toTranslation2d())
            hub_pose = Translation3d(hub_pose.X(), hub_pose.Y(), shooting_location.Z())
        else:
            hub_pose = shooting_location

        if hub_pose is None and RobotFeatures.LOGGING_TURRET:
            Logger.recordOutput("Turret/calc_valid", False)
            Logger.recordOutput("Turret/calc_failure_reason", "hub_pose_is_none")
            return -1, -1, 0

        dx = hub_pose.X() - robot_pose.X()
        dy = hub_pose.Y() - robot_pose.Y()
        distance = math.sqrt(dx * dx + dy * dy) 
        Logger.recordOutput("Turret/initial_distance_to_hub", float(distance))

        v_fixed = 10
        self.actual_velocity_to_go = 2 * distance + 10


        # if distance > 3 and distance < 3.5:
        #     v_fixed = 10
        #     self.actual_velocity_to_go = 16.5
        # elif distance > 2.5 and distance < 3:
        #     v_fixed = 10
        #     self.actual_velocity_to_go = 15.5
        # elif distance > 2 and distance < 2.5:
        #     v_fixed = 10
        #     self.actual_velocity_to_go = 14.5
        # else:
        #     v_fixed = SmartDashboard.getNumber("Turret/SHOOTER_SET_VELOCITY_CONSTANT", v_fixed)
        #     self.actual_velocity_to_go = SmartDashboard.getNumber("CustomFloatVelocity", v_fixed)

        dz = hub_pose.Z() - TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M
        if RobotFeatures.LOGGING_TURRET:
            Logger.recordOutput("Turret/initial_dx", float(dx))
            Logger.recordOutput("Turret/initial_dy", float(dy))
            Logger.recordOutput("Turret/initial_dz", float(dz))
            Logger.recordOutput("Turret/v_fixed", float(v_fixed))

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
            if RobotFeatures.LOGGING_TURRET:
                Logger.recordOutput(f"Turret/Iterations/iter_{i}_r", float(r))
                Logger.recordOutput(
                    f"Turret/Iterations/iter_{i}_discriminant", float(discriminant)
                )

            if discriminant < 0:
                if RobotFeatures.LOGGING_TURRET:
                    Logger.recordOutput("Turret/calc_valid", False)
                    Logger.recordOutput(
                        "Turret/calc_failure_reason",
                        "target_unreachable_discriminant_negative",
                    )
                    Logger.recordOutput("Turret/discriminant", float(discriminant))
                    Logger.recordOutput("Turret/horizontal_distance", float(r))
                    Logger.recordOutput(
                        f"Turret/Iterations/iter_{i}_hood_angle_deg", 0.0
                    )
                    Logger.recordOutput(f"Turret/Iterations/iter_{i}_tof", 0.0)
                return -1, -1, 0

            tan_theta = (-B - math.sqrt(discriminant)) / (2 * A)
            hood_angle = math.atan(tan_theta)  # elevation from horizontal, radians
            tof = r / (math.cos(hood_angle) * v_fixed)
            if RobotFeatures.LOGGING_TURRET:
                Logger.recordOutput(
                    f"Turret/Iterations/iter_{i}_hood_angle_deg",
                    float(hood_angle * SI.radians_to_degrees),
                )
                Logger.recordOutput(f"Turret/Iterations/iter_{i}_tof", float(tof))

            dx = hub_pose.X() - robot_pose.X() - robot_speeds.vx * tof
            dy = hub_pose.Y() - robot_pose.Y() - robot_speeds.vy * tof

        # --- Yaw ---
        field_angle = math.atan2(dy, dx)
        turret_yaw = field_angle - robot_pose.rotation().radians()
        turret_yaw = (turret_yaw + math.pi) % (2 * math.pi) - math.pi  # wrap to [-π, π]
        turret_yaw_deg = turret_yaw * SI.radians_to_degrees

        # --- Hood: solver gives elevation from horizontal in degrees ---
        hood_elev_deg = hood_angle * SI.radians_to_degrees

        # --- Enforce physical limits ---

        # Hard fail if yaw is unreachable
        if not (YAW_MIN_DEG <= turret_yaw_deg <= YAW_MAX_DEG):
            if RobotFeatures.LOGGING_TURRET:
                Logger.recordOutput("Turret/calc_valid", False)
                Logger.recordOutput(
                    "Turret/calc_failure_reason",
                    f"turret_yaw_out_of_range: {turret_yaw_deg:.1f} deg",
                )
                Logger.recordOutput(
                    "Turret/target_turret_yaw", float(turret_yaw_deg)
                )
            return -1, -1, 0

        # Clamp hood elevation to physical travel range [20°, 50°]
        hood_elev_clamped = max(
            HOOD_ELEV_MIN_DEG, min(HOOD_ELEV_MAX_DEG, hood_elev_deg)
        )
        if hood_elev_clamped != hood_elev_deg:
            if RobotFeatures.LOGGING_TURRET:
                Logger.recordOutput(
                    "Turret/calc_warning",
                    f"hood_elevation_clamped from {hood_elev_deg:.1f} to {hood_elev_clamped:.1f} deg",
                )
        # Convert clamped elevation back to unit circle convention for output
        # unit_circle_deg = elevation_deg + 90°
        hood_unit_circle_deg = hood_elev_clamped + 90.0
        hood_motor_deg = hood_unit_circle_deg - 140.0  # [-30, 0]
        if RobotFeatures.LOGGING_TURRET:
            Logger.recordOutput("Turret/calc_valid", True)
            Logger.recordOutput("Turret/calc_failure_reason", "")
            Logger.recordOutput(
                "Turret/hub_dx", float(hub_pose.X() - robot_pose.X())
            )
            Logger.recordOutput(
                "Turret/hub_dy", float(hub_pose.Y() - robot_pose.Y())
            )
            Logger.recordOutput(
                "Turret/hub_dz",
                float(hub_pose.Z() - TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M),
            )
            Logger.recordOutput("Turret/horizontal_distance", float(r))
            Logger.recordOutput("Turret/time_of_flight", float(tof))
            Logger.recordOutput("Turret/discriminant", float(discriminant))
            Logger.recordOutput(
                "Turret/target_hood_angle_elevation_deg", float(hood_elev_clamped)
            )
            Logger.recordOutput(
                "Turret/target_hood_angle_unit_circle_deg", float(hood_unit_circle_deg)
            )
            Logger.recordOutput(
                "Turret/target_hood_angle_motor_deg", float(hood_motor_deg)
            )
        # print("hub Z:", hub_pose.Z())
        # print("shooter height:", TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M)
        # print("dz:", hub_pose.Z() - TurretConstants.SHOOTER_HEIGHT_FOR_FUEL_M)

        return v_fixed, -hood_motor_deg, -turret_yaw_deg

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
    def enableAutoAim(self, enable: bool):
        if enable:
            if not self.driveSub.allow_center_auto_align:
                self.set_target_hood_and_turret()
                target_pose = Hub.TOP_CENTER_POINT
            else:
                pose = self.driveSub.get_pose()
                target_pose = pose.nearest(
                    [
                        Pose2d(
                            CustomPoints.TARGET_POSE_SHOOT.toTranslation2d(), Rotation2d(0)
                        ),
                        Pose2d(
                            CustomPoints.TARGET_POSE_SHOOT_OTHER_SIDE.toTranslation2d(),
                            Rotation2d(0),
                        ),
                    ]
                )
                target_pose = Translation3d(target_pose.X(), target_pose.Y(), 0)
                self.set_target_hood_and_turret(shooting_location=target_pose)


            if self.turret_yaw_deg == Rotation2d(0):
                self.driveSub.changeAngTelop = False
            self.driveSub.changeAngTelop = enable
            Logger.recordOutput("Turret/target_pose", target_pose)
        else:
            self.reset_target_hood_and_turret()
            self.driveSub.changeAngTelop = enable
