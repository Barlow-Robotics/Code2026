from commands2 import Subsystem
from subsystems import Drivetrain
from constants import Hub
import math
from constants import ShooterConstants
from constants import SI
from utils import Logger
from dataclasses import dataclass


@dataclass
class ShooterIterationTelemetry:
    iter_0_r: float
    iter_0_discriminant: float
    iter_0_hood_angle_deg: float
    iter_0_tof: float
    iter_1_r: float
    iter_1_discriminant: float
    iter_1_hood_angle_deg: float
    iter_1_tof: float
    iter_2_r: float
    iter_2_discriminant: float
    iter_2_hood_angle_deg: float
    iter_2_tof: float


@dataclass
class ShooterTelemetry:
    target_velocity: float
    calc_valid: bool
    calc_failure_reason: str
    v_fixed: float
    hood_angle: float
    turret_yaw: float
    hub_dx: float
    hub_dy: float
    hub_dz: float
    horizontal_distance: float
    time_of_flight: float
    discriminant: float
    initial_dx: float
    initial_dy: float
    initial_dz: float


class Shooter(Subsystem):
    def __init__(self, driveSub: Drivetrain):
        super().__init__()
        self.driveSub = driveSub
        self.log = Logger("Shooter")
        self.target_velocity = 0.0

        self._telemetry = ShooterTelemetry(
            target_velocity=0.0,
            calc_valid=False,
            calc_failure_reason="not_yet_run",
            v_fixed=0.0,
            hood_angle=0.0,
            turret_yaw=0.0,
            hub_dx=0.0,
            hub_dy=0.0,
            hub_dz=0.0,
            horizontal_distance=0.0,
            time_of_flight=0.0,
            discriminant=0.0,
            initial_dx=0.0,
            initial_dy=0.0,
            initial_dz=0.0,
        )

        self._iteration_telemetry = ShooterIterationTelemetry(
            iter_0_r=0.0,
            iter_0_discriminant=0.0,
            iter_0_hood_angle_deg=0.0,
            iter_0_tof=0.0,
            iter_1_r=0.0,
            iter_1_discriminant=0.0,
            iter_1_hood_angle_deg=0.0,
            iter_1_tof=0.0,
            iter_2_r=0.0,
            iter_2_discriminant=0.0,
            iter_2_hood_angle_deg=0.0,
            iter_2_tof=0.0,
        )

    def set_velocity(self, velocity_rpm: float):
        """Set target velocity in RPM"""
        self.target_velocity = velocity_rpm

    def periodic(self):
        """This method will be called once per scheduler run"""
        self._optimal_angle_calc(9.5)

        self._telemetry.target_velocity = float(self.target_velocity)
        self.log.publish(self._telemetry)
        self.log.child("Iterations").publish(self._iteration_telemetry)

    def _optimal_angle_calc(self, v_fixed: float = 9.5):
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
            self._telemetry.calc_valid = False
            self._telemetry.calc_failure_reason = "robot_pose_is_none"
            return -1, -1, -1

        G = 9.81
        hub_pose = Hub.TOP_CENTER_POINT

        dx = hub_pose.X() - robot_pose.X()
        dy = hub_pose.Y() - robot_pose.Y()
        dz = hub_pose.Z() - ShooterConstants.SHOOTER_HEIGHT_FOR_FUEL_M

        self._telemetry.initial_dx = float(dx)
        self._telemetry.initial_dy = float(dy)
        self._telemetry.initial_dz = float(dz)
        self._telemetry.v_fixed = float(v_fixed)

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

            setattr(self._iteration_telemetry, f"iter_{i}_r", float(r))
            setattr(
                self._iteration_telemetry, f"iter_{i}_discriminant", float(discriminant)
            )

            if discriminant < 0:
                self._telemetry.calc_valid = False
                self._telemetry.calc_failure_reason = (
                    "target_unreachable_discriminant_negative"
                )
                self._telemetry.discriminant = float(discriminant)
                self._telemetry.horizontal_distance = float(r)
                setattr(self._iteration_telemetry, f"iter_{i}_hood_angle_deg", 0.0)
                setattr(self._iteration_telemetry, f"iter_{i}_tof", 0.0)
                return None

            tan_theta = (-B - math.sqrt(discriminant)) / (2 * A)
            hood_angle = math.atan(tan_theta)
            tof = r / (math.cos(hood_angle) * v_fixed)

            setattr(
                self._iteration_telemetry,
                f"iter_{i}_hood_angle_deg",
                float(hood_angle * SI.radians_to_degrees),
            )
            setattr(self._iteration_telemetry, f"iter_{i}_tof", float(tof))

            dx = hub_pose.X() - robot_pose.X() - robot_speeds.vx * tof
            dy = hub_pose.Y() - robot_pose.Y() - robot_speeds.vy * tof

        field_angle = math.atan2(dy, dx)
        turret_yaw = field_angle - robot_pose.rotation().radians()

        turret_yaw = (turret_yaw + math.pi) % (2 * math.pi) - math.pi
        hood_angle_deg = hood_angle * SI.radians_to_degrees
        turret_yaw_deg = turret_yaw * SI.radians_to_degrees

        self._telemetry.calc_valid = True
        self._telemetry.calc_failure_reason = ""
        self._telemetry.hood_angle = float(hood_angle_deg)
        self._telemetry.turret_yaw = float(turret_yaw_deg)
        self._telemetry.hub_dx = float(hub_pose.X() - robot_pose.X())
        self._telemetry.hub_dy = float(hub_pose.Y() - robot_pose.Y())
        self._telemetry.hub_dz = float(
            hub_pose.Z() - ShooterConstants.SHOOTER_HEIGHT_FOR_FUEL_M
        )
        self._telemetry.horizontal_distance = float(r)
        self._telemetry.time_of_flight = float(tof)
        self._telemetry.discriminant = float(discriminant)

        return v_fixed, hood_angle_deg, turret_yaw_deg

    @staticmethod
    def find_v_fixed():
        G = 9.81
        height_needed = 1.8288
        height_of_shooter = 0.5224  # meters
        dz = height_needed - height_of_shooter

        r_min = 0.6
        r_max = 5.549110139617

        results = []
        for v_tenth in range(10, 150):  # 1.0 to 15.0 m/s
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
