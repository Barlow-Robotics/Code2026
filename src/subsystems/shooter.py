import wpilib
from commands2 import Subsystem
from subsystems import Drivetrain
from constants import Hub
import math
from constants import ShooterConstants
from constants import SI


class Shooter(Subsystem):
    def __init__(self, driveSub: Drivetrain):
        super().__init__()
        self.driveSub = driveSub

    def set_velocity(self, velocity_rpm: float):
        """Set target velocity in RPM"""
        self.target_velocity = velocity_rpm

    def periodic(self):
        """This method will be called once per scheduler run"""
        print(self.optimal_angle_calc(9.5))
        wpilib.SmartDashboard.putNumber("Shooter/Target", self.target_velocity)

    def optimal_angle_calc(self, v_fixed: float = 9.5):
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
            return -1, -1, -1
        G = 9.81
        hub_pose = Hub.TOP_CENTER_POINT

        dx = hub_pose.X() - robot_pose.X()
        dy = hub_pose.Y() - robot_pose.Y()
        dz = hub_pose.Z() - ShooterConstants.SHOOTER_HEIGHT_FOR_FUEL_M

        for _ in range(3):
            r = math.sqrt(dx * dx + dy * dy)

            # Quadratic in tan(theta)
            A = (G * r**2) / (2 * v_fixed**2)
            B = -r
            C = dz + A

            discriminant = B**2 - 4 * A * C
            if discriminant < 0:
                return None  # target unreachable at this velocity

            # Two solutions — take the lower angle (flatter shot)
            tan_theta = (-B - math.sqrt(discriminant)) / (2 * A)
            hood_angle = math.atan(tan_theta)

            tof = r / (math.cos(hood_angle) * v_fixed)

            dx = hub_pose.X() - robot_pose.X() - robot_speeds.vx * tof
            dy = hub_pose.Y() - robot_pose.Y() - robot_speeds.vy * tof

        field_angle = math.atan2(dy, dx)
        turret_yaw = field_angle - robot_pose.rotation().radians()

        turret_yaw = (turret_yaw + math.pi) % (2 * math.pi) - math.pi
        hood_angle = hood_angle * SI.radians_to_degrees
        turret_yaw = turret_yaw * SI.radians_to_degrees
        return v_fixed, hood_angle, turret_yaw

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
