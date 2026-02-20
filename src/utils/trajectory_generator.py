"""
Python port of TOPPGenerator from FRC 4481 - Team Rembrandts
Original: https://github.com/FRC-4481-Team-Rembrandts
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from constants import VisionConstants, DriveConstants


@dataclass
class ChassisAccelerations:
    ax: float
    ay: float
    alpha: float


class ChassisState:
    def __init__(self, vx=0.0, vy=0.0, omega=0.0, ax=0.0, ay=0.0, alpha=0.0):
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.ax = ax
        self.ay = ay
        self.alpha = alpha
        self.speeds = ChassisSpeeds(vx, vy, omega)
        self.accelerations = ChassisAccelerations(ax, ay, alpha)


@dataclass
class Sample:
    time: float
    pose: Pose2d
    chassis_state: ChassisState


class NativeHolonomicTrajectory:
    def __init__(self, samples: List[Sample]):
        self.samples = samples

    def total_time(self) -> float:
        return self.samples[-1].time if self.samples else 0.0

    def sample_at(self, t: float) -> Sample:
        """Linear interpolation between samples at time t."""
        if t <= self.samples[0].time:
            return self.samples[0]
        if t >= self.samples[-1].time:
            return self.samples[-1]
        for i in range(len(self.samples) - 1):
            s0 = self.samples[i]
            s1 = self.samples[i + 1]
            if s0.time <= t <= s1.time:
                dt = s1.time - s0.time
                a = (t - s0.time) / dt if dt > 0 else 0.0
                x = s0.pose.X() + a * (s1.pose.X() - s0.pose.X())
                y = s0.pose.Y() + a * (s1.pose.Y() - s0.pose.Y())
                r = s0.pose.rotation().radians() + a * (
                    s1.pose.rotation().radians() - s0.pose.rotation().radians()
                )
                pose = Pose2d(x, y, Rotation2d(r))
                sp0, sp1 = s0.chassis_state.speeds, s1.chassis_state.speeds
                ac0, ac1 = (
                    s0.chassis_state.accelerations,
                    s1.chassis_state.accelerations,
                )
                cs = ChassisState(
                    vx=sp0.vx + a * (sp1.vx - sp0.vx),
                    vy=sp0.vy + a * (sp1.vy - sp0.vy),
                    omega=sp0.omega + a * (sp1.omega - sp0.omega),
                    ax=ac0.ax + a * (ac1.ax - ac0.ax),
                    ay=ac0.ay + a * (ac1.ay - ac0.ay),
                    alpha=ac0.alpha + a * (ac1.alpha - ac0.alpha),
                )
                return Sample(t, pose, cs)
        return self.samples[-1]


# ---------------------------------------------------------------------------
# Bezier math helpers
# ---------------------------------------------------------------------------


def _cubic_bezier(
    t: float, p0: Translation2d, p1: Translation2d, p2: Translation2d, p3: Translation2d
) -> Translation2d:
    mt = 1.0 - t
    x = mt**3 * p0.x + 3 * mt**2 * t * p1.x + 3 * mt * t**2 * p2.x + t**3 * p3.x
    y = mt**3 * p0.y + 3 * mt**2 * t * p1.y + 3 * mt * t**2 * p2.y + t**3 * p3.y
    return Translation2d(x, y)


def _cubic_bezier_velocity(
    t: float, p0: Translation2d, p1: Translation2d, p2: Translation2d, p3: Translation2d
) -> Translation2d:
    mt = 1.0 - t
    x = 3 * (mt**2 * (p1.x - p0.x) + 2 * mt * t * (p2.x - p1.x) + t**2 * (p3.x - p2.x))
    y = 3 * (mt**2 * (p1.y - p0.y) + 2 * mt * t * (p2.y - p1.y) + t**2 * (p3.y - p2.y))
    return Translation2d(x, y)


def _cubic_bezier_acceleration(
    t: float, p0: Translation2d, p1: Translation2d, p2: Translation2d, p3: Translation2d
) -> Translation2d:
    mt = 1.0 - t
    x = 6 * (mt * (p2.x - 2 * p1.x + p0.x) + t * (p3.x - 2 * p2.x + p1.x))
    y = 6 * (mt * (p2.y - 2 * p1.y + p0.y) + t * (p3.y - 2 * p2.y + p1.y))
    return Translation2d(x, y)


def _dot(a: list, b: list) -> float:
    return sum(ai * bi for ai, bi in zip(a, b))


def _scale(v: list, s: float) -> list:
    return [vi * s for vi in v]


def _add(a: list, b: list) -> list:
    return [ai + bi for ai, bi in zip(a, b)]


# ---------------------------------------------------------------------------
# Rotation helpers (smooth cubic interpolation with dead zones)
# ---------------------------------------------------------------------------

_NO_ROT_THRESHOLD = 0.9
_START_ROT_THRESHOLD = 0.1


def _calculate_rotation(s: float, start: Rotation2d, end: Rotation2d) -> Rotation2d:
    if s > _NO_ROT_THRESHOLD:
        return end
    if s < _START_ROT_THRESHOLD:
        return start
    s = (s - _START_ROT_THRESHOLD) / (_NO_ROT_THRESHOLD - _START_ROT_THRESHOLD)

    end_r = end.radians()
    if abs(start.radians() - end_r) > math.pi:
        end_r = (end + Rotation2d(math.pi)).radians()

    diff = end_r - start.radians()
    r = -2 * diff * s**3 + 3 * diff * s**2 + start.radians()
    return Rotation2d(r)


def _calculate_rotation_prime(
    s: float, start: Rotation2d, end: Rotation2d
) -> Rotation2d:
    if s > _NO_ROT_THRESHOLD or s < _START_ROT_THRESHOLD:
        return Rotation2d(0)
    s = (s - _START_ROT_THRESHOLD) / (_NO_ROT_THRESHOLD - _START_ROT_THRESHOLD)

    end_r = end.radians()
    if abs(start.radians() - end_r) > math.pi:
        end_r = (end + Rotation2d(math.pi)).radians()

    diff = end_r - start.radians()
    r = -6 * diff * s**2 + 6 * diff * s
    return Rotation2d(r)


def _calculate_rotation_double_prime(
    s: float, start: Rotation2d, end: Rotation2d
) -> Rotation2d:
    if s > _NO_ROT_THRESHOLD or s < _START_ROT_THRESHOLD:
        return Rotation2d(0)
    s = (s - _START_ROT_THRESHOLD) / (_NO_ROT_THRESHOLD - _START_ROT_THRESHOLD)

    end_r = end.radians()
    if abs(start.radians() - end_r) > math.pi:
        end_r = (end + Rotation2d(math.pi)).radians()

    diff = end_r - start.radians()
    r = -12 * diff * s + 6 * diff
    return Rotation2d(r)


class TOPPGenerator:
    EPSILON = 1e-6
    N = 81
    DS = 1.0 / (N - 1)
    SCAN_LENGTH = 14

    MIN_ROBOT_SPEED = 0.4
    END_CONTROL_POINT_LENGTH = 0.8
    VELOCITY_TO_CONTROL_POINT_FACTOR = 0.5
    AWAY_FROM_TARGET_ANGLE_THRESHOLD = 100.0  # degrees
    AWAY_FROM_TARGET_FACTOR = 2.0

    _s_values: list = [i / (81 - 1) for i in range(N)]

    def __init__(
        self,
        mass: float,
        moi: float,
        wheel_base: float,
        robot_pose_supplier: Callable[[], Pose2d],
        chassis_speed_supplier: Callable[[], ChassisSpeeds],
    ):
        robot_radius = math.sqrt(2) * wheel_base
        self._m_diag = [mass, mass, moi / robot_radius]
        self._pose_supplier = robot_pose_supplier
        self._speed_supplier = chassis_speed_supplier

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate(
        self,
        target_position: Pose2d,
        target_heading: Rotation2d,
        v_max: float,
        a_max: float,
    ) -> NativeHolonomicTrajectory:
        speeds = self._speed_supplier()()
        robot_speed = math.hypot(speeds.vx, speeds.vy)
        current_pose = self._pose_supplier()()

        direct_to_target = (
            target_position.translation() - current_pose.translation()
        ).angle()

        velocity_direction = (
            Rotation2d(speeds.vx, speeds.vy)
            if robot_speed > self.MIN_ROBOT_SPEED
            else direct_to_target
        )

        end_control_point_rotation = target_heading + Rotation2d(math.pi)

        start_cp_length = max(
            self.END_CONTROL_POINT_LENGTH,
            robot_speed * self.VELOCITY_TO_CONTROL_POINT_FACTOR,
        )

        distance_to_target = target_position.translation().distance(
            current_pose.translation()
        )
        end_cp_length = min(self.END_CONTROL_POINT_LENGTH, distance_to_target * 0.4)

        angle_diff = abs(
            _angle_modulus((direct_to_target - velocity_direction).radians())
        )
        if (
            angle_diff > math.radians(self.AWAY_FROM_TARGET_ANGLE_THRESHOLD)
            and robot_speed > self.MIN_ROBOT_SPEED
        ):
            start_cp_length *= self.AWAY_FROM_TARGET_FACTOR
        else:
            start_cp_length = min(start_cp_length, distance_to_target * 0.4)

        q, q_prime, q_double_prime = self._generate_path_points(
            target_position,
            current_pose,
            start_cp_length,
            end_cp_length,
            velocity_direction,
            end_control_point_rotation,
        )

        f_max = a_max * self._m_diag[0]
        qp_start_mag_sq = q_prime[0][0] ** 2 + q_prime[0][1] ** 2
        x0 = robot_speed**2 / qp_start_mag_sq if qp_start_mag_sq > 0 else 0.0

        x_max = self._backward_pass(q, q_prime, q_double_prime, v_max, f_max, 0.0)
        x, u = self._forward_pass(q, q_prime, q_double_prime, x_max, f_max, x0)

        return NativeHolonomicTrajectory(
            self._forward_pass_to_samples(q, q_prime, q_double_prime, x, u)
        )

    # ------------------------------------------------------------------
    # Path generation
    # ------------------------------------------------------------------

    def _generate_path_points(
        self,
        end_pose,
        start_pose,
        start_cp_length,
        end_cp_length,
        start_cp_rotation,
        end_cp_rotation,
    ):
        start_cp = (
            Translation2d(start_cp_length, start_cp_rotation) + start_pose.translation()
        )
        end_cp = Translation2d(end_cp_length, end_cp_rotation) + end_pose.translation()
        p0, p1, p2, p3 = (
            start_pose.translation(),
            start_cp,
            end_cp,
            end_pose.translation(),
        )
        start_rot, end_rot = start_pose.rotation(), end_pose.rotation()

        N = self.N
        q = [[0.0, 0.0, 0.0] for _ in range(N)]
        q_prime = [[0.0, 0.0, 0.0] for _ in range(N)]
        q_double_prime = [[0.0, 0.0, 0.0] for _ in range(N)]

        for i, s in enumerate(self._s_values):
            pt = _cubic_bezier(s, p0, p1, p2, p3)
            vt = _cubic_bezier_velocity(s, p0, p1, p2, p3)
            at = _cubic_bezier_acceleration(s, p0, p1, p2, p3)

            q[i] = [pt.x, pt.y, _calculate_rotation(s, start_rot, end_rot).radians()]
            q_prime[i] = [
                vt.x,
                vt.y,
                _calculate_rotation_prime(s, start_rot, end_rot).radians(),
            ]
            q_double_prime[i] = [
                at.x,
                at.y,
                _calculate_rotation_double_prime(s, start_rot, end_rot).radians(),
            ]

        return q, q_prime, q_double_prime

    # ------------------------------------------------------------------
    # Backward pass (binary search)
    # ------------------------------------------------------------------

    def _backward_pass(self, q, q_prime, q_double_prime, v_max, f_max, x_end):
        N = self.N
        DS = self.DS
        x_max = [0.0] * N
        x_max[N - 1] = x_end

        for i in range(N - 2, -1, -1):
            qp_dot = _dot(q_prime[i], q_prime[i])
            x_low = 0.0
            x_high = (v_max**2 / qp_dot) if qp_dot > self.EPSILON else 0.0
            x = 0.0
            for _ in range(self.SCAN_LENGTH):
                x = (x_low + x_high) / 2
                u_min = self._u_min_feasible(q_prime[i], q_double_prime[i], x, f_max)
                x_next_min = x + 2 * DS * u_min
                if x_next_min <= x_max[i + 1]:
                    x_low = x  # valid, try higher
                else:
                    x_high = x  # too high, try lower
            x_max[i] = x

        return x_max

    # ------------------------------------------------------------------
    # Forward pass (binary search)
    # ------------------------------------------------------------------

    def _forward_pass(self, q, q_prime, q_double_prime, x_max, f_max, x0):
        N = self.N
        DS = self.DS
        x = [0.0] * N
        u = [0.0] * N

        if x0 > x_max[0]:
            print(
                f"Warning: Starting path velocity {x0:.3f} exceeds maximum feasible velocity."
            )
        x[0] = min(x0, x_max[0])

        for i in range(1, N):
            x_low = 0.0
            x_high = x_max[i]
            x_i = 0.0
            u_required = 0.0
            for _ in range(self.SCAN_LENGTH):
                x_i = (x_low + x_high) / 2
                u_required = (x_i - x[i - 1]) / (2 * DS)
                force_vec, force_grad = self._get_force_vectors(
                    q_prime[i - 1], q_double_prime[i - 1], x_i, x[i - 1]
                )
                force_sq = _dot(force_vec, force_vec)
                grad_dot = _dot(force_vec, force_grad)

                if force_sq <= f_max**2:
                    x_low = x_i  # valid, try higher
                elif grad_dot >= 0:
                    x_low = x_i  # highest valid x is above current
                else:
                    x_high = x_i  # highest valid x is below current

            x[i] = x_i
            u[i] = u_required

        return x, u

    # ------------------------------------------------------------------
    # Convert to samples
    # ------------------------------------------------------------------

    @staticmethod
    def _forward_pass_to_samples(q, q_prime, q_double_prime, x, u) -> List[Sample]:
        N = TOPPGenerator.N
        DS = TOPPGenerator.DS
        EPSILON = TOPPGenerator.EPSILON

        samples = []
        time = [0.0] * N

        for i in range(N):
            if i > 0:
                v1 = math.sqrt(max(x[i - 1], 0.0))
                v2 = math.sqrt(max(x[i], 0.0))
                v_avg = (v1 + v2) / 2.0
                dt = DS / v_avg if v_avg > EPSILON else 0.0
                time[i] = time[i - 1] + dt

            vel = _scale(q_prime[i], math.sqrt(max(x[i], 0.0)))
            acc = _add(_scale(q_double_prime[i], x[i]), _scale(q_prime[i], u[i]))

            pose = Pose2d(q[i][0], q[i][1], Rotation2d(q[i][2]))
            cs = ChassisState(
                vx=vel[0],
                vy=vel[1],
                omega=vel[2],
                ax=acc[0],
                ay=acc[1],
                alpha=acc[2],
            )
            samples.append(Sample(time[i], pose, cs))

        return samples

    # ------------------------------------------------------------------
    # Constraint helpers
    # ------------------------------------------------------------------

    def _u_min_feasible(self, q_prime, q_double_prime, x, f_max) -> float:
        A = B = 0.0
        C = -(f_max**2)
        for i in range(3):
            a_i = self._m_diag[i] * q_prime[i]
            b_i = self._m_diag[i] * q_double_prime[i] * x
            A += a_i * a_i
            B += 2 * a_i * b_i
            C += b_i * b_i

        D = B * B - 4 * A * C
        if D < 0:
            return float("inf")
        return (-B - math.sqrt(D)) / (2 * A)

    def _get_force_vectors(self, q_prime, q_double_prime, x, x_prev):
        DS = self.DS
        u_required = (x - x_prev) / (2 * DS)
        force_vec = [0.0, 0.0, 0.0]
        force_grad = [0.0, 0.0, 0.0]
        for i in range(3):
            a = self._m_diag[i] * q_prime[i]
            b = self._m_diag[i] * q_double_prime[i]
            force_vec[i] = a * u_required + b * x_prev
            force_grad[i] = -a / (2 * DS)
        return force_vec, force_grad


def _angle_modulus(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class CreateTrajectory:
    def __init__(
        self,
        robot_pose_supplier: Callable[[], Pose2d],
        chassis_speed_supplier: Callable[[], ChassisSpeeds],
    ):
        self.generator = TOPPGenerator(
            mass=DriveConstants.ROBOT_MASS_KG,
            moi=DriveConstants.ROBOT_MOI,
            wheel_base=DriveConstants.CENTER_WHEEL_TO_CENTER_WHEEL_METERS,
            robot_pose_supplier=robot_pose_supplier,
            chassis_speed_supplier=chassis_speed_supplier,
        )

    def get_trajectory(
        self, target_position: Pose2d, target_heading: Rotation2d
    ) -> NativeHolonomicTrajectory:
        # target_heading = the direction the robot is travelling (its velocity vector) as it arrives
        # target_position = this is the robot's facing direction at the target
        trajectory = self.generator.generate(
            target_position=target_position,
            target_heading=target_heading,
            v_max=VisionConstants.AUTO_ALIGN_VELOCITY_CONSTANT,
            a_max=VisionConstants.AUTO_ALIGN_ACCELERATION_CONSTANT,
        )

        return trajectory


if __name__ == "__main__":
    # Stub suppliers — replace with your real drivetrain references
    current_pose = Pose2d(0, 0, Rotation2d(0))
    current_speeds = ChassisSpeeds(0, 0, 0)

    generator = TOPPGenerator(
        mass=50.0,
        moi=6.0,
        wheel_base=0.55,
        robot_pose_supplier=lambda: current_pose,
        chassis_speed_supplier=lambda: current_speeds,
    )

    trajectory = generator.generate(
        target_position=Pose2d(3.0, 2.0, Rotation2d(0)),
        target_heading=Rotation2d.fromDegrees(90),
        v_max=4.0,
        a_max=3.0,
    )

    print(f"Trajectory generated: {len(trajectory.samples)} samples")
    print(f"Total time: {trajectory.total_time():.3f}s")
    sample = trajectory.sample_at(0.5)
    print(f"At t=0.5s: x={sample.pose.X():.3f}, y={sample.pose.Y():.3f}")
