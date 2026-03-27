"""
FieldCentricFacingAngle variant that uses operator perspective for
velocities but always treats target_direction as field-absolute
(blue-alliance origin).
"""

from phoenix6 import swerve


class FieldCentricFacingAngleAbsolute(swerve.requests.FieldCentricFacingAngle):
    """FieldCentricFacingAngle where velocities follow operator perspective
    but target_direction is always field-absolute (no alliance rotation)."""

    def apply(self, parameters, modules_to_apply):
        # Target direction is field-absolute — don't rotate by operator perspective
        angle_to_face = self.target_direction

        to_apply_omega = (
            self.target_rate_feedforward
            + self.heading_controller.calculate(
                parameters.current_pose.rotation().radians(),
                angle_to_face.radians(),
                parameters.timestamp,
            )
        )
        if self.max_abs_rotational_rate > 0.0:
            if to_apply_omega > self.max_abs_rotational_rate:
                to_apply_omega = self.max_abs_rotational_rate
            elif to_apply_omega < -self.max_abs_rotational_rate:
                to_apply_omega = -self.max_abs_rotational_rate

        # Velocities still use forward_perspective (operator perspective) for joystick
        return (
            self._FieldCentricFacingAngle__field_centric.with_velocity_x(
                self.velocity_x
            )
            .with_velocity_y(self.velocity_y)
            .with_rotational_rate(to_apply_omega)
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
            .with_center_of_rotation(self.center_of_rotation)
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules_to_apply)
        )
