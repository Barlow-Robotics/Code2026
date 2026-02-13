from phoenix6 import StatusCode, configs, signals


class TalonConfig:
    kP: float
    kI: float
    kD: float
    kF: float
    kA: float
    current_limit: float
    break_mode: bool
    output_range: tuple[float, float]

    def __init__(
        self,
        kP: float,
        kI: float,
        kD: float,
        kF: float,
        kA: float,
        kV: float = 0,
        kG: float = 0,
        current_limit: int = 80,
        brake_mode: bool = True,
        output_range: tuple[float, float] = (-1, 1),
        motion_magic_cruise_velocity=20,
        motion_magic_acceleration=600,
        motion_magic_jerk=6000,
    ):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kA = kA
        self.kV = kV
        self.kG = kG
        self.current_limit = current_limit
        self.brake_mode = brake_mode
        self.output_range = output_range
        self.motion_magic_cruise_velocity = motion_magic_cruise_velocity
        self.motion_magic_acceleration = motion_magic_acceleration
        self.motion_magic_jerk = motion_magic_jerk

    def _apply_settings(self, motor, inverted: bool = False):
        print("applying settings to Talon")
        talon_config = configs.TalonFXConfiguration()

        # PID
        pid = talon_config.slot0
        pid.k_p = self.kP
        pid.k_i = self.kI
        pid.k_d = self.kD
        pid.k_s = self.kF
        pid.k_a = self.kA
        pid.k_v = self.kV
        pid.k_g = self.kG

        # current limits
        current_limits_config = talon_config.current_limits
        current_limits_config.stator_current_limit = self.current_limit
        current_limits_config.stator_current_limit_enable = (
            True if self.current_limit > 0 else False
        )

        # brake mode
        brake_mode_config = talon_config.motor_output
        brake_mode_config.neutral_mode = (
            signals.NeutralModeValue.BRAKE
            if self.brake_mode
            else signals.NeutralModeValue.COAST
        )
        brake_mode_config.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if inverted
            else signals.InvertedValue.CLOCKWISE_POSITIVE
        )

        # motion magic
        magic = talon_config.motion_magic
        magic.motion_magic_acceleration = self.motion_magic_acceleration
        magic.motion_magic_jerk = self.motion_magic_jerk
        magic.motion_magic_cruise_velocity = self.motion_magic_cruise_velocity

        # Implementing 6328 logic on configuring talons

        for i in range(10):
            res = motor.configurator.apply(
                talon_config, 0.2
            )  # default timeout is 0.1; we seem to need more time
            if res == StatusCode.OK:
                break

        if res == StatusCode.OK:
            print("talon configured")

        else:
            # pass
            print("error! config not applying")
