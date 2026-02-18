from phoenix6.hardware import TalonFX
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations


class FlywheelMotorSim:
    """Simulates a single TalonFX-driven flywheel (motor + gearbox + inertia)."""

    def __init__(self, motor: TalonFX, motor_model: DCMotor, gearing: float, moi: float):
        self._sim_state = motor.sim_state
        plant = LinearSystemId.flywheelSystem(motor_model, moi, gearing)
        self._flywheel = FlywheelSim(plant, motor_model)
        self._position_rot = 0.0

    def update(self, tm_diff: float) -> None:
        self._sim_state.set_supply_voltage(12.0)
        self._flywheel.setInputVoltage(self._sim_state.motor_voltage)
        self._flywheel.update(tm_diff)

        velocity_rps = radiansToRotations(self._flywheel.getAngularVelocity())
        self._position_rot += velocity_rps * tm_diff

        self._sim_state.set_raw_rotor_position(self._position_rot)
        self._sim_state.set_rotor_velocity(velocity_rps)
