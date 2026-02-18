from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from subsystems.spindex import Spindex


class SpindexSim:
    def __init__(self, spindex: Spindex):
        # Get the spindex motor from the subsystem
        self.motor_spindex = spindex.motor_spindex

        # Flywheel physics model
        self.gearing = 4.0
        self.moi = 0.005  # kg*m^2 - moment of inertia

        # Create the DC motor model (Kraken X60)
        self.motor_model = DCMotor.krakenX60(1)

        # Create the flywheel simulation
        plant = LinearSystemId.flywheelSystem(self.motor_model, self.moi, self.gearing)
        self.flywheel_sim = FlywheelSim(plant, self.motor_model)

        # Track position by integrating velocity
        self.position_rot = 0.0

        # Get the sim state for the TalonFX
        self.sim_state = self.motor_spindex.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Set supply voltage (battery voltage)
        self.sim_state.set_supply_voltage(12.0)

        # Get the voltage being applied to the motor
        motor_voltage = self.motor_spindex.sim_state.motor_voltage

        # Update the flywheel simulation with the motor voltage
        self.flywheel_sim.setInputVoltage(motor_voltage)
        self.flywheel_sim.update(tm_diff)

        # Get the simulated velocity (rad/s) and convert to rotations/s
        velocity_rps = radiansToRotations(self.flywheel_sim.getAngularVelocity())

        # Integrate velocity to get position
        self.position_rot += velocity_rps * tm_diff

        # Feed the simulated values back to the TalonFX sim state
        self.motor_spindex.sim_state.set_raw_rotor_position(self.position_rot)
        self.motor_spindex.sim_state.set_rotor_velocity(velocity_rps)
