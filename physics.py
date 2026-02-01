"""
Physics simulation for the robot.

This module provides physics simulation for the shooter flywheel using
WPILib's DCMotorSim. The simulation takes motor voltage output from
the TalonFX and calculates the resulting velocity, which is fed back
to the motor controller's sim state.
"""

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from phoenix6.hardware import TalonFX
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller

        # Get the shooter motor from the robot
        self.shooter_motor: TalonFX = robot.shooterSubsystem.motor

        # Flywheel physics model
        # Using a Falcon 500 motor with a 1:1 gear ratio
        # Moment of inertia for a typical shooter flywheel (~0.01 kg*m^2)
        self.gearing = 1.0
        self.moi = 0.01  # kg*m^2 - moment of inertia

        # Create the DC motor model (Falcon 500)
        self.motor_model = DCMotor.falcon500(1)

        # Create the flywheel simulation
        plant = LinearSystemId.flywheelSystem(self.motor_model, self.moi, self.gearing)
        self.flywheel_sim = FlywheelSim(plant, self.motor_model)

        # Track position by integrating velocity
        self.position_rot = 0.0

        # Get the sim state for the TalonFX
        self.shooter_sim = self.shooter_motor.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called by pyfrc ~50 times per second (20ms intervals).

        :param now: Current simulation time
        :param tm_diff: Time since last update
        """
        # Feed the Phoenix6 simulation - required for motor controllers to work
        unmanaged.feed_enable(100)  # Keep motors enabled for 100ms

        # Set supply voltage (battery voltage)
        self.shooter_sim.set_supply_voltage(12.0)

        # Get the voltage being applied to the motor
        motor_voltage = self.shooter_sim.motor_voltage

        # Update the flywheel simulation with the motor voltage
        self.flywheel_sim.setInputVoltage(motor_voltage)
        self.flywheel_sim.update(tm_diff)

        # Get the simulated velocity (rad/s) and convert to rotations/s
        velocity_rps = radiansToRotations(self.flywheel_sim.getAngularVelocity())

        # Integrate velocity to get position
        self.position_rot += velocity_rps * tm_diff

        # Feed the simulated values back to the TalonFX sim state
        self.shooter_sim.set_raw_rotor_position(self.position_rot)
        self.shooter_sim.set_rotor_velocity(velocity_rps)
