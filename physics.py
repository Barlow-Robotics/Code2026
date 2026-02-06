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
from wpilib.simulation import FlywheelSim, SingleJointedArmSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations
from typing import TYPE_CHECKING
import math
if TYPE_CHECKING:
    from robot import Robot

class IntakeSim:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        # Get the shooter motor from the robot
        self.motor_roller_top: TalonFX = robot.intakeSubsystem.motor_roller_top
        self.motor_roller_bottom: TalonFX = robot.intakeSubsystem.motor_roller_bottom
        
        
        self.motor_head: TalonFX = robot.intakeSubsystem.motor_head
        self.motor_arm: TalonFX = robot.intakeSubsystem.motor_arm


        # Flywheel physics model
        # Using a Falcon 500 motor with a 1:1 gear ratio
        # Moment of inertia for a typical shooter flywheel (~0.01 kg*m^2)
        self.gearing = 4.0
        self.moi = 0.01  # kg*m^2 - moment of inertia

        # Create the DC motor model (Falcon 500)
        self.motor_model_roller_top = DCMotor.krakenX60(1)
        self.motor_model_roller_bottom = DCMotor.krakenX60(1)
        self.motor_model_head = DCMotor.krakenX60(1)
        self.motor_model_arm = DCMotor.krakenX60(1)


        # Create the flywheel simulation
        roller_top_plant = LinearSystemId.flywheelSystem(self.motor_model_roller_top, self.moi, self.gearing)
        self.roller_top_flywheel_sim = FlywheelSim(roller_top_plant, self.motor_model_roller_top)

        roller_bottom_plant = LinearSystemId.flywheelSystem(self.motor_model_roller_bottom, self.moi, self.gearing)
        self.roller_bottom_flywheel_sim = FlywheelSim(roller_bottom_plant, self.motor_model_roller_bottom)
        
        head_plant = LinearSystemId.singleJointedArmSystem(self.motor_model_head, self.moi, self.gearing)
        self.head_jointed_sim = SingleJointedArmSim(head_plant, self.motor_model_head, armLength=6.99/12, minAngle=math.radians(18.6), maxAngle=math.radians(160), simulateGravity=True)

        arm_plant = LinearSystemId.singleJointedArmSystem(self.motor_model_arm, self.moi, self.gearing)
        self.arm_jointed_sim = SingleJointedArmSim(arm_plant, self.motor_model_arm, gearing=self.gearing, armLength=5.513/12, minAngle=math.radians(0), maxAngle=math.radians(135), simulateGravity=True)
        
        # Track position by integrating velocity
        self.roller_top_position_rot = 0.0
        self.roller_bottom_position_rot = 0.0
        self.arm_position_rot = 0.0
        self.head_position_rot = 0.0

        # Get the sim state for the TalonFX
        self.roller_top_sim = self.motor_roller_top.sim_state
        self.roller_bottom_sim = self.motor_roller_bottom.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:

        # Set supply voltage (battery voltage)
        self.roller_top_sim.set_supply_voltage(12.0)
        self.roller_bottom_sim.set_supply_voltage(12.0)
        self.motor_head.sim_state.set_supply_voltage(12.0)
        self.motor_arm.sim_state.set_supply_voltage(12.0)

        # Get the voltage being applied to the motor
        motor_voltage_top = self.motor_roller_top.sim_state.motor_voltage
        motor_voltage_bottom = self.motor_roller_bottom.sim_state.motor_voltage
        motor_voltage_arm = self.motor_arm.sim_state.motor_voltage
        motor_voltage_head = self.motor_head.sim_state.motor_voltage

        # Update the flywheel simulation with the motor voltage
        self.roller_top_flywheel_sim.setInputVoltage(motor_voltage_top)
        self.roller_top_flywheel_sim.update(tm_diff)

        self.roller_bottom_flywheel_sim.setInputVoltage(motor_voltage_bottom)
        self.roller_bottom_flywheel_sim.update(tm_diff)
        
        self.head_jointed_sim.setInputVoltage(motor_voltage_head)
        self.head_jointed_sim.update(tm_diff)
        
        self.arm_jointed_sim.setInputVoltage(motor_voltage_arm)
        self.arm_jointed_sim.update(tm_diff)
        # Get the simulated velocity (rad/s) and convert to rotations/s
        velocity_rps_top = radiansToRotations(self.roller_top_flywheel_sim.getAngularVelocity())
        velocity_rps_bottom = radiansToRotations(self.roller_bottom_flywheel_sim.getAngularVelocity())
        velocity_rps_head = radiansToRotations(self.head_jointed_sim.getVelocity())
        velocity_rps_arm = radiansToRotations(self.arm_jointed_sim.getVelocity())

        # Integrate velocity to get position
        self.roller_top_position_rot += velocity_rps_top * tm_diff
        self.roller_bottom_position_rot += velocity_rps_bottom * tm_diff
        self.head_position_rot += velocity_rps_head * tm_diff
        self.arm_position_rot += velocity_rps_arm * tm_diff
        
        # Feed the simulated values back to the TalonFX sim state
        self.motor_roller_top.sim_state.set_raw_rotor_position(self.roller_top_position_rot)
        self.motor_roller_top.sim_state.set_rotor_velocity(velocity_rps_top)

        self.motor_roller_bottom.sim_state.set_raw_rotor_position(self.roller_bottom_position_rot)
        self.motor_roller_bottom.sim_state.set_rotor_velocity(velocity_rps_bottom)

        self.motor_head.sim_state.set_raw_rotor_position(self.head_position_rot)
        self.motor_head.sim_state.set_rotor_velocity(velocity_rps_head)
        
        self.motor_arm.sim_state.set_raw_rotor_position(self.arm_position_rot)
        self.motor_arm.sim_state.set_rotor_velocity(velocity_rps_arm)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller

        self.intake_sim = IntakeSim(physics_controller, robot)



    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called by pyfrc ~50 times per second (20ms intervals).

        :param now: Current simulation time
        :param tm_diff: Time since last update
        """
        # Feed the Phoenix6 simulation - required for motor controllers to work
        unmanaged.feed_enable(100)  # Keep motors enabled for 100ms

        self.intake_sim.update_sim(now, tm_diff)
