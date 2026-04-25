import math
from phoenix6.hardware import TalonFX
from rev import SparkFlex, SparkFlexSim
from wpilib.simulation import DCMotorSim, FlywheelSim, SingleJointedArmSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations


class FlywheelMotorSim:
    """Simulates a single TalonFX-driven flywheel (motor + gearbox + inertia)."""

    def __init__(
        self,
        motor: TalonFX,
        motor_model: DCMotor,
        gearing: float,
        moi: float,
        viscous_damping: float = 0.0,
    ):
        """
        viscous_damping: volts per (output rps) of opposing input. Lowers the
            sim's effective free speed and provides braking — useful for
            mechanisms that would otherwise rail at unreachable target
            velocities. Default 0 preserves old behavior.
        """
        self._sim_state = motor.sim_state
        self._gearing = gearing
        self._viscous_damping = viscous_damping
        plant = LinearSystemId.flywheelSystem(motor_model, moi, gearing)
        self._flywheel = FlywheelSim(plant, motor_model)
        self._position_rot = 0.0

    def update(self, tm_diff: float) -> None:
        self._sim_state.set_supply_voltage(12.0)
        velocity_rps = radiansToRotations(self._flywheel.getAngularVelocity())
        damping_v = -self._viscous_damping * velocity_rps
        self._flywheel.setInputVoltage(self._sim_state.motor_voltage + damping_v)
        self._flywheel.update(tm_diff)

        velocity_rps = radiansToRotations(self._flywheel.getAngularVelocity())
        self._position_rot += velocity_rps * tm_diff

        self._sim_state.set_raw_rotor_position(self._position_rot * self._gearing)
        self._sim_state.set_rotor_velocity(velocity_rps * self._gearing)


class ArmMotorSim:
    def __init__(
        self,
        motor: TalonFX,
        motor_model: DCMotor,
        gearing: float,
        moi: float,
        arm_length: float,
        min_angle: float,
        max_angle: float,
        starting_angle: float = 0.0,
    ):
        self._sim_state = motor.sim_state
        self._gearing = gearing
        plant = LinearSystemId.singleJointedArmSystem(motor_model, moi, gearing)
        self._arm = SingleJointedArmSim(
            plant,
            motor_model,
            gearing,
            arm_length,
            min_angle,
            max_angle,
            simulateGravity=True,
            startingAngle=starting_angle,
        )

    def update(self, tm_diff: float) -> None:
        self._sim_state.set_supply_voltage(12.0)
        self._arm.setInputVoltage(self._sim_state.motor_voltage)
        self._arm.update(tm_diff)

        velocity_rps = radiansToRotations(self._arm.getVelocity())
        position_rot = radiansToRotations(self._arm.getAngle())

        self._sim_state.set_raw_rotor_position(position_rot * self._gearing)
        self._sim_state.set_rotor_velocity(velocity_rps * self._gearing)


class MotorPositionSim:
    """Simulates a TalonFX under position control using DCMotorSim.

    Follows the Phoenix 6 Python MotionMagic example pattern
    (CrossTheRoadElec/Phoenix6-Examples, python/MotionMagic/physics.py):
    feed motor_voltage in, pull position/velocity out, write them back to
    the TalonFX sim state as rotor-side rotations.

    DCMotorSim.getAngularPosition/Velocity return output-shaft (post-gearbox)
    values, so multiply by gearing to get the rotor-side values
    set_raw_rotor_position/set_rotor_velocity expect.
    """

    def __init__(
        self,
        motor: TalonFX,
        motor_model: DCMotor,
        gearing: float,
        moi: float,
        viscous_damping: float = 0.0,
        static_friction: float = 0.0,
        load_voltage: float = 0.0,
    ):
        """
        viscous_damping: volts per (output rps) of opposing input, linear in
            velocity. Lumps all velocity-proportional losses — mirrors what
            real-robot kV was calibrated against.
        static_friction: volts of holding force at rest. Mechanism stays
            put until the external force exceeds this. Sized to hold
            against gravity + any other bias with no active control.
        load_voltage: constant voltage added to the sim's motor input to
            simulate a directional external load like gravity. For a
            mechanism with kG tuned on the real robot, pass ``kG`` so the
            controller's kG output cancels this bias at rest.
        """
        self._sim_state = motor.sim_state
        self._gearing = gearing
        self._viscous_damping = viscous_damping
        self._static_friction = static_friction
        self._load_voltage = load_voltage
        plant = LinearSystemId.DCMotorSystem(motor_model, moi, gearing)
        self._motor_sim = DCMotorSim(plant, motor_model)

    def update(self, tm_diff: float) -> None:
        self._sim_state.set_supply_voltage(12.0)
        velocity_rps = radiansToRotations(self._motor_sim.getAngularVelocity())
        external_v = self._sim_state.motor_voltage + self._load_voltage
        viscous_v = -self._viscous_damping * velocity_rps

        # At rest, static friction holds the mechanism up to ±static_friction
        # of external force; past that threshold it breaks free.
        if abs(velocity_rps) < 1e-4 and abs(external_v) < self._static_friction:
            friction_v = -external_v
        else:
            friction_v = 0.0

        self._motor_sim.setInputVoltage(external_v + viscous_v + friction_v)
        self._motor_sim.update(tm_diff)

        position_rot = radiansToRotations(self._motor_sim.getAngularPosition())
        velocity_rps = radiansToRotations(self._motor_sim.getAngularVelocity())

        self._sim_state.set_raw_rotor_position(position_rot * self._gearing)
        self._sim_state.set_rotor_velocity(velocity_rps * self._gearing)


class FlywheelMotorSimSparkFlex:
    """Simulates a single SparkFlex-driven flywheel (motor + gearbox + inertia)."""

    def __init__(
        self, motor: SparkFlex, motor_model: DCMotor, gearing: float, moi: float
    ):
        self._sim_state = SparkFlexSim(motor, motor_model)
        plant = LinearSystemId.flywheelSystem(motor_model, moi, gearing)
        self._flywheel = FlywheelSim(plant, motor_model)
        self._position_rot = 0.0

    def update(self, tm_diff: float) -> None:
        self._sim_state.setBusVoltage(12.0)

        # Manually iterate the closed loop controller
        self._sim_state.iterate(
            self._flywheel.getAngularVelocity() * 60 / (2 * math.pi),  # current RPM
            12.0,  # bus voltage
            tm_diff,  # dt
        )

        applied_voltage = self._sim_state.getAppliedOutput() * 12.0
        self._flywheel.setInputVoltage(applied_voltage)
        self._flywheel.update(tm_diff)

        velocity_rps = radiansToRotations(self._flywheel.getAngularVelocity())
        self._position_rot += velocity_rps * tm_diff

        self._sim_state.setPosition(self._position_rot)
        self._sim_state.setVelocity(velocity_rps)
