import math
from phoenix6.hardware import TalonFX
from rev import SparkFlex, SparkFlexSim
from wpilib.simulation import FlywheelSim, SingleJointedArmSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations


class FlywheelMotorSim:
    """Simulates a single TalonFX-driven flywheel (motor + gearbox + inertia)."""

    def __init__(
        self, motor: TalonFX, motor_model: DCMotor, gearing: float, moi: float
    ):
        self._sim_state = motor.sim_state
        self._gearing = gearing
        plant = LinearSystemId.flywheelSystem(motor_model, moi, gearing)
        self._flywheel = FlywheelSim(plant, motor_model)
        self._position_rot = 0.0

    def update(self, tm_diff: float) -> None:
        self._sim_state.set_supply_voltage(12.0)
        self._flywheel.setInputVoltage(self._sim_state.motor_voltage)
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
