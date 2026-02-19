from dataclasses import dataclass

from phoenix6 import SignalLogger, swerve, units
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState

from utils.telemetry import Logger


@dataclass
class DriveStateTelemetry:
    Pose: Pose2d
    Speeds: ChassisSpeeds
    ModuleStates: list[SwerveModuleState]
    ModuleTargets: list[SwerveModuleState]
    ModulePositions: list[SwerveModulePosition]
    Timestamp: float
    OdometryFrequency: float


class SwerveTelemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        self.log = Logger("DriveState")
        self._field_log = Logger("Pose")

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        ]

        # Set up the module state Mechanism2d telemetry
        for i, module_mechanism in enumerate(self._module_mechanisms):
            SmartDashboard.putData(f"Module {i}", module_mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        # Publish drive state to NT via Logger
        self.log.publish(
            DriveStateTelemetry(
                Pose=state.pose,
                Speeds=state.speeds,
                ModuleStates=state.module_states,
                ModuleTargets=state.module_targets,
                ModulePositions=state.module_positions,
                Timestamp=state.timestamp,
                OdometryFrequency=1.0 / state.odometry_period,
            )
        )

        # Write to .hoot log file
        SignalLogger.write_struct("DriveState/Pose", Pose2d, state.pose)
        SignalLogger.write_struct("DriveState/Speeds", ChassisSpeeds, state.speeds)
        SignalLogger.write_struct_array(
            "DriveState/ModuleStates", SwerveModuleState, state.module_states
        )
        SignalLogger.write_struct_array(
            "DriveState/ModuleTargets", SwerveModuleState, state.module_targets
        )
        SignalLogger.write_struct_array(
            "DriveState/ModulePositions", SwerveModulePosition, state.module_positions
        )
        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )

        # Publish Field2d pose
        self._field_log.put_string(".type", "Field2d")
        self._field_log.put_double_array(
            "robotPose",
            [state.pose.x, state.pose.y, state.pose.rotation().degrees()],
        )

        # Update Mechanism2d visualization
        for i, module_state in enumerate(state.module_states):
            self._module_speeds[i].setAngle(module_state.angle.degrees())
            self._module_directions[i].setAngle(module_state.angle.degrees())
            self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))
