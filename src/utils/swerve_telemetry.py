from phoenix6 import SignalLogger, swerve, units
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState

from pykit.logger import Logger as PyKitLogger


class SwerveTelemetry:
    def __init__(self, max_speed: units.meters_per_second):
        self._max_speed = max_speed

        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
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

        for i, module_mechanism in enumerate(self._module_mechanisms):
            SmartDashboard.putData(f"Module {i}", module_mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        # PyKit logging
        PyKitLogger.recordOutput("DriveState/Pose", state.pose)
        PyKitLogger.recordOutput("DriveState/Speeds", state.speeds)
        PyKitLogger.recordOutput("DriveState/ModuleStates", state.module_states)
        PyKitLogger.recordOutput("DriveState/ModuleTargets", state.module_targets)
        PyKitLogger.recordOutput("DriveState/ModulePositions", state.module_positions)
        PyKitLogger.recordOutput("DriveState/Timestamp", state.timestamp)
        PyKitLogger.recordOutput(
            "DriveState/OdometryFrequency", 1.0 / state.odometry_period
        )

        # Field2d for SmartDashboard/Shuffleboard
        PyKitLogger.recordOutput("Pose/robotPose", state.pose)

        # Write to .hoot log file (keep for Phoenix Tuner X compatibility)
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
        PyKitLogger.recordOutput(".type", "Field2d")
        for i, module_state in enumerate(state.module_states):
            self._module_speeds[i].setAngle(module_state.angle.degrees())
            self._module_directions[i].setAngle(module_state.angle.degrees())
            self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))
