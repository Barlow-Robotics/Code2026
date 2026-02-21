from dataclasses import dataclass, field
from typing import Optional, List
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import (
    Transform3d,
    Pose2d,
    Translation2d,
    Translation3d,
    Rotation3d,
    Transform2d,
    Rotation2d,
)
from wpilib import DriverStation, Timer
from constants import VisionConstants
from subsystems import Drivetrain
from utils import Logger
from commands2 import Subsystem, cmd
from commands import FollowTrajectoryCommand

from utils.trajectory_generator import CreateTrajectory

XY_STD_DEV_COEFFICIENT = 0.005  # Base xy std dev coefficient
THETA_STD_DEV_COEFFICIENT = 0.01  # Base theta std dev coefficient
DISTANCE_EXPONENT = 1.2  # How aggressively distance degrades trust
TAG_COUNT_EXPONENT = 2.0  # How aggressively tag count improves trust
FIELD_BORDER_MARGIN = 0.5  # Metres outside field to still accept a pose
TIMESTAMP_OFFSET = 0.0  # Adjust if clocks drift between coprocessor/RIO


@dataclass
class CameraStats:
    std_dev: float
    tag_count: float
    distance: float


@dataclass
class VisionObservation:
    pose: Pose2d
    timestamp: float
    std_devs: List[float]


@dataclass
class VisionTelemetry:
    elevator_camera_pose: Pose2d | None = None
    closest_april_tag: Pose2d | None = None
    front_left: CameraStats | None = None
    front_right: CameraStats | None = None
    back_left: CameraStats | None = None
    back_right: CameraStats | None = None


@dataclass
class _CameraConfig:
    camera: PhotonCamera
    estimator: PhotonPoseEstimator
    name: str
    std_dev_factor: float = 1.0


class Vision(Subsystem):
    def __init__(self, drive_sub: Drivetrain):
        self.drive_sub = drive_sub

        field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
        self.april_tag_field_layout = field_layout

        self._cameras: List[_CameraConfig] = [
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.FRONT_LEFT_SWERVE_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.FRONT_LEFT_SWERVE_TO_ROBOT
                ),
                name="front_left_swerve",
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.FRONT_RIGHT_SWERVE_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.FRONT_RIGHT_SWERVE_TO_ROBOT
                ),
                name="front_right_swerve",
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.BACK_LEFT_SWERVE_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.BACK_LEFT_SWERVE_TO_ROBOT
                ),
                name="back_left_swerve",
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.BACK_RIGHT_SWERVE_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.BACK_RIGHT_SWERVE_TO_ROBOT
                ),
                name="back_right_swerve",
            ),
        ]

        self.disabled_vision = False
        self._last_pose_estimate: Optional[Pose2d] = None
        self._camera_stats: dict[str, CameraStats] = {}

        self.log = Logger("Vision")
        self.auto_align_command = cmd.runOnce(self.auto_align)

    def periodic(self):
        if not self.disabled_vision:
            current_pose = self.drive_sub.get_pose()
            self._update_all_cameras(current_pose)

        self.log.publish(
            VisionTelemetry(
                elevator_camera_pose=self._last_pose_estimate,
                closest_april_tag=self.find_pose_of_tag_closest_to_robot(
                    self.drive_sub.get_pose()
                ),
                front_left=self._camera_stats.get("front_left_swerve"),
                front_right=self._camera_stats.get("front_right_swerve"),
                back_left=self._camera_stats.get("back_left_swerve"),
                back_right=self._camera_stats.get("back_right_swerve"),
            )
        )

    def _update_all_cameras(self, drive_pose: Pose2d):
        """
        Collect observations from every camera, sort them chronologically,
        then add them to the drive estimator — matching 6328's approach of
        sorted multi-source fusion.
        """
        all_observations: List[VisionObservation] = []

        for cam_cfg in self._cameras:
            obs_list = self._get_observations_from_camera(drive_pose, cam_cfg)
            all_observations.extend(obs_list)

        all_observations.sort(key=lambda o: o.timestamp)

        for obs in all_observations:
            self.drive_sub.add_vision_measurement(obs.pose, obs.timestamp, obs.std_devs)
            self._last_pose_estimate = obs.pose

    def _get_observations_from_camera(
        self,
        drive_pose: Pose2d,
        cam_cfg: _CameraConfig,
    ) -> List[VisionObservation]:
        """
        Process every unread result from one camera and return a list of
        validated VisionObservations.  Returns an empty list if the camera
        is disconnected or produces no usable data.
        """
        if not cam_cfg.camera.isConnected():
            return []

        observations: List[VisionObservation] = []

        for result in cam_cfg.camera.getAllUnreadResults():
            estimated = self._get_best_pose_estimate(result, cam_cfg.estimator)
            if estimated is None:
                continue

            pose_2d = estimated.estimatedPose.toPose2d()

            # ── Reject poses outside the field ────────────────────────────
            field_length = self.april_tag_field_layout.getFieldLength()
            field_width = self.april_tag_field_layout.getFieldWidth()
            if (
                pose_2d.X() < -FIELD_BORDER_MARGIN
                or pose_2d.X() > field_length + FIELD_BORDER_MARGIN
                or pose_2d.Y() < -FIELD_BORDER_MARGIN
                or pose_2d.Y() > field_width + FIELD_BORDER_MARGIN
            ):
                continue

            if self._should_reject_by_alliance(estimated.targetsUsed):
                continue

            tags = estimated.targetsUsed
            tag_count = len(tags)
            if tag_count == 0:
                continue

            avg_distance = self._average_tag_distance(
                estimated.estimatedPose.toPose2d(), tags, cam_cfg.estimator
            )

            std_devs = self._calculate_std_devs(
                avg_distance, tag_count, cam_cfg.std_dev_factor
            )

            # ── Apply timestamp offset (corrects coprocessor clock drift) ─
            timestamp = estimated.timestampSeconds + TIMESTAMP_OFFSET

            # ── Update telemetry stats ────────────────────────────────────
            self._camera_stats[cam_cfg.name] = CameraStats(
                std_dev=std_devs[0],
                tag_count=float(tag_count),
                distance=avg_distance,
            )

            observations.append(
                VisionObservation(pose=pose_2d, timestamp=timestamp, std_devs=std_devs)
            )

        return observations

    def _get_best_pose_estimate(
        self,
        result: PhotonPipelineResult,
        estimator: PhotonPoseEstimator,
    ) -> Optional[EstimatedRobotPose]:
        """
        Prefer multi-tag (coprocessor) estimate; fall back to lowest-ambiguity
        single-tag estimate.  Multi-tag results are now accepted, not discarded.
        """
        estimated = estimator.estimateCoprocMultiTagPose(result)
        if estimated is None:
            estimated = estimator.estimateLowestAmbiguityPose(result)
        return estimated

    # ── Standard deviation calculation ───────────────────────────────────────

    @staticmethod
    def _calculate_std_devs(
        avg_distance: float,
        tag_count: int,
        std_dev_factor: float = 1.0,
    ) -> List[float]:
        """
        Mirror 6328's formula:
            xy  = XY_COEFF  * dist^1.2 / tag_count^2 * factor
            θ   = TH_COEFF  * dist^1.2 / tag_count^2 * factor

        More tags → lower std dev (more trust).
        More distance → higher std dev (less trust).
        Single-tag at very long range → effectively infinite std dev.
        """
        tag_weight = tag_count**TAG_COUNT_EXPONENT
        dist_weight = avg_distance**DISTANCE_EXPONENT

        xy_std = XY_STD_DEV_COEFFICIENT * dist_weight / tag_weight * std_dev_factor
        theta_std = (
            THETA_STD_DEV_COEFFICIENT * dist_weight / tag_weight * std_dev_factor
        )

        # For single-tag observations at long range, trust rotation even less
        if tag_count == 1 and avg_distance > 4.0:
            theta_std = float("inf")

        return [xy_std, xy_std, theta_std]

    def _average_tag_distance(
        self,
        robot_pose: Pose2d,
        targets: List[PhotonTrackedTarget],
        estimator: PhotonPoseEstimator,
    ) -> float:
        """Calculate average 2D distance from robot to each visible tag."""
        total = 0.0
        count = 0
        for tgt in targets:
            tag_pose = estimator.fieldTags.getTagPose(tgt.getFiducialId())
            if tag_pose is None:
                continue
            total += (
                tag_pose.toPose2d().translation().distance(robot_pose.translation())
            )
            count += 1
        return total / count if count > 0 else 0.0

    @staticmethod
    def _should_reject_by_alliance(targets: List[PhotonTrackedTarget]) -> bool:
        """Return True if any visible tag belongs to the opponent's side."""
        alliance = DriverStation.getAlliance()
        if alliance is None:
            return False

        is_blue = alliance == DriverStation.Alliance.kBlue
        for tag in targets:
            tag_id = tag.getFiducialId()
            if is_blue and 6 <= tag_id <= 11:
                return True
            if not is_blue and 17 <= tag_id <= 22:
                return True
        return False

    def get_layout(self) -> AprilTagFieldLayout:
        return self.april_tag_field_layout

    def disable_the_vision(self, val: bool):
        self.disabled_vision = val

    def find_pose_of_tag_closest_to_robot(self, drive_pose: Pose2d) -> Optional[Pose2d]:
        alliance = DriverStation.getAlliance()
        if alliance is None:
            return None

        if alliance == DriverStation.Alliance.kBlue:
            april_tag_list = VisionConstants.BLUE_APRIL_TAG_LIST_REEF
        elif alliance == DriverStation.Alliance.kRed:
            april_tag_list = VisionConstants.RED_APRIL_TAG_LIST_REEF
        else:
            return None

        possible_poses: List[Pose2d] = []
        for tag_id in april_tag_list:
            tag_pose = self.get_layout().getTagPose(tag_id)
            if tag_pose is not None:
                possible_poses.append(tag_pose.toPose2d())

        return drive_pose.nearest(possible_poses) if possible_poses else None

    @staticmethod
    def is_reef_tag(primary_id: int) -> bool:
        return (6 <= primary_id <= 11) or (17 <= primary_id <= 22)

    @staticmethod
    def filter_april_tag_field(field: AprilTagFieldLayout) -> AprilTagFieldLayout:
        new_tags = [tag for tag in field.getTags() if Vision.is_reef_tag(tag.ID)]
        return AprilTagFieldLayout(
            new_tags, field.getFieldLength(), field.getFieldWidth()
        )

    def get_all_detected_targets(self) -> List[PhotonTrackedTarget]:
        all_targets: List[PhotonTrackedTarget] = []
        for cam_cfg in self._cameras:
            if cam_cfg.camera.isConnected():
                result = cam_cfg.camera.getLatestResult()
                all_targets.extend(result.getTargets())
        return all_targets

    def simulation_periodic(self):
        pass

    def auto_align(self):
        print("AUTO ALIGN")
        starting_pose = self.drive_sub.get_pose()
        target_pose = Pose2d(
            starting_pose.X() + 1, starting_pose.Y(), starting_pose.rotation()
        )

        trajectory_obj = CreateTrajectory(
            lambda: self.drive_sub.get_pose,
            lambda: self.drive_sub.get_speeds,
        ).get_trajectory(
            target_pose,
            Rotation2d(target_pose.rotation().radians()),
        )
        FollowTrajectoryCommand(self.drive_sub, trajectory_obj).schedule()
