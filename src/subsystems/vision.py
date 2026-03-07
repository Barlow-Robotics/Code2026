from dataclasses import dataclass
from typing import Optional, List
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout
from wpilib._wpilib import RobotBase
from wpimath.geometry import Pose2d, Rotation2d, Transform3d
from wpilib import DriverStation
from constants import SI, VisionConstants, RobotFeatures
from subsystems import Drivetrain
from commands2 import Subsystem, cmd
from commands import FollowTrajectoryCommand

from pykit.logger import Logger as PyKitLogger
from utils.trajectory_generator import CreateTrajectory

if not RobotBase.isReal():
    from photonlibpy.simulation import (
        PhotonCameraSim,
        SimCameraProperties,
        VisionSystemSim,
    )

XY_STD_DEV_COEFFICIENT = 0.05  # Base xy std dev coefficient
THETA_STD_DEV_COEFFICIENT = 0.01  # Base theta std dev coefficient
DISTANCE_EXPONENT = 1.9  # How aggressively distance degrades trust
TAG_COUNT_EXPONENT = 2.0  # How aggressively tag count improves trust
FIELD_BORDER_MARGIN = 0.5  # Metres outside field to still accept a pose
TIMESTAMP_OFFSET = 0.0  # Adjust if clocks drift between coprocessor/RIO
CAMERA_HEIGHT_TOLERANCE = 1  # Metres of tolerance on the camera Z value
POSE_AMBIGUITY = 0.2  # Minimum pose ambiguity to accept from a single-tag detection (0-1, lower is more strict)
MAX_ANGULAR_VELOCITY_DEGREES = 90
MAX_ANGLE_DIFFERENCE_FOR_GYRO_DISAMBIGUATION_DEGREES = 30





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
class _CameraConfig:
    camera: PhotonCamera
    estimator: PhotonPoseEstimator
    name: str
    robot_to_camera: Transform3d
    std_dev_factor: float = 1.0
    camera_sim: Optional[PhotonCameraSim] = None


class Vision(Subsystem):
    def __init__(self, drive_sub: Drivetrain):
        self.drive_sub = drive_sub

        field_layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT)
        self.april_tag_field_layout = field_layout

        self._cameras: List[_CameraConfig] = [
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.CAMERA_2_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.FRONT_RIGHT_SWERVE_TO_ROBOT
                ),
                name="right_cam_swerve",
                robot_to_camera=VisionConstants.FRONT_RIGHT_SWERVE_TO_ROBOT,
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.CAMERA_1_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.FRONT_LEFT_SWERVE_TO_ROBOT
                ),
                name="left_cam_swerve",
                robot_to_camera=VisionConstants.FRONT_LEFT_SWERVE_TO_ROBOT,
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.CAMERA_3_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.BACK_LEFT_SWERVE_TO_ROBOT
                ),
                name="back_left_swerve",
                robot_to_camera=VisionConstants.BACK_LEFT_SWERVE_TO_ROBOT,
            ),
            _CameraConfig(
                camera=PhotonCamera(VisionConstants.CAMERA_4_NAME),
                estimator=PhotonPoseEstimator(
                    field_layout, VisionConstants.BACK_RIGHT_SWERVE_TO_ROBOT
                ),
                name="back_right_swerve",
                robot_to_camera=VisionConstants.BACK_RIGHT_SWERVE_TO_ROBOT,
            ),
        ]
        self._cameras = self._cameras[0 : RobotFeatures.vision_camera_count]

        self.disabled_vision = False
        self._last_pose_estimate: Optional[Pose2d] = None
        self._camera_stats: dict[str, CameraStats] = {}

        self._observations_per_cycle: float = 0.0
        self._total_detected_targets: float = 0.0
        self._auto_align_triggered: bool = False
        self._auto_align_starting_pose: Optional[Pose2d] = None
        self._auto_align_target_pose: Optional[Pose2d] = None

        self.auto_align_command = cmd.runOnce(self.auto_align)

        # Log camera config on init
        for i, cam_cfg in enumerate(self._cameras):
            PyKitLogger.recordOutput(f"Vision/Config/camera_{i}_name", cam_cfg.name)
        PyKitLogger.recordOutput(
            "Vision/Config/camera_count", float(len(self._cameras))
        )

        if not RobotBase.isReal() and VisionConstants.VISION_SIM:
            self._vision_sim = VisionSystemSim("main")
            self._vision_sim.addAprilTags(self.april_tag_field_layout)

            for cam_cfg in self._cameras:
                cam_props = SimCameraProperties()
                cam_props.setFPS(1)
                cam_props.setCalibrationFromFOV(
                    800, 600, Rotation2d(SI.degrees_to_radians * 97)
                )

                cam_cfg.camera_sim = PhotonCameraSim(
                    cam_cfg.camera, cam_props, self.april_tag_field_layout
                )

                self._vision_sim.addCamera(cam_cfg.camera_sim, cam_cfg.robot_to_camera)
        else:
            self._vision_sim = None

    def periodic(self):
        if not RobotBase.isReal() and VisionConstants.VISION_SIM:
            self.simulation_periodic()

        if not self.disabled_vision:
            current_pose = self.drive_sub.get_pose()
            self._update_all_cameras(current_pose)

        self._total_detected_targets = float(len(self.get_all_detected_targets()))

        if self._last_pose_estimate is not None:
            PyKitLogger.recordOutput(
                "Vision/elevator_camera_pose", self._last_pose_estimate
            )

        closest_tag = self.find_pose_of_tag_closest_to_robot(self.drive_sub.get_pose())
        if closest_tag is not None:
            PyKitLogger.recordOutput("Vision/closest_april_tag", closest_tag)

        for stat_name, stat_key in [
            ("front_left_swerve", "front_left"),
            ("front_right_swerve", "front_right"),
            ("back_left_swerve", "back_left"),
            ("back_right_swerve", "back_right"),
        ]:
            stats = self._camera_stats.get(stat_name)
            if stats is not None:
                PyKitLogger.recordOutput(f"Vision/{stat_key}/std_dev", stats.std_dev)
                PyKitLogger.recordOutput(
                    f"Vision/{stat_key}/tag_count", stats.tag_count
                )
                PyKitLogger.recordOutput(f"Vision/{stat_key}/distance", stats.distance)

        # --- VisionStateTelemetry ---
        PyKitLogger.recordOutput("Vision/State/vision_disabled", self.disabled_vision)
        PyKitLogger.recordOutput(
            "Vision/State/observations_per_cycle", self._observations_per_cycle
        )
        PyKitLogger.recordOutput(
            "Vision/State/total_detected_targets", self._total_detected_targets
        )
        PyKitLogger.recordOutput(
            "Vision/State/auto_align_triggered", self._auto_align_triggered
        )

        if self._auto_align_starting_pose is not None:
            PyKitLogger.recordOutput(
                "Vision/State/auto_align_starting_pose", self._auto_align_starting_pose
            )
        if self._auto_align_target_pose is not None:
            PyKitLogger.recordOutput(
                "Vision/State/auto_align_target_pose", self._auto_align_target_pose
            )

        # --- Camera connection status ---
        for cam_cfg in self._cameras:
            PyKitLogger.recordOutput(
                f"Vision/Connection/{cam_cfg.name}", cam_cfg.camera.isConnected()
            )

    def _update_all_cameras(self, drive_pose: Pose2d):
        all_observations: List[VisionObservation] = []

        for cam_cfg in self._cameras:
            obs_list = self._get_observations_from_camera(drive_pose, cam_cfg)
            all_observations.extend(obs_list)

        all_observations.sort(key=lambda o: o.timestamp)
        self._observations_per_cycle = float(len(all_observations))

        for obs in all_observations:
            self.drive_sub.add_vision_measurement(obs.pose, obs.timestamp, obs.std_devs)
            self._last_pose_estimate = obs.pose

    def _get_observations_from_camera(
        self,
        drive_pose: Pose2d,
        cam_cfg: _CameraConfig,
    ) -> List[VisionObservation]:
        prefix = f"Vision/{cam_cfg.name}"

        if not cam_cfg.camera.isConnected():
            return []

        observations: List[VisionObservation] = []

        for result in cam_cfg.camera.getAllUnreadResults():
            targets = result.getTargets()
            PyKitLogger.recordOutput(f"{prefix}/targets_seen", float(len(targets)))

            if len(targets) == 1 and targets[0].getPoseAmbiguity() > POSE_AMBIGUITY:
                # continue
                target = targets[0]

                PyKitLogger.recordOutput(
                    f"{prefix}/using_single_tag_gyro_disambiguation", True
                )
                PyKitLogger.recordOutput(
                    f"{prefix}/single_tag_id", float(target.getFiducialId())
                )
                PyKitLogger.recordOutput(
                    f"{prefix}/single_tag_ambiguity", target.getPoseAmbiguity()
                )

                tag_pose = self.april_tag_field_layout.getTagPose(
                    target.getFiducialId()
                )
                if tag_pose is None:
                    PyKitLogger.recordOutput(
                        f"{prefix}/rejected_no_tag_pose_in_layout", True
                    )
                    continue

                camera_to_robot = cam_cfg.estimator.robotToCamera.inverse()
                robot_pose_best = tag_pose.transformBy(
                    target.getBestCameraToTarget().inverse()
                ).transformBy(camera_to_robot)
                robot_pose_alt = tag_pose.transformBy(
                    target.getAlternateCameraToTarget().inverse()
                ).transformBy(camera_to_robot)

                gyro = self.drive_sub.get_rotation()
                diff_best = abs(
                    (gyro - robot_pose_best.toPose2d().rotation()).radians()
                )
                diff_alt = abs((gyro - robot_pose_alt.toPose2d().rotation()).radians())

                PyKitLogger.recordOutput(f"{prefix}/gyro_diff_best_rad", diff_best)
                PyKitLogger.recordOutput(f"{prefix}/gyro_diff_alt_rad", diff_alt)
                PyKitLogger.recordOutput(
                    f"{prefix}/chose_best_pose", diff_best < diff_alt
                )

                if diff_best < diff_alt:
                    estimated = EstimatedRobotPose(
                        estimatedPose=robot_pose_best,
                        targetsUsed=[target],
                        timestampSeconds=result.getTimestampSeconds(),
                    )
                else:
                    estimated = EstimatedRobotPose(
                        estimatedPose=robot_pose_alt,
                        targetsUsed=[target],
                        timestampSeconds=result.getTimestampSeconds(),
                    )

                if (
                    min(diff_best, diff_alt)
                    > MAX_ANGLE_DIFFERENCE_FOR_GYRO_DISAMBIGUATION_DEGREES
                    * SI.degrees_to_radians
                ):
                    PyKitLogger.recordOutput(
                        f"{prefix}/rejected_gyro_disambiguation_too_large", True
                    )
                    continue
            else:
                PyKitLogger.recordOutput(
                    f"{prefix}/using_single_tag_gyro_disambiguation", False
                )
                estimated = self._get_best_pose_estimate(result, cam_cfg.estimator)
                if estimated is None:
                    PyKitLogger.recordOutput(
                        f"{prefix}/rejected_no_valid_estimate", True
                    )
                    continue

            tags = estimated.targetsUsed
            tag_count = len(tags)
            PyKitLogger.recordOutput(f"{prefix}/accepted_tag_count", float(tag_count))

            if tag_count == 0:
                PyKitLogger.recordOutput(f"{prefix}/rejected_zero_tags", True)
                continue

            pose_2d = estimated.estimatedPose.toPose2d()

            PyKitLogger.recordOutput(f"{prefix}/raw_estimated_pose", pose_2d)
            PyKitLogger.recordOutput(
                f"{prefix}/raw_estimated_pose_z", estimated.estimatedPose.Z()
            )

            field_length = self.april_tag_field_layout.getFieldLength()
            field_width = self.april_tag_field_layout.getFieldWidth()
            out_of_bounds = (
                pose_2d.X() < -FIELD_BORDER_MARGIN
                or pose_2d.X() > field_length + FIELD_BORDER_MARGIN
                or pose_2d.Y() < -FIELD_BORDER_MARGIN
                or pose_2d.Y() > field_width + FIELD_BORDER_MARGIN
            )
            if out_of_bounds:
                PyKitLogger.recordOutput(f"{prefix}/rejected_out_of_bounds", True)
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_out_of_bounds_x", pose_2d.X()
                )
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_out_of_bounds_y", pose_2d.Y()
                )
                continue
            if (
                self.drive_sub.get_speeds().omega
                > SI.degrees_to_radians * MAX_ANGULAR_VELOCITY_DEGREES
            ):
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_excessive_angular_velocity", True
                )
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_excessive_angular_velocity_value",
                    self.drive_sub.get_speeds().omega,
                )
                continue

            # if self._should_reject_by_alliance(estimated.targetsUsed):
            #     PyKitLogger.recordOutput(f"{prefix}/rejected_wrong_alliance_tag", True)
            #     continue

            if self._should_reject_by_z(estimated):
                PyKitLogger.recordOutput(f"{prefix}/rejected_bad_z", True)
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_bad_z_value", estimated.estimatedPose.Z()
                )
                continue

            avg_distance = self._average_tag_distance(
                estimated.estimatedPose.toPose2d(), tags, cam_cfg.estimator
            )
            std_devs = self._calculate_std_devs(
                avg_distance, tag_count, cam_cfg.std_dev_factor
            )
            if avg_distance > 4.2:
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_too_far_from_tags", True
                )
                PyKitLogger.recordOutput(
                    f"{prefix}/rejected_too_far_from_tags_distance", avg_distance
                )
                continue
            timestamp = estimated.timestampSeconds + TIMESTAMP_OFFSET
            
            PyKitLogger.recordOutput(f"{prefix}/accepted_avg_distance_m", avg_distance)
            PyKitLogger.recordOutput(f"{prefix}/accepted_xy_std_dev", std_devs[0])
            PyKitLogger.recordOutput(
                f"{prefix}/accepted_theta_std_dev",
                std_devs[2] if std_devs[2] != float("inf") else -1.0,
            )
            PyKitLogger.recordOutput(f"{prefix}/accepted_timestamp", timestamp)
            PyKitLogger.recordOutput(f"{prefix}/accepted_pose", pose_2d)

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
        estimated = estimator.estimateCoprocMultiTagPose(result)
        if estimated is None:
            estimated = estimator.estimateLowestAmbiguityPose(result)
        return estimated

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
        """
        tag_weight = tag_count**TAG_COUNT_EXPONENT
        dist_weight = avg_distance**DISTANCE_EXPONENT

        xy_std = XY_STD_DEV_COEFFICIENT * dist_weight / tag_weight * std_dev_factor
        theta_std = (
            THETA_STD_DEV_COEFFICIENT * dist_weight / tag_weight * std_dev_factor
        )

        if tag_count == 1 and avg_distance > 4.0:
            theta_std = float("inf")

        return [xy_std, xy_std, theta_std]

    def _average_tag_distance(
        self,
        robot_pose: Pose2d,
        targets: List[PhotonTrackedTarget],
        estimator: PhotonPoseEstimator,
    ) -> float:
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
    def _should_reject_by_z(estimated: EstimatedRobotPose) -> bool:
        return abs(estimated.estimatedPose.Z()) > CAMERA_HEIGHT_TOLERANCE

    @staticmethod
    def _should_reject_by_alliance(targets: List[PhotonTrackedTarget]) -> bool:
        alliance = DriverStation.getAlliance()
        if alliance is None:
            return False
        is_blue = alliance == DriverStation.Alliance.kBlue
        for tag in targets:
            tag_id = tag.getFiducialId()
            if is_blue and 1 <= tag_id <= 16:
                return True
            if not is_blue and 17 <= tag_id <= 32:
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
            april_tag_list = VisionConstants.BLUE_APRIL_TAG_LIST
        elif alliance == DriverStation.Alliance.kRed:
            april_tag_list = VisionConstants.RED_APRIL_TAG_LIST
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
        if self._vision_sim is not None:
            self._vision_sim.update(self.drive_sub.get_pose())

    def auto_align(self):
        print("AUTO ALIGN")
        starting_pose = self.drive_sub.get_pose()
        target_pose = Pose2d(
            starting_pose.X() + 1, starting_pose.Y(), starting_pose.rotation()
        )

        self._auto_align_triggered = True
        self._auto_align_starting_pose = starting_pose
        self._auto_align_target_pose = target_pose

        trajectory_obj = CreateTrajectory(
            lambda: self.drive_sub.get_pose(),
            lambda: self.drive_sub.get_speeds(),
        ).get_trajectory(
            target_pose,
            Rotation2d(target_pose.rotation().radians()),
        )
        FollowTrajectoryCommand(self, trajectory_obj).schedule()
