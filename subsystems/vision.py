from typing import Optional, List
import ntcore
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import Transform3d, Pose2d, Translation2d
from wpilib import DriverStation
from utils import VisionConstants
from subsystems import Drivetrain
import commands2


class Vision(commands2.Subsystem):
    """
    Vision subsystem for AprilTag-based pose estimation using PhotonVision
    """

    def __init__(
        self,
        drive_sub: Drivetrain,
    ):
        self.drive_sub = drive_sub

        # BW PhotonVision cameras
        self.back_left_swerve_cam = PhotonCamera(VisionConstants.BACK_LEFT_SWERVE_NAME)
        self.back_right_swerve_cam = PhotonCamera(
            VisionConstants.BACK_RIGHT_SWERVE_NAME
        )
        self.front_left_swerve_cam = PhotonCamera(
            VisionConstants.FRONT_LEFT_SWERVE_NAME
        )
        self.front_right_swerve_cam = PhotonCamera(
            VisionConstants.FRONT_RIGHT_SWERVE_NAME
        )

        self.april_tag_field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltAndyMark
        )

        self.front_right_photon_estimator = PhotonPoseEstimator(
            self.april_tag_field_layout, VisionConstants.FRONT_RIGHT_SWERVE_TO_ROBOT
        )

        self.front_left_photon_estimator = PhotonPoseEstimator(
            self.april_tag_field_layout, VisionConstants.FRONT_LEFT_SWERVE_TO_ROBOT
        )
        self.back_right_photon_estimator = PhotonPoseEstimator(
            self.april_tag_field_layout, VisionConstants.BACK_RIGHT_SWERVE_TO_ROBOT
        )
        self.back_left_photon_estimator = PhotonPoseEstimator(
            self.april_tag_field_layout, VisionConstants.BACK_LEFT_SWERVE_TO_ROBOT
        )

        self.disabled_vision = False
        self.all_detected_targets: List[PhotonTrackedTarget] = []
        self.april_tag_detected = False
        self.robot_to_camera: Optional[Transform3d] = None
        self.cur_std_devs = VisionConstants.K_SINGLE_TAG_STD_DEVS.copy()

        # NetworkTables for logging
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Vision")

    def periodic(self):
        """Called periodically by the scheduler"""
        if not self.disabled_vision:
            current_pose = self.drive_sub.get_pose()
            self.update_vision_localization(current_pose)

        self.advantage_kit_logging()

    def get_layout(self) -> AprilTagFieldLayout:
        """Returns the AprilTag field layout"""
        return self.april_tag_field_layout

    def disable_the_vision(self, val: bool):
        """Enable or disable vision processing"""
        self.disabled_vision = val

    def update_vision_localization_camera(
        self,
        drive_pose: Pose2d,
        swerve_cam: PhotonCamera,
        photon_estimator: PhotonPoseEstimator,
    ):
        vision_poses = self.get_unprocessed_poses(
            drive_pose, swerve_cam, photon_estimator
        )
        if vision_poses is not None:
            for vision_pose in vision_poses:
                self.add_vision_measure(vision_pose, swerve_cam.getName())
                self.nt.putNumberArray(
                    "ElevatorCameraPoseEstimate",
                    [
                        vision_pose.estimatedPose.toPose2d().X(),
                        vision_pose.estimatedPose.toPose2d().Y(),
                        vision_pose.estimatedPose.toPose2d().rotation().radians(),
                    ],
                )

    def update_vision_localization(self, drive_pose: Pose2d):
        """Update pose estimation using vision measurements"""
        self.update_vision_localization_camera(
            drive_pose, self.front_left_swerve_cam, self.front_left_photon_estimator
        )
        self.update_vision_localization_camera(
            drive_pose, self.front_right_swerve_cam, self.front_right_photon_estimator
        )
        self.update_vision_localization_camera(
            drive_pose, self.back_left_swerve_cam, self.back_left_photon_estimator
        )
        self.update_vision_localization_camera(
            drive_pose, self.back_right_swerve_cam, self.back_right_photon_estimator
        )

    def get_camera_vision_est(
        self, result: PhotonPipelineResult, estimator: PhotonPoseEstimator
    ) -> EstimatedRobotPose | None:
        """
        Returns an EstimatedRobotPose, which includes pose, timestamp, tags, and strategy
        """
        # result = camera.getLatestResult()
        camEstPose = estimator.estimateCoprocMultiTagPose(result)
        if camEstPose is None:
            camEstPose = estimator.estimateLowestAmbiguityPose(result)

        return camEstPose

    def get_unprocessed_poses(
        self,
        robot_pose: Pose2d,
        camera: PhotonCamera,
        pose_estimator: PhotonPoseEstimator,
    ) -> List[EstimatedRobotPose]:
        """
        Get estimated global pose from camera

        Args:
            robot_pose: Current robot pose estimate
            camera: PhotonCamera instance
            pose_estimator: PhotonPoseEstimator instance

        Returns:
            EstimatedRobotPose if available, None otherwise
        """
        if not camera.isConnected():
            return []

        # BW: Process all unread results
        vision_est = None
        vision_poses: List[EstimatedRobotPose] = []
        for result in camera.getAllUnreadResults():  # BW: What do we want to do with old camera information if there is multiple frames? If robot is moving not rlly relv.
            vision_est = self.get_camera_vision_est(result, camera, pose_estimator)
            vision_poses.append(vision_est)
            # OR UPDATE STD DEVS
            # self.update_estimation_std_devs(vision_est, result.getTargets(), pose_estimator)

        return vision_poses

    def update_estimation_std_devs(
        self,
        estimated_pose: Optional[EstimatedRobotPose],
        targets: List[PhotonTrackedTarget],
        pose_estimator: PhotonPoseEstimator,
    ):
        """
        Calculates new standard deviations based on number of tags and distance

        This algorithm is a heuristic that creates dynamic standard deviations
        based on number of tags, estimation strategy, and distance from the tags.
        """
        if estimated_pose is None:
            # No pose input. Default to single-tag std devs
            self.cur_std_devs = VisionConstants.K_SINGLE_TAG_STD_DEVS.copy()
            return

        # Pose present. Start running Heuristic
        est_std_devs = VisionConstants.K_SINGLE_TAG_STD_DEVS.copy()
        num_tags = 0
        avg_dist = 0.0

        # Precalculation - see how many tags we found, and calculate average distance
        for tgt in targets:
            tag_pose = pose_estimator.fieldTags.getTagPose(tgt.getFiducialId())
            if tag_pose is None:
                continue
            num_tags += 1
            avg_dist += (
                tag_pose.toPose2d()
                .translation()
                .distance(estimated_pose.estimatedPose.toPose2d().translation())
            )

        if num_tags == 0:
            # No tags visible. Default to single-tag std devs
            self.cur_std_devs = VisionConstants.K_SINGLE_TAG_STD_DEVS.copy()
        else:
            # One or more tags visible, run the full heuristic
            avg_dist /= num_tags

            # Decrease std devs if multiple targets are visible
            if num_tags > 1:
                est_std_devs = VisionConstants.K_MULTI_TAG_STD_DEVS.copy()

            # Increase std devs based on (average) distance
            if num_tags == 1 and avg_dist > 4:
                est_std_devs = [float("inf"), float("inf"), float("inf")]
            else:
                multiplier = 1 + (avg_dist * avg_dist / 30)
                est_std_devs = [x * multiplier for x in est_std_devs]

            self.cur_std_devs = est_std_devs

    def get_estimation_std_devs(self) -> List[float]:
        """
        Returns the latest standard deviations of the estimated pose

        For use with SwerveDrivePoseEstimator. This should only be used
        when there are targets visible.
        """
        return self.cur_std_devs

    def add_vision_measure(
        self, estimated_pose: EstimatedRobotPose, camera_name: str
    ) -> Optional[List[float]]:
        """
        Add vision measurement to pose estimator with dynamic standard deviations

        Args:
            estimated_pose: The estimated robot pose from vision
            camera_name: Name of the camera for logging

        Returns:
            Standard deviations used, or None if measurement was rejected
        """
        pose = estimated_pose.estimatedPose.toPose2d()
        vision_time = estimated_pose.timestampSeconds
        tags = estimated_pose.targetsUsed
        tag_count = len(tags)

        if tag_count == 0:
            return None

        tag_ids = [tag.fiducialId for tag in tags]
        primary_id = tag_ids[0]

        # Get alliance
        alliance = DriverStation.getAlliance()
        if alliance is not None:
            is_blue = alliance == DriverStation.Alliance.kBlue

            # Filter out opponent tags
            if is_blue:
                for tag in tags:
                    tag_id = tag.getFiducialId()
                    if 6 <= tag_id <= 11:
                        return None
            else:
                for tag in tags:
                    tag_id = tag.getFiducialId()
                    if 17 <= tag_id <= 22:
                        return None

        # Check if primary tag is a reef tag
        if self.is_reef_tag(primary_id):
            distance_to_target = (
                tags[0]
                .bestCameraToTarget.translation()
                .toTranslation2d()
                .distance(Translation2d(0, 0))
            )

            std_dev = 2.0
            self.nt.putNumber(f"stdDev/{camera_name}", std_dev)
            self.nt.putNumber(f"tagCount/{camera_name}", tag_count)
            self.nt.putNumber(f"DistanceToTarget/{camera_name}", distance_to_target)

            if tag_count == 1:
                if distance_to_target > 2.5:
                    return None

                if distance_to_target <= 1.5:
                    std_dev = 0.25
                    if distance_to_target <= 0.75:
                        std_dev = 0.1

                    self.drive_sub.add_vision_measurement(
                        pose, vision_time, [std_dev, std_dev, std_dev]
                    )
                    return None

            elif tag_count >= 2:
                # Multi-tag measurement commented out in original
                return None

        return None

    def find_pose_of_tag_closest_to_robot(self, drive_pose: Pose2d) -> Optional[Pose2d]:
        """
        Find the pose of the AprilTag closest to the robot

        Args:
            drive_pose: Current robot pose

        Returns:
            Pose2d of closest tag, or None if no alliance
        """
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
                pose_2d = tag_pose.toPose2d()
                possible_poses.append(pose_2d)
                self.nt.putNumberArray(
                    f"Poses{tag_id}",
                    [pose_2d.X(), pose_2d.Y(), pose_2d.rotation().radians()],
                )

        if not possible_poses:
            return None

        return drive_pose.nearest(possible_poses)

    @staticmethod
    def is_reef_tag(primary_id: int) -> bool:
        """Check if tag ID is a reef tag"""
        return (6 <= primary_id <= 11) or (17 <= primary_id <= 22)

    @staticmethod
    def filter_april_tag_field(
        field: AprilTagFieldLayout,
    ) -> AprilTagFieldLayout:
        """
        Filter AprilTag field to only include reef tags

        Args:
            field: Original field layout

        Returns:
            Filtered field layout with only reef tags
        """
        tags = field.getTags()
        new_tags = []

        for tag in tags:
            if Vision.is_reef_tag(tag.ID):
                new_tags.append(tag)

        return AprilTagFieldLayout(
            new_tags, field.getFieldLength(), field.getFieldWidth()
        )

    def get_all_detected_targets(self) -> List[PhotonTrackedTarget]:
        """Returns all detected targets"""
        return self.all_detected_targets

    def advantage_kit_logging(self):
        """Log data to AdvantageKit/NetworkTables"""
        closest_pose = self.find_pose_of_tag_closest_to_robot(self.drive_sub.get_pose())
        if closest_pose is not None:
            self.nt.putNumberArray(
                "ClosestAprilTag",
                [
                    closest_pose.X(),
                    closest_pose.Y(),
                    closest_pose.rotation().radians(),
                ],
            )

        if self.robot_to_camera is not None:
            pass

    def simulation_periodic(self):
        """Called periodically in simulation"""
        pass
