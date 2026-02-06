import ntcore

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.geometry import Transform3d, Pose3d, Translation2d
from wpilib import TimedRobot
from utils.field_constants import _layout

class PhotonCamCustom:
    def __init__(self, name: str, robot_to_camera: Transform3d):
        self.cam = PhotonCamera(name)
        self.name = name
        self.robot_to_camera = robot_to_camera
        self.estimator = PhotonPoseEstimator(
            _layout,
            # PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam,
            self.robot_to_camera,
        )
        # self.estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY
        self.table = (
            ntcore.NetworkTableInstance.getDefault()
            .getTable("Cameras")
            .getSubTable(self.name)
        )

    def init(self):
        pass

    def update_tables(self):
        if not TimedRobot.isSimulation():
            result = self.cam.getLatestResult()
            camEstPose = self.estimator.estimateCoprocMultiTagPose(result)
            if camEstPose is None:
                camEstPose = self.estimator.estimateLowestAmbiguityPose(result)

            if camEstPose:
                estimatedPose = camEstPose.estimatedPose.toPose2d()
                self.table.putNumberArray(
                    "estimated pose",
                    [
                        estimatedPose.X(),
                        estimatedPose.Y(),
                        estimatedPose.rotation().radians(),
                    ],
                )

            self.table.putBoolean("has target", result.hasTargets())
            if result.hasTargets():
                self.table.putNumberArray(
                    "ids", [target.getFiducialId() for target in result.getTargets()]
                )
                self.table.putNumber(
                    "distance to closest target",
                    result.getBestTarget()
                    .bestCameraToTarget.translation()
                    .toTranslation2d()
                    .distance(Translation2d(0, 0)),
                )

    def get_estimated_robot_pose(self) -> Pose3d:
        """
        Returns a Pose3d of the estimated robot position
        """
        result = self.cam.getLatestResult()
        camEstPose = self.estimator.estimateCoprocMultiTagPose(result)
        if camEstPose is None:
            camEstPose = self.estimator.estimateLowestAmbiguityPose(result)

        return camEstPose.estimatedPose

    def get_result(self) -> EstimatedRobotPose | None:
        """
        Returns an EstimatedRobotPose, which includes pose, timestamp, tags, and strategy
        """
        result = self.cam.getLatestResult()
        camEstPose = self.estimator.estimateCoprocMultiTagPose(result)
        if camEstPose is None:
            camEstPose = self.estimator.estimateLowestAmbiguityPose(result)

        return camEstPose


class PhotonController:
    def __init__(self, cams: list[PhotonCamCustom | None]):
        self.cams = cams

    def init(self):
        pass

    def update_tables(self):
        for cam in self.cams:
            cam.update_tables()

    def get_results(self) -> list[EstimatedRobotPose | None]:
        return [cam.get_result() for cam in self.cams]
