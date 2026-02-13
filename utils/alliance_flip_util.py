from field_constants import *
from wpimath.geometry import Translation2d, Translation3d, Pose2d, Pose3d, Rotation2d
from wpilib import DriverStation
import math

### Credit to Team 6328 Mechanical Advantage

"""
the code for this entire page is to say what alliance we are on and what way we need to be oriented to account for that alliance
"""


def should_flip():
    """
    Should_flip means that if something is true, that thing should orient the opposite way
    of how it currently is
    """
    return DriverStation.getAlliance() == DriverStation.Alliance().kRed


def get_x(x: float) -> float:
    """
    in this case x is in the context of odometry
    """
    return FIELD_LENGTH - x if should_flip() else x


def get_y(y: float) -> float:
    """
    In this case, y is also in the context of the robot's odometry
    """
    return FIELD_WIDTH - y if should_flip() else y


def get_alliance(translation: Translation2d) -> Translation2d:
    """
    If the should_flip statement is true, then it should generate a new translation which transforms the x and y to the opposites of what they would be otherwise
    """
    return (
        Translation2d(get_x(translation.X()), get_y(translation.Y()))
        if should_flip()
        else translation
    )


def get_alliance(rotation: Rotation2d) -> Rotation2d:
    """
    If should_flip is true here, it makes the angle turn 180 degrees
    """
    return rotation.rotateBy(Rotation2d(math.pi)) if should_flip() else rotation


def get_alliance(pose: Pose2d) -> Pose2d:
    return (
        Pose2d(get_alliance(pose.translation()), get_alliance(pose.rotation()))
        if should_flip()
        else pose
    )


def get_alliance(translation: Translation3d) -> Translation3d:
    return Translation3d(
        get_x(translation.X()), get_y(translation.Y()), translation.Z()
    )


def get_alliance(pose: Pose3d) -> Pose3d:
    return (
        Pose3d(get_alliance(pose.translation()), get_alliance(pose.rotation()))
        if should_flip()
        else pose
    )
