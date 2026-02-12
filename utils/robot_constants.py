from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from wpimath import units
# from photonlibpy.photonPoseEstimator import PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
import math

class DriveConstants:
    TOTAL_WIDTH_INCHES = 27.0
    TOTAL_WIDTH_INCHES_BUMPERS = 34.5


class VisionConstants:
    """Vision subsystem constants"""
    
    # Camera configuration
    CAMERA_LIGHT_ID = 0  # NEED TO FIX/CHANGE
    BACK_LEFT_SWERVE_NAME = "Back_Left_Swerve" 
    FRONT_LEFT_SWERVE_NAME = "Front_Left_Swerve"
    BACK_RIGHT_SWERVE_NAME = "Back_Right_Swerve"
    FRONT_RIGHT_SWERVE_NAME = "Front_Right_Swerve"

    K_SINGLE_TAG_STD_DEVS = [4.0, 4.0, 8.0]
    K_MULTI_TAG_STD_DEVS = [0.5, 0.5, 1.0]

    # Vision strategies
        
    BACK_LEFT_SWERVE_TO_ROBOT = Transform3d( # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 12.625),
            units.inchesToMeters((-1.0 * (DriveConstants.TOTAL_WIDTH_INCHES / 2) + 2.5)),
            units.inchesToMeters(12)
        ),
        Rotation3d(0, units.degreesToRadians(0), 0)
    )
    
    BACK_RIGHT_SWERVE_TO_ROBOT = Transform3d( # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625)
        ),
        Rotation3d(0, units.degreesToRadians(0), 0)
    )
    FRONT_LEFT_SWERVE_TO_ROBOT = Transform3d( # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625)
        ),
        Rotation3d(0, units.degreesToRadians(0), 0)
    )
    FRONT_RIGHT_SWERVE_TO_ROBOT = Transform3d( # BW: NEED TO FIX
        Translation3d(
            units.inchesToMeters(DriveConstants.TOTAL_WIDTH_INCHES / 2 - 2.5),
            units.inchesToMeters((DriveConstants.TOTAL_WIDTH_INCHES / 2 - 9.25)),
            units.inchesToMeters(12.625)
        ),
        Rotation3d(0, units.degreesToRadians(0), 0)
    )

    
    
    # Look at, call Vision.filter_april_tag_field() 
        
    NULL_APRIL_TAG_ID = -1
    INVALID_ANGLE = -361.0
    NO_TARGET_DISTANCE = -1.0
    
    AUTO_ALIGN_VELOCITY_CONSTANT = 3.0
    AUTO_ALIGN_ANGULAR_VELOCITY_CONSTANT = units.degreesToRadians(180.0)  # wpk is this a good amount?
    
    # AprilTag lists for different game pieces and alliances NEED TO CHNAGE
    BLUE_APRIL_TAG_LIST_REEF = [17, 18, 19, 20, 21, 22]
    RED_APRIL_TAG_LIST_REEF = [6, 7, 8, 9, 10, 11]
    
    BLUE_APRIL_TAG_LIST_CORAL_STATION = [12, 13]
    RED_APRIL_TAG_LIST_CORAL_STATION = [1, 2]

    NOTE_ALIGN_PIXEL_TOLERANCE = 250.0  # NEED TO CHANGE