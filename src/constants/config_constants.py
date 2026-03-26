from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.units import inchesToMeters

_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)

APRILTAG_COUNT = len(_layout.getTags())
APRILTAG_WIDTH = inchesToMeters(6.5)

# Field dimensions
FIELD_LENGTH = _layout.getFieldLength()
FIELD_WIDTH = _layout.getFieldWidth()
