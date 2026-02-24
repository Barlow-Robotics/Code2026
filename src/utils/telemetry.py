from __future__ import annotations
from typing import Protocol, Sequence, runtime_checkable

from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d, Transform3d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition

WPIStruct = (
    Pose2d
    | Pose3d
    | Rotation2d
    | Translation2d
    | Transform3d
    | ChassisSpeeds
    | SwerveModuleState
    | SwerveModulePosition
)

_WPI_STRUCT_TYPES = (
    Pose2d,
    Pose3d,
    Rotation2d,
    Translation2d,
    Transform3d,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
)


@runtime_checkable
class LogBackend(Protocol):
    def put_double(self, key: str, value: float) -> None: ...
    def put_string(self, key: str, value: str) -> None: ...
    def put_boolean(self, key: str, value: bool) -> None: ...
    def put_double_array(self, key: str, value: Sequence[float]) -> None: ...
    def put_struct(self, key: str, value: WPIStruct) -> None: ...
    def put_struct_array(self, key: str, value: Sequence[WPIStruct]) -> None: ...


class NTBackend:
    """NetworkTables backend — caches publishers on first use."""

    def __init__(self, table_name: str):
        self._table = NetworkTableInstance.getDefault().getTable(table_name)
        self._double_pubs: dict[str, object] = {}
        self._string_pubs: dict[str, object] = {}
        self._boolean_pubs: dict[str, object] = {}
        self._double_array_pubs: dict[str, object] = {}
        self._struct_pubs: dict[str, object] = {}
        self._struct_array_pubs: dict[str, object] = {}

    def put_double(self, key: str, value: float) -> None:
        pub = self._double_pubs.get(key)
        if pub is None:
            pub = self._table.getDoubleTopic(key).publish()
            self._double_pubs[key] = pub
        pub.set(value)

    def put_string(self, key: str, value: str) -> None:
        pub = self._string_pubs.get(key)
        if pub is None:
            pub = self._table.getStringTopic(key).publish()
            self._string_pubs[key] = pub
        pub.set(value)

    def put_boolean(self, key: str, value: bool) -> None:
        pub = self._boolean_pubs.get(key)
        if pub is None:
            pub = self._table.getBooleanTopic(key).publish()
            self._boolean_pubs[key] = pub
        pub.set(value)

    def put_double_array(self, key: str, value: Sequence[float]) -> None:
        pub = self._double_array_pubs.get(key)
        if pub is None:
            pub = self._table.getDoubleArrayTopic(key).publish()
            self._double_array_pubs[key] = pub
        pub.set(value)

    def put_struct(self, key: str, value: WPIStruct) -> None:
        pub = self._struct_pubs.get(key)
        if pub is None:
            pub = self._table.getStructTopic(key, type(value)).publish()
            self._struct_pubs[key] = pub
        pub.set(value)

    def put_struct_array(self, key: str, value: Sequence[WPIStruct]) -> None:
        pub = self._struct_array_pubs.get(key)
        if pub is None:
            pub = self._table.getStructArrayTopic(key, type(value[0])).publish()
            self._struct_array_pubs[key] = pub
        pub.set(value)
