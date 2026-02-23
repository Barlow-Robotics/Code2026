from __future__ import annotations

import dataclasses
from typing import Protocol, Sequence, runtime_checkable

from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d, Transform3d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from pykit.logger import Logger as PyKitLogger

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

class PyKitBackend(LogBackend):
    def __init__(self, name: str):
        self._name = name

    def _key(self, key: str) -> str:
        return f"{self._name}/{key}"

    def put_double(self, key: str, value: float) -> None:
        PyKitLogger.recordOutput(self._key(key), value)

    def put_string(self, key: str, value: str) -> None:
        PyKitLogger.recordOutput(self._key(key), value)

    def put_boolean(self, key: str, value: bool) -> None:
        PyKitLogger.recordOutput(self._key(key), value)

    def put_double_array(self, key: str, value) -> None:
        PyKitLogger.recordOutput(self._key(key), list(value))

    def put_struct(self, key: str, value) -> None:
        PyKitLogger.recordOutput(self._key(key), value)

    def put_struct_array(self, key: str, value) -> None:
        PyKitLogger.recordOutput(self._key(key), list(value))


class Logger:
    def __init__(self, name: str, backend: LogBackend | None = None, _prefix: str = ""):
        self._backend = backend or PyKitBackend(name) 
        self._prefix = _prefix

    def child(self, name: str) -> Logger:
        """Create a nested logger. Logger("Vision").child("FrontLeft") → /Vision/FrontLeft/*"""
        new_prefix = f"{self._prefix}{name}/" if self._prefix else f"{name}/"
        return Logger("", backend=self._backend, _prefix=new_prefix)

    def _full_key(self, key: str) -> str:
        return f"{self._prefix}{key}"

    # --- explicit typed methods ---

    def put_double(self, key: str, value: float) -> None:
        self._backend.put_double(self._full_key(key), value)

    def put_string(self, key: str, value: str) -> None:
        self._backend.put_string(self._full_key(key), value)

    def put_boolean(self, key: str, value: bool) -> None:
        self._backend.put_boolean(self._full_key(key), value)

    def put_double_array(self, key: str, value: Sequence[float]) -> None:
        self._backend.put_double_array(self._full_key(key), value)

    def put_struct(self, key: str, value: WPIStruct) -> None:
        self._backend.put_struct(self._full_key(key), value)

    def put_struct_array(self, key: str, value: Sequence[WPIStruct]) -> None:
        self._backend.put_struct_array(self._full_key(key), value)

    # --- convenience put() with type dispatch ---

    def put(
        self, key: str, value: float | bool | str | Sequence[float] | WPIStruct
    ) -> None:
        if isinstance(value, bool):
            self.put_boolean(key, value)
        elif isinstance(value, (int, float)):
            self.put_double(key, float(value))
        elif isinstance(value, str):
            self.put_string(key, value)
        elif isinstance(value, _WPI_STRUCT_TYPES):
            self.put_struct(key, value)
        elif isinstance(value, (list, tuple)):
            self.put_double_array(key, value)
        else:
            raise TypeError(f"Logger.put: unsupported type {type(value)}")

    # --- dataclass publishing ---

    def publish(self, data: object) -> None:
        """Publish all fields of a dataclass to the backend.

        Recursively walks nested dataclasses. None fields are skipped.
        Construct the dataclass at the call site with no defaults so the
        linter enforces completeness.
        """
        for field in dataclasses.fields(data):
            value = getattr(data, field.name)

            if value is None:
                continue
            elif isinstance(value, bool):
                self.put_boolean(field.name, value)
            elif isinstance(value, (int, float)):
                self.put_double(field.name, float(value))
            elif isinstance(value, str):
                self.put_string(field.name, value)
            elif isinstance(value, _WPI_STRUCT_TYPES):
                self.put_struct(field.name, value)
            elif isinstance(value, (list, tuple)):
                if len(value) > 0 and isinstance(value[0], _WPI_STRUCT_TYPES):
                    self.put_struct_array(field.name, value)
                else:
                    self.put_double_array(field.name, value)
            elif dataclasses.is_dataclass(value):
                self.child(field.name).publish(value)
        
            from pykit.logger import Logger as PyKitLogger
            key = f"{self._prefix}{field.name}"
            PyKitLogger.recordOutput(key, value)


