from __future__ import annotations
from typing import Protocol, Sequence, runtime_checkable
import dataclasses

from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d, Transform3d, Translation3d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
import json
from typing import Any

WPIStruct = (
    Pose2d
    | Pose3d
    | Rotation2d
    | Translation2d
    | Transform3d
    | ChassisSpeeds
    | SwerveModuleState
    | SwerveModulePosition
    | Translation3d
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
    Translation3d,
    
)
from wpiutil.log import (
    DoubleLogEntry,
    StringLogEntry,
    BooleanLogEntry,
    DoubleArrayLogEntry,
    StructLogEntry,
    StructArrayLogEntry,
)

from wpilib import DataLogManager


from utils.advantagekit import ddatetime_obj
_shared_wpilog_backend = None

def _get_shared_wpilog():
    return _shared_wpilog_backend  # may be None until start_logging() is called
def start_logging(log_dir: str = "logs") -> None:
    """Call this from robotInit() ONLY. Initializes the shared wpilog backend."""
    global _shared_wpilog_backend
    DataLogManager.start(log_dir)
    _shared_wpilog_backend = WPILOGBackend._from_started_manager()


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


class WPILOGBackend:
    def __init__(self):
        # Do NOT call DataLogManager here — use start_logging() from robotInit
        self._log = DataLogManager.getLog()
        self._entries: dict[str, object] = {}

    @classmethod
    def _from_started_manager(cls):
        instance = cls.__new__(cls)
        instance._log = DataLogManager.getLog()
        instance._entries = {}
        return instance

    def _entry(self, key: str, factory):
        if key not in self._entries:
            self._entries[key] = factory(self._log, key)
        return self._entries[key]

    def put_double(self, key: str, value: float) -> None:
        self._entry(key, DoubleLogEntry).append(value)

    def put_string(self, key: str, value: str) -> None:
        self._entry(key, StringLogEntry).append(value)

    def put_boolean(self, key: str, value: bool) -> None:
        self._entry(key, BooleanLogEntry).append(value)

    def put_double_array(self, key: str, value: Sequence[float]) -> None:
        self._entry(key, DoubleArrayLogEntry).append(list(value))

    def put_struct(self, key: str, value: WPIStruct) -> None:
        if key not in self._entries:
            self._entries[key] = StructLogEntry(self._log, key, type(value))
        self._entries[key].append(value)

    def put_struct_array(self, key: str, value: Sequence[WPIStruct]) -> None:
        if key not in self._entries:
            self._entries[key] = StructArrayLogEntry(self._log, key, type(value[0]))
        self._entries[key].append(list(value))

    def flush(self) -> None:
        self._log.flush()
class DualBackend:
    """Send every log to multiple backends at once."""
    def __init__(self, *backends):
        self.backends = backends

    def put_double(self, key: str, value: float):
        for b in self.backends:
            b.put_double(key, value)

    def put_string(self, key: str, value: str):
        for b in self.backends:
            b.put_string(key, value)

    def put_boolean(self, key: str, value: bool):
        for b in self.backends:
            b.put_boolean(key, value)

    def put_double_array(self, key: str, value: list[float]):
        for b in self.backends:
            b.put_double_array(key, value)

    def put_struct(self, key: str, value: Any):
        for b in self.backends:
            b.put_struct(key, value)

    def put_struct_array(self, key: str, value: list[Any]):
        for b in self.backends:
            b.put_struct_array(key, value)

class Logger:
    """Thin logging facade with dataclass publishing and child nesting.

    Usage::

        self.log = Logger("Intake")
        self.log.publish(IntakeTelemetry(velocity=42.0))

        # Or manual puts:
        self.log.put("key", 42.0)
        self.log.child("sub").put("nested_key", 1.0)
    """

    def __init__(self, name: str, backend: Any | None = None, _prefix: str = ""):
        if backend is None:
            nt = NTBackend("AdvantageKit/" + name)
            wpilog = _get_shared_wpilog()
            backend = DualBackend(nt, wpilog) if wpilog is not None else nt
        self._backend = backend
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

    def recordOutput(
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
            if len(value) > 0 and isinstance(value[0], _WPI_STRUCT_TYPES):
                self.put_struct_array(key, value)
            else:
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

