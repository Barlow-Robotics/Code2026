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

from utils.advantagekit import ddatetime_obj


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

class FileBackend:
    def __init__(self, filename: str):
        self.filename = filename
        self._buffer = {}

    def _serialize(self, value):
        import dataclasses
        from wpimath.geometry import Pose2d, Rotation2d, Translation2d

        # Dataclasses → dict automatically
        if dataclasses.is_dataclass(value):
            return {f.name: getattr(value, f.name) for f in dataclasses.fields(value)}

        # WPILib geometry structs → dict manually
        if isinstance(value, Pose2d):
            return {"x": value.X(), "y": value.Y(), "rotation_deg": value.rotation().degrees()}
        if isinstance(value, Rotation2d):
            return {"radians": value.radians(), "degrees": value.degrees()}
        if isinstance(value, Translation2d):
            return {"x": value.X(), "y": value.Y()}

        # fallback for normal Python objects
        if hasattr(value, "__dict__"):
            return value.__dict__

        # fallback for unsupported types
        return str(value)

    def _write_file(self):
        with open(self.filename, "w") as f:
            json.dump(self._buffer, f, indent=2)

    def put_double(self, key: str, value: float):
        self._buffer[key] = value
        self._write_file()

    def put_string(self, key: str, value: str):
        self._buffer[key] = value
        self._write_file()

    def put_boolean(self, key: str, value: bool):
        self._buffer[key] = value
        self._write_file()

    def put_double_array(self, key: str, value: list[float]):
        self._buffer[key] = value
        self._write_file()

    def put_struct(self, key: str, value: Any):
        self._buffer[key] = self._serialize(value)
        self._write_file()

    def put_struct_array(self, key: str, value: list[Any]):
        self._buffer[key] = [self._serialize(v) for v in value]
        self._write_file()
        
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
            nt = NTBackend(name)
            file = FileBackend(f"logs/{ddatetime_obj}/{name}_log.json")
            backend = DualBackend(nt, file)
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

