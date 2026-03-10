"""
Lightweight loop profiler for subsystem periodic timing and optional cProfile support.

LoopTimer: Tracks per-subsystem periodic() durations with zero-allocation hot path.
           Logs avg/max every ~1 second under Profiling/ in NetworkTables.

cProfile: Wraps robotPeriodic() when enabled (sim-only by default via RobotFeatures).
         Writes a .prof file on disable that can be viewed with:
           python -m pstats profile.prof
           snakeviz profile.prof
"""

import atexit
import cProfile
import os
import time

from wpilib import RobotBase

from pykit.logger import Logger as PyKitLogger


class LoopTimer:
    """Allocation-free statistical timer for one named section.

    Call `start()` at the top of periodic() and `stop()` at the bottom.
    `stop()` automatically logs avg/max at ~1 Hz.
    """

    __slots__ = (
        "_prefix",
        "_t0",
        "_sum_us",
        "_max_us",
        "_count",
        "_last_log_time",
    )

    def __init__(self, name: str):
        self._prefix = f"Profiling/{name}"
        self._t0: float = 0.0
        self._sum_us: float = 0.0
        self._max_us: float = 0.0
        self._count: int = 0
        self._last_log_time: float = 0.0

    def start(self) -> None:
        self._t0 = time.perf_counter()

    def stop(self) -> None:
        elapsed_us = (time.perf_counter() - self._t0) * 1_000_000.0
        self._sum_us += elapsed_us
        if elapsed_us > self._max_us:
            self._max_us = elapsed_us
        self._count += 1

        now = time.perf_counter()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now

            if self._count > 0:
                avg_ms = (self._sum_us / self._count) / 1000.0
                max_ms = self._max_us / 1000.0
            else:
                avg_ms = 0.0
                max_ms = 0.0

            PyKitLogger.recordOutput(f"{self._prefix}/avg_ms", avg_ms)
            PyKitLogger.recordOutput(f"{self._prefix}/max_ms", max_ms)
            PyKitLogger.recordOutput(f"{self._prefix}/count", float(self._count))

            self._sum_us = 0.0
            self._max_us = 0.0
            self._count = 0


class PeriodicProfiler:
    """Optional cProfile wrapper around robotPeriodic().

    Enable via RobotFeatures.HAS_CPROFILE. Accumulates profiling data across
    the entire session and writes a .prof file on exit.

    View results with:
        python -m pstats logs/<timestamp>/periodic.prof
        snakeviz logs/<timestamp>/periodic.prof
    """

    __slots__ = ("_profiler", "_enabled", "_output_path", "_last_flush_time")

    FLUSH_INTERVAL_S = 10.0

    def __init__(self, enabled: bool = False):
        self._enabled = enabled
        self._profiler: cProfile.Profile | None = None
        self._output_path: str = ""
        self._last_flush_time: float = 0.0
        if enabled:
            self._profiler = cProfile.Profile()
            # Put the .prof next to the other logs
            from utils.advantagekit import ddatetime_obj

            if RobotBase.isReal():
                log_dir = f"/home/lvuser/logs2/{ddatetime_obj}"
            else:
                log_dir = f"logs/{ddatetime_obj}"
            os.makedirs(log_dir, exist_ok=True)
            self._output_path = os.path.join(log_dir, "periodic.prof")
            atexit.register(self._flush)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        if self._profiler is not None:
            self._profiler.enable()

    def stop(self) -> None:
        if self._profiler is not None:
            self._profiler.disable()
            now = time.perf_counter()
            if now - self._last_flush_time >= self.FLUSH_INTERVAL_S:
                self._last_flush_time = now
                self._flush()

    def _flush(self) -> None:
        if self._profiler is not None and self._output_path:
            self._profiler.dump_stats(self._output_path)
            print(f"cProfile data written to {self._output_path}")
