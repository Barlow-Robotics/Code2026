"""
Lightweight loop profiler for subsystem periodic timing and optional pyinstrument support.

LoopTimer: Tracks per-subsystem periodic() durations with zero-allocation hot path.
           Logs avg/max every ~1 second under Profiling/ in NetworkTables.

pyinstrument: Statistical profiler (~1ms sampling) enabled via RobotFeatures.HAS_CPROFILE.
              Starts on teleopInit, stops on disable, writes an interactive HTML report.
              Open the .html file in a browser to view the flamegraph.
"""

import atexit
import os
import time

from wpilib import RobotBase

from pykit.logger import Logger as PyKitLogger

from constants.robot_constants import RobotFeatures


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
            if self._prefix == 'Profiling/Scheduler' and RobotFeatures.LOW_LOGGING:
                PyKitLogger.recordOutput(f"{self._prefix}/avg_ms", avg_ms)
                # PyKitLogger.recordOutput(f"{self._prefix}/max_ms", max_ms)
                # PyKitLogger.recordOutput(f"{self._prefix}/count", float(self._count))
            # elif not RobotFeatures.LOW_LOGGING:
            #     PyKitLogger.recordOutput(f"{self._prefix}/avg_ms", avg_ms)
            #     PyKitLogger.recordOutput(f"{self._prefix}/max_ms", max_ms)
            #     PyKitLogger.recordOutput(f"{self._prefix}/count", float(self._count))

            self._sum_us = 0.0
            self._max_us = 0.0
            self._count = 0


class PeriodicProfiler:
    """Optional pyinstrument profiler for robotPeriodic().

    Enable via RobotFeatures.HAS_CPROFILE. Samples the call stack at ~1ms
    intervals (low overhead). Starts on teleopInit, stops on disable.

    View results by opening the .html file in a browser.
    """

    __slots__ = ("_profiler", "_enabled", "_output_dir", "_session")

    def __init__(self):
        self._enabled = False
        self._profiler = None
        self._session: int = 0
        from utils.advantagekit import ddatetime_obj

        if RobotBase.isReal():
            self._output_dir = f"/home/lvuser/logs2/{ddatetime_obj}"
        else:
            self._output_dir = f"logs/{ddatetime_obj}"
        os.makedirs(self._output_dir, exist_ok=True)
        atexit.register(self._flush)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        from pyinstrument import Profiler

        self._profiler = Profiler()
        self._enabled = True
        self._session += 1
        self._profiler.start()

    def disable(self) -> None:
        if not self._enabled:
            return
        self._enabled = False
        self._profiler.stop()
        self._flush()
        self._profiler = None

    def _flush(self) -> None:
        if self._profiler is not None and self._output_dir:
            output_path = f"{self._output_dir}/periodic_{self._session}.html"
            with open(output_path, "w", encoding="utf-8") as f:
                f.write(self._profiler.output_html())
            print(f"Profile written to {output_path}")
            if not RobotBase.isReal():
                import webbrowser

                webbrowser.open(os.path.abspath(output_path))
