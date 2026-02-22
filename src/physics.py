"""
Physics simulation orchestrator.

Delegates to per-subsystem sim modules in the sim/ package.
"""

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from typing import TYPE_CHECKING

from sim import DrivetrainSim

if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    _SIM_PERIOD = 0.004  # 4 ms (250 Hz)

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drivetrain_sim = DrivetrainSim(robot.container.drivetrain)
        # self.intake_sim = IntakeSim(robot.container.intake)
        # self.spindex_sim = SpindexSim(robot.container.spindex)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called by pyfrc ~50 times per second (20ms intervals).
        Subdivides into 4ms steps so all sims run at 250 Hz.
        """
        # Feed the Phoenix6 simulation - required for motor controllers to work
        unmanaged.feed_enable(100)  # Keep motors enabled for 100ms

        remaining = tm_diff
        while remaining > 0:
            dt = min(self._SIM_PERIOD, remaining)
            self.drivetrain_sim.update_sim(now, dt)
            # self.intake_sim.update_sim(now, dt)
            # self.spindex_sim.update_sim(now, dt)
            remaining -= dt
