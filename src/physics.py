"""
Physics simulation orchestrator.

Delegates to per-subsystem sim modules in the sim/ package.
"""

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from typing import TYPE_CHECKING

from constants import RobotFeatures
from sim import DrivetrainSim, IntakeSim, SpindexSim, FeederSim, ShooterSim, TurretSim

if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    _SIM_PERIOD = 0.004  # 4 ms (250 Hz)

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.drivetrain_sim = DrivetrainSim(robot.container.drivetrain)
        self.intake_sim = (
            IntakeSim(robot.container.intake) if RobotFeatures.HAS_INTAKE else None
        )
        self.spindex_sim = (
            SpindexSim(robot.container.spindex) if RobotFeatures.HAS_SPINDEX else None
        )
        self.feeder_sim = (
            FeederSim(robot.container.feeder) if RobotFeatures.HAS_FEEDER else None
        )
        self.shooter_sim = (
            ShooterSim(robot.container.shooter) if RobotFeatures.HAS_SHOOTER else None
        )
        self.turret_sim = (
            TurretSim(robot.container.turret) if RobotFeatures.HAS_TURRET else None
        )

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
            if self.intake_sim:
                self.intake_sim.update_sim(now, dt)
            if self.spindex_sim:
                self.spindex_sim.update_sim(now, dt)
            if self.feeder_sim:
                self.feeder_sim.update_sim(now, dt)
            if self.shooter_sim:
                self.shooter_sim.update_sim(now, dt)
            if self.turret_sim:
                self.turret_sim.update_sim(now, dt)
            remaining -= dt
