from commands2 import Command
from wpilib import Timer
from subsystems import Drivetrain
from utils import NativeHolonomicTrajectory


class FollowTrajectoryCommand(Command):
    def __init__(self, driveSub: Drivetrain, trajectory: NativeHolonomicTrajectory):
        """
        Command to make the robot follow a NativeHolonomicTrajectory.

        :param drivetrain: The drivetrain subsystem to control.
        :param trajectory: The trajectory to follow.
        """
        super().__init__()
        self.driveSub = driveSub
        self.trajectory = trajectory
        self.timer = Timer()

    def initialize(self):
        """Start the timer when the command is initialized."""
        self.timer.reset()
        self.timer.start()

    def execute(self):
        print("HIIII")
        """Sample the trajectory and command the drivetrain."""
        current_time = self.timer.get()
        if current_time > self.trajectory.total_time():
            return  # Prevent sampling beyond the trajectory's total time

        # Get the desired state at the current time
        sample = self.trajectory.sample_at(current_time)
        desired_pose = sample.pose
        desired_chassis_state = sample.chassis_state

        # Command the drivetrain with the desired velocities
        self.driveSub.drive_based_velocity(
            vx=desired_chassis_state.vx,
            vy=desired_chassis_state.vy,
            omega=desired_chassis_state.omega,
        )

    def isFinished(self):
        """End the command when the trajectory is complete."""
        return self.timer.get() >= self.trajectory.total_time()

    def end(self, interrupted):
        """Stop the drivetrain when the command ends."""
        self.driveSub.stop()
