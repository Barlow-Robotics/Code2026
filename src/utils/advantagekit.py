from enum import Enum
from pykit.wpilog.wpilogreader import WPILOGReader

from pykit.logger import Logger as PyKitLogger
from pykit.wpilog.wpilogwriter import WPILOGWriter

from pykit.networktables.nt4Publisher import NT4Publisher
from wpilib import RobotBase
import wpilib
import os


@staticmethod
def configure_pykit(a, obj):
    PyKitLogger.recordMetadata("Robot", a)

    class RobotModes(Enum):
        """Enum for robot modes."""

        REAL = 1
        SIMULATION = 2
        REPLAY = 3

    kSimMode = (
        RobotModes.REPLAY
        if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
        else RobotModes.SIMULATION
    )

    kRobotMode = RobotModes.REAL if RobotBase.isReal() else kSimMode
    useTiming = None
    match kRobotMode:
        case RobotModes.REAL:
            deploy_config = wpilib.deployinfo.getDeployData()
            if deploy_config is not None:
                PyKitLogger.recordMetadata(
                    "Deploy Host", deploy_config.get("deploy-host", "")
                )
                PyKitLogger.recordMetadata(
                    "Deploy User", deploy_config.get("deploy-user", "")
                )
                PyKitLogger.recordMetadata(
                    "Deploy Date", deploy_config.get("deploy-date", "")
                )
                PyKitLogger.recordMetadata(
                    "Code Path", deploy_config.get("code-path", "")
                )
                PyKitLogger.recordMetadata(
                    "Git Hash", deploy_config.get("git-hash", "")
                )
                PyKitLogger.recordMetadata(
                    "Git Branch", deploy_config.get("git-branch", "")
                )
                PyKitLogger.recordMetadata(
                    "Git Description", deploy_config.get("git-desc", "")
                )
            PyKitLogger.addDataReciever(NT4Publisher(True))
            PyKitLogger.addDataReciever(WPILOGWriter())
        case RobotModes.SIMULATION:
            PyKitLogger.addDataReciever(WPILOGWriter(f"logs/{obj}/sim.wpilog"))
            PyKitLogger.addDataReciever(NT4Publisher(True))

        case RobotModes.REPLAY:
            useTiming = False  # Disable timing in replay mode, run as fast as possible
            log_path = os.environ["LOG_PATH"]
            log_path = os.path.abspath(log_path)
            print(f"Starting log from {log_path}")
            PyKitLogger.setReplaySource(WPILOGReader(log_path))
            PyKitLogger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
    PyKitLogger.start()
    return useTiming
