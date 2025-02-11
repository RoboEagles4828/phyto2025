from pathplannerlib.auto import PathFollowingController
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.config import RobotConfig

class AutoConstants:
    robot_config = RobotConfig.fromGUISettings()

    holonomicPathConfig = PPHolonomicDriveController(
        PIDConstants(5,0,0),
        PIDConstants(10,0,0),
        0.2
    )
    