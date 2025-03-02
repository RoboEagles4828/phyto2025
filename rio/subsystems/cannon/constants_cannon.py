from phoenix5 import SupplyCurrentLimitConfiguration


class Constants_Cannon:

    leftMotorID = 14
    rightMotorID = 15

    current_limit = 40
    current_threshold = 60
    current_threshold_time = 3.0

    supply_config = SupplyCurrentLimitConfiguration(
        True, current_limit, current_threshold, current_threshold_time
    )

    loadPercentOutput = 0.5
    placePercentOutput = 0.75
    backupPercentOutput = -0.3
    placeCoralAutoTimeoutSec = 1.0
