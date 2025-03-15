from phoenix5 import SupplyCurrentLimitConfiguration
class AlgaeConstants:
    kWheelMotorID = 23 # put in actual thing
    kPivotMotorID = 23
    
    kBottomLimitSwitch = 0 # put in actual thing
    kTopLimitSwitch = 1

    current_limit = 30
    current_threshold= 50
    current_threshold_time= 3.0

    supply_config = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)