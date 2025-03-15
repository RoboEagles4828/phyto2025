from phoenix5 import SupplyCurrentLimitConfiguration
class AlgaeConstants:
    wheelMotorID = 23 # put in actual thing
    pivotMotorID = 23
    
    current_limit = 30
    current_threshold= 50
    current_threshold_time= 3.0

    supply_config = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)