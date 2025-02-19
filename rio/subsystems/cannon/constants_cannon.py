from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, SupplyCurrentLimitConfiguration
from wpilib import DigitalInput

class Constants_Cannon:

    # TODO: find the values for these constants
    leftMotorID = 31
    rightMotorID = 32

    current_limit = 30
    current_threshold = 50
    current_threshold_time = 3.0
    

    supply_config = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)

    



    

