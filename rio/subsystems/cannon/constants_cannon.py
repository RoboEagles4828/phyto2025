from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, SupplyCurrentLimitConfiguration
from wpilib import DigitalInput

class Constants_Cannon:

    leftMotorID = 14
    rightMotorID = 15
    digitalInputID = 0

    current_limit = 40
    current_threshold = 60
    current_threshold_time = 3.0
    

    supply_config = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)

    



    

