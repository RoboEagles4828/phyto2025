from commands2 import Subsystem
from phoenix5 import TalonSRX, SupplyCurrentLimitConfiguration, TalonSRXControlMode
from wpilib import DigitalInput

from subsystems.hopper.hopper_constants import Hopper_Constants

class Hopper(Subsystem):

    def __init__(self):
        self.hopperMotor = TalonSRX(Hopper_Constants.hopperMotorID)

        self.hopperBeamBreak = DigitalInput(Hopper_Constants.hopperBeamBreakID)

        currentLimit = 40
        currentThreshold = 60
        currentThresholdTime = 3.0
        supply_config = SupplyCurrentLimitConfiguration(True, currentLimit, currentThreshold, currentThresholdTime)

        self.hopperMotor.configSupplyCurrentLimit(supply_config)

    def setHopperSpeed(self, percentOutput):
        self.hopperMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
    
    def run(self):
        return self.run(lambda: self.setHopperSpeed(1))
    
    def stop(self):
        return self.run(lambda: self.setHopperSpeed(0))
