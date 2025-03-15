
from wpilib import Subsystem
from commands2 import Command
from phoenix5 import TalonSRX, TalonSRXControlMode
from elevator.elevator import Elevator
from wpimath.filter import Debouncer
from algaemanipulator_constants import AlgaeConstants

class AlgaeManipulator(Subsystem):

    def __init__(self):
        self.wheelMotor = TalonSRX(AlgaeConstants.wheelMotorID)
        self.pivotMotor = TalonSRX(AlgaeConstants.pivotMotorID)

        self.stallDebouncer = Debouncer(0.5, Debouncer.DebounceType.kBoth)
        self.wheelMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)
        self.pivotMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)
    
    def setSpeed(self, motor : TalonSRX, percentSpeed):
        """sets motor speed"""
        return motor.set(TalonSRXControlMode.PercentOutput, percentSpeed)
    
    def stop(self : TalonSRX):
        self.set(TalonSRXControlMode.PercentOutput, 0.0)

    def intake(self, level):
        self.setSpeed(self.pivotMotor, 0.5)
        self.setSpeed(self.wheelMotor, 1.0)
    
    def outtake(self):
        self.setSpeed(self.wheelMotor, -1.0)

    def pivotStall(self):
        return self.stallDebouncer.calculate(self.pivotMotor.getStatorCurrent() < 10)

    def periodic(self):
        if self.pivotStall() == True:
            self.pivotMotor.stop()
        