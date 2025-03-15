
from commands2 import Command, ConditionalCommand, Subsystem
from phoenix5 import TalonSRX, TalonSRXControlMode
from elevator.elevator import Elevator
from wpimath.filter import Debouncer
from algaemanipulator_constants import AlgaeConstants

class AlgaeManipulator(Subsystem):

    def __init__(self):
        self.wheelMotor = TalonSRX(AlgaeConstants.wheelMotorID)
        self.pivotMotor = TalonSRX(AlgaeConstants.pivotMotorID)

        self.stallDebouncer = Debouncer(0.3, Debouncer.DebounceType.kBoth)
        self.wheelMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)
        self.pivotMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)
    
    def setSpeed(self, motor : TalonSRX, percentSpeed):
        """sets motor speed"""
        return motor.set(TalonSRXControlMode.PercentOutput, percentSpeed)
    
    def stop(self, motor : TalonSRX) -> Command:   
        """STOP"""
        return self.runOnce(lambda: self.setSpeed(motor, 0.0))

    def intake(self) -> Command:
        """spin wheel motor"""
        return self.run(lambda: self.setSpeed(self.wheelMotor, 1.0))
    
    def pivotPosition(self, pos : bool) -> ConditionalCommand: # down = false, up = true
        return ConditionalCommand(
            self.run(lambda: self.setSpeed(self.pivotMotor(0.5))), # ontrue
            self.run(lambda: self.setSpeed(self.pivotMotor(-0.5))), # onfalse
            pos #bool
            )
    
    def outtake(self) -> Command:
        return self.run(lambda: self.setSpeed(self.wheelMotor, -1.0))

    def pivotStall(self):
        return self.stallDebouncer.calculate(self.pivotMotor.getStatorCurrent() < 10)

    def periodic(self):
        if self.pivotStall() == True:
            self.stop(self.pivotMotor)
        pass
        