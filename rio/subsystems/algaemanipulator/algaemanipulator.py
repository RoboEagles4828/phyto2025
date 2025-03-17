
"""robotcontainer needs to have option for up/down in outtake"""
"""sequences not done"""

from commands2 import Command, ConditionalCommand, Subsystem, InstantCommand
from phoenix5 import TalonSRX, TalonSRXControlMode, LimitSwitchSource, LimitSwitchNormal
from wpimath.filter import Debouncer
from wpilib import DigitalInput
from algaemanipulator_constants import AlgaeConstants

class AlgaeManipulator(Subsystem):

    def __init__(self):
        self.wheelMotor = TalonSRX(AlgaeConstants.kWheelMotorID)
        self.pivotMotor = TalonSRX(AlgaeConstants.kPivotMotorID)
        self.pivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen)
        self.pivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen)

        self.stallDebouncer = Debouncer(0.3, Debouncer.DebounceType.kBoth)

        self.wheelMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)
        self.pivotMotor.configSupplyCurrentLimit(AlgaeConstants.supply_config)

    
    def setSpeed(self, motor  : TalonSRX, percentSpeed):
        """sets motor speed"""
        return motor.set(TalonSRXControlMode.PercentOutput, percentSpeed)
    
    def stop(self, motor : TalonSRX = None) -> Command:   
        """STOP"""
        if motor is None:
            return self.run(        lambda: self.setSpeed(self.pivotMotor, 0.0)) \
                        .alongWith( lambda: self.setSpeed(self.wheelMotor, 0.0))
        else:
            return self.run(lambda: self.setSpeed(motor, 0.0))

    def intake(self) -> Command:
        """spin wheel motor"""
        # figure out speeds and times
        return self.run(       lambda: self.setSpeed(self.wheelMotor, 0.25))\
                 .until(       lambda: self.wheelStall()) \
                 .andThen(     lambda: self.setSpeed(self.pivotMotor, 0.1)) \
                 .withTimeout(0.1)
    
    # intake sequence - pivot starts down. spins wheel until wheel stall is detected, at which point pivot moves up to grip ball
    
    """
    def hold(self) -> Command:
        return self.run(lambda: self.setSpeed(self.wheelMotor, 0.1))"
    """
    
    def outtake(self) -> Command: # down = false/none, up = true
        return self.pivotPosition(False).andThen(lambda: self.setSpeed(self.wheelMotor, -0.2))

    def pivotPosition(self, pos : bool) -> ConditionalCommand: # down = false, up = true
        return ConditionalCommand(
            self.run(lambda: self.setSpeed(self.pivotMotor, 1.0)).until(self.pivotMotor.isFwdLimitSwitchClosed()), # ontrue
            self.run(lambda: self.setSpeed(self.pivotMotor, 1.0)).until(self.pivotMotor.isRevLimitSwitchClosed()), # onfalse
            pos #bool
            )
    
    #def pivotStall(self):
    #    return self.stallDebouncer.calculate(abs(self.pivotMotor.getStatorCurrent()) > 10)

    def wheelStall(self):
        return self.stallDebouncer.calculate(abs(self.wheelMotor.getStatorCurrent()) > 10) # test to figure out current number