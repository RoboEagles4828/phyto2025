from commands2 import Subsystem
from wpilib import DigitalInput
from phoenix5 import TalonSRX, TalonSRXControlMode, FollowerType
from subsystems.cannon.constants_cannon import Constants_Cannon
from wpilib import SmartDashboard

class Cannon(Subsystem):
    def __init__(self):
        self.leftMotor = TalonSRX(Constants_Cannon.leftMotorID)
        self.rightMotor = TalonSRX(Constants_Cannon.rightMotorID)

        # if the digital input is false, then it can see the coral
        self.beamBreak = DigitalInput(Constants_Cannon.digitalInputID)

        self.leftMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)
        self.rightMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)

        #check to see which one should be inverted
        self.leftMotor.setInverted(True)
        self.rightMotor.setInverted(False)

        self.leftMotor.follow(self.rightMotor, FollowerType.PercentOutput)

        self.loaded = False

    def getBeamBreakState(self):
        return not(self.beamBreak.get())
        
    def setCannonSpeed(self, percentOutput):

        """
        Sets the speed for both the left and right
        """

        self.rightMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
        

    def loadCoral(self):
        """
        This sets the motors to run when the coral is being loaded from the hopper
        """
        return self.run(lambda: self.setCannonSpeed(1)).until(lambda: self.rightMotor.getSupplyCurrent()>50).andThen(self.hasCoralOverride())
        
    
    def placeCoral(self):
        """
        Outtakes the coral from the cannon
        """ 
        self.loaded = False
        return self.run(lambda: self.setCannonSpeed(1))

    def stop(self):
        """
        Stops the motors
        """
        return self.run(lambda: self.setCannonSpeed(0))

    def hasCoralOverride(self):
        """
        This is used to override the current state of the robot
        """
        self.loaded = not(self.loaded)
    
    def getLoaded(self):
        """
        Returns wether the robot thinks it has a coral in the cannon
        """
        return self.loaded
    
    def placeL1(self):
        return self.run(self.leftMotor.set(TalonSRXControlMode.PercentOutput, 0.2)).alongWith(self.rightMotor.set(TalonSRXControlMode.PercentOutput, 0.2)).until(lambda: self.rightMotor.getSupplyCurrent()>50)
    
    def periodic(self):
        SmartDashboard.putNumber("Cannon/leftMotor", self.leftMotor.getMotorOutputPercent())
        SmartDashboard.putNumber("Cannon/rigthMotor", self.rightMotor.getMotorOutputPercent())
        SmartDashboard.getBoolean("Cannon/loaded", self.loaded)

