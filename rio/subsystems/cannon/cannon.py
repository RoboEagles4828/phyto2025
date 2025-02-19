from commands2 import Subsystem
from wpilib import DigitalInput
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix5 import FollowerType
from constants_cannon import Constants_Cannon
from commands2.waitcommand import WaitCommand
from wpilib import SmartDashboard

class Cannon(Subsystem):
    def __init__(self):
        self.leftMotor = TalonSRX(Constants_Cannon.leftMotorID)
        self.rightMotor = TalonSRX(Constants_Cannon.rightMotorID)

        self.leftMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)
        self.rightMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)

        #check to see which one should be inverted
        self.leftMotor.setInverted(True)
        self.rightMotor.setInverted(False)

        self.leftMotor.follow(self.rightMotor, FollowerType.PercentOutput)

        self.loaded = False #TODO: Move this variable to the robot state for it to handle
        
    def setCannonSpeed(self, percentOutput):

        """
        Sets the speed for both the left and right
        """

        self.rightMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
    
    def setLeftMotorSpeed(self, percentOutput):
        """
        Sets the speed for the left motor
        """
        self.leftMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
    
    def setRightMotorSpeed(self, percentOutput):
        """
        Sets the speed for the right motor
        """
        self.rightMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
    
    def loadCoral(self):
        """
        This sets the motors to run when the coral is being loaded from the hopper
        """
        loadCommand =  self.run(lambda: self.setCannonSpeed(1)).until(
                       self.rightMotor.getSupplyCurrent()>=40) #TODO: Adjust the current condition
        self.loaded = True
        return loadCommand
        
    
    def scoreCoral(self):
        """
        Outtakes the coral from the cannon
        """ 
        scoreCoral = self.run(lambda: self.setCannonSpeed(1)).withTimeout(2).andThen(
                     lambda: self.stop()) #TODO: Adjust the time
        self.loaded = False
        return scoreCoral
    
    def scoreCoralL1(self):
        """
        Specific to the L1 scoring position. Runs one side faster than the other
        causing the coral to be score horizontally instead of vertically.
        """
        scoreCoralL1 = self.run(self.setLeftMotorSpeed(0.2)).alongWith(
                       self.setRightMotorSpeed(1)).withTimeout(2).andThen(
                       lambda: self.stop()) #TODO: Adjust the time
        self.loaded = False
        return scoreCoralL1

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
    
    def getCannonState(self):
        """
        Returns wether the robot thinks it has a coral in the cannon
        """
        return self.loaded
    
    def periodic(self):
        """
        Puts crucial Cannon related variables to the dashboard
        """

        SmartDashboard.putBoolean("Cannon/HasCoral", self.loaded)
        SmartDashboard.putNumber("Cannon/RightMotorCurrent", self.rightMotor.getSupplyCurrent())
        SmartDashboard.putNumber("Cannon/LeftMotorCurrent", self.leftMotor.getSupplyCurrent())
        SmartDashboard.putNumber("Cannon/RightMotorOutput", self.rightMotor.getMotorOutputPercent())
        SmartDashboard.putNumber("Cannon/LeftMotorOutput", self.leftMotor.getMotorOutputPercent())

