from commands2 import Subsystem
from wpilib import DigitalInput
from phoenix5 import TalonSRX, TalonSRXControlMode, FollowerType
from subsystems.cannon.constants_cannon import Constants_Cannon
from wpilib import SmartDashboard
from robotstate import RobotState

class Cannon(Subsystem):
    def __init__(self):
        self.leftMotor = TalonSRX(Constants_Cannon.leftMotorID)
        self.rightMotor = TalonSRX(Constants_Cannon.rightMotorID)

        # if the digital input is false, then it can see the coral
        # self.beamBreak = DigitalInput(Constants_Cannon.digitalInputID)

        self.leftMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)
        self.rightMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)

        # check to see which one should be inverted
        self.leftMotor.setInverted(True)
        self.rightMotor.setInverted(False)

        self.leftMotor.follow(self.rightMotor, FollowerType.PercentOutput)

        self.state = RobotState()

    # def getBeamBreakState(self):
    #     return not(self.beamBreak.get())

    def setCannonSpeed(self, percentOutput):

        """
        Sets the speed for both the left and right
        """

        self.rightMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)

    def loadCoral(self):
        """
        This sets the motors to run when the coral is being loaded from the hopper
        """
        return (
            self.run(lambda: self.setCannonSpeed(0.3))
            .until(self.stopLoading)
            .andThen(self.runOnce(self.hasCoralOverride))
        )

    def placeCoral(self):
        """
        Outtakes the coral from the cannon
        """ 
        self.state.loaded = True
        return self.run(lambda: self.setCannonSpeed(1))

    def stop(self):
        """
        Stops the motors
        """
        return self.run(lambda: self.setCannonSpeed(0))

    def stopLoading(self):
        return abs(self.rightMotor.getStatorCurrent())>10

    def hasCoralOverride(self):
        """
        This is used to override the current state of the robot
        """
        self.state.loaded = False

    def getLoaded(self):
        """
        Returns wether the robot thinks it has a coral in the cannon
        """
        return self.state.loaded

    def placeL1(self):
        return self.run(self.leftMotor.set(TalonSRXControlMode.PercentOutput, 0.2)).alongWith(self.rightMotor.set(TalonSRXControlMode.PercentOutput, 0.2)).until(lambda: self.rightMotor.getSupplyCurrent()>50)

    def periodic(self):
        # SmartDashboard.putNumber("Cannon/leftMotorPercentOut", self.leftMotor.getMotorOutputPercent())
        # SmartDashboard.putNumber("Cannon/rigthMotorPercentOut", self.rightMotor.getMotorOutputPercent())
        SmartDashboard.putBoolean("Cannon/loaded", self.state.loaded)
        SmartDashboard.putNumber("Cannon/leftStatorCurrent", self.leftMotor.getStatorCurrent())
        SmartDashboard.putNumber("Cannon/rightStatorCurrent", self.rightMotor.getStatorCurrent())
        SmartDashboard.putNumber("Cannon/leftSupplyCurrent", self.leftMotor.getSupplyCurrent())
        SmartDashboard.putNumber("Cannon/rightSupplyCurrent", self.rightMotor.getSupplyCurrent())
        # SmartDashboard.putNumber("Cannon/leftMotorSensorVelocity", self.leftMotor.getSelectedSensorVelocity())
        # SmartDashboard.putNumber("Cannon/rightMotorSensorVelocity", self.rightMotor.getSelectedSensorVelocity())
