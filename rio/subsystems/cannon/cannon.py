from commands2 import Subsystem, Command
from wpilib import DigitalInput
from phoenix5 import TalonSRX, TalonSRXControlMode, FollowerType
from commands2.conditionalcommand import ConditionalCommand
from subsystems.cannon.constants_cannon import Constants_Cannon
from typing import Callable
from subsystems.robotstate.robotstate import RobotState
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

        self.lastRan = "None"
 
        self.scoringL1 = False
 
    # def getBeamBreakState(self):
    #     return not(self.beamBreak.get())

    def setCannonSpeed(self, percentOutput):

        """
        Sets the speed for both the left and right
        """
        self.leftMotor.follow(self.rightMotor, FollowerType.PercentOutput)
        self.rightMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)
        self.lastRan = "Set cannon speed"

    def loadCoral(self):
        """
        This sets the motors to run when the coral is being loaded from the hopper
        """
        return (
            self.run(lambda: self.setCannonSpeed(0.3))
            .until(self.stopLoading)
            .andThen(self.runOnce(lambda: self.hasCoralOverride(True))).andThen(self.runOnce(lambda: RobotState.setCoralInCannon(True)))
        )

    def createPlaceCoralCommand(self, isL1: Callable[[], bool]) -> Command:
        """
        Creates a command that will run the cannon appropriately for any level.
        :param isL1: a Callable that returns true when the elevator is at L1 height-ish.
        :returns: the placement command
        """

        RobotState.coralInCannon = False
        self.leftMotor.follow(self.rightMotor, FollowerType.PercentOutput)

        return ConditionalCommand(
            self.run(lambda: self._spinForL1()),
            self.run(lambda: self.setCannonSpeed(0.6)),
            isL1,
        ).andThen(self.runOnce(lambda: RobotState.setCoralInCannon(False)))
    
    def _spinForL1(self)-> None:
        """Exists just to make the lambda in createPlaceCoralCommand easy to write and read."""
        self.leftMotor.set(TalonSRXControlMode.PercentOutput, 0.65)
        self.rightMotor.set(TalonSRXControlMode.PercentOutput, 0.0)
        self.lastRan = "_Spin for L1"

    def stop(self):
        """
        Stops the motors
        """
        return self.run(lambda: self.setCannonSpeed(0))

    def stopLoading(self):
        return abs(self.leftMotor.getStatorCurrent())>10

    def hasCoralOverride(self, coral : bool):
        """
        This is used to override the current state of the robot
        """
        if not coral: coral = not self.loaded

        self.loaded = coral
 
    def getLoaded(self):
        """
        Returns wether the robot thinks it has a coral in the cannon
        """
        return self.loaded

    def periodic(self):
        # SmartDashboard.putNumber("Cannon/ Right Stator Current", self.rightMotor.getStatorCurrent())
        # SmartDashboard.putNumber("Cannon/ Left Stator Current", self.leftMotor.getStatorCurrent())
        pass