from wpilib import Spark, SmartDashboard
from commands2 import Subsystem
from subsystems.led.led_constants import LED_Constants

from subsystems.robotstate.robotstate import RobotState

from wpilib import DriverStation

class LED(Subsystem):
    def __init__(self, robotStateManager: RobotState):
        self.led = Spark(LED_Constants.kSparkID)
        self.lastSet = None
        self.robotState = robotStateManager

    def set_color(self, value: float):
        self.lastSet = value
        return self.runOnce(self.led.set(value))

    def set_default_color(self):
        return self.set_color(LED_Constants.kDefaultColor)
    
    def set_has_coral(self):
        return self.set_color(LED_Constants.kHasCoral)
    
    def set_has_algae(self):
        return self.set_color(LED_Constants.kHasAlgae)
    
    def set_is_ready(self):
        return self.set_color(LED_Constants.kisReady)
    
    def set_is_not_ready(self):
        return self.set_color(LED_Constants.kisNotReady)

    def set_is_aligning(self):
        return self.set_color(LED_Constants.kisAligning)
    
    def set_is_auto(self):
        return self.set_color(LED_Constants.kisAuto)

    def periodic(self):
        # SmartDashboard.putNumber("LED/LED Value", self.led.get())
        # SmartDashboard.putBoolean("RobotState/AlgaeScoring Mode", self.robotState.getAlgaeScoringMode())
        # SmartDashboard.putBoolean("RobotState/DeAlgefying Mode", self.robotState.getDeAlgaefyingMode())
        # SmartDashboard.putBoolean("RobotState/Automation Mode", self.robotState.getAutomationMode())
        # SmartDashboard.putBoolean("RobotState/Align Left", self.robotState.getAlignLeft())
        # if DriverStation.isAutonomous() == True:
        #     self.set_is_auto()
        # elif self.robotState.getIsReady():
        #     self.set_is_ready()
        # elif self.robotState.getAutoAligning():
        #     self.set_is_aligning()
        # elif self.robotState.getCoralInCannon():
        #     self.set_has_coral()
        # else:
        #     self.set_default_color()
        pass    