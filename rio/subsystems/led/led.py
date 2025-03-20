from wpilib import Spark, SmartDashboard
from commands2 import Subsystem
from subsystems.led.led_constants import LED_Constants

from subsystems.robotstate.robotstate import RobotState

from wpilib import DriverStation

class LED(Subsystem):
    def __init__(self):
        self.led = Spark(LED_Constants.kSparkID)

    def set_color(self, value: float):
        return self.runOnce(self.led.set(value))

    def set_default_color(self):
        return self.set_color(LED_Constants.kDefaultColor)
    
    def set_has_coral(self):
        return self.set_color(LED_Constants.kHasCoral)
    
    def set_has_algea(self):
        return self.set_color(LED_Constants.kHasAlgea)
    
    def set_is_ready(self):
        return self.set_color(LED_Constants.kisReady)
    
    def set_is_not_ready(self):
        return self.set_color(LED_Constants.kisNotReady)

    def set_is_aligning(self):
        return self.set_color(LED_Constants.kisAligning)
    
    def set_is_auto(self):
        return self.set_color(LED_Constants.kisAuto)

    def periodic(self):
        SmartDashboard.putNumber("LED/LED Value", self.led.get())

        if DriverStation.isDisabled() or RobotState.getIsZeroed():
            self.set_default_color()
        elif DriverStation.isAutonomous():
            self.set_is_auto()
        elif RobotState.getIsReady():
            self.set_is_ready()
        elif RobotState.getAutoAligning():
            self.set_is_aligning()
        elif RobotState.getCoralInCannon():
            self.set_has_coral()
        else:
            self.set_default_color()    
        


    

