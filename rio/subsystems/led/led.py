from wpilib import Spark, SmartDashboard
from commands2 import Subsystem
from subsystems.led.led_constants import LED_Constants

from subsystems.robotstate.robotstate import RobotState

from wpilib import DriverStation

class LED(Subsystem):
    def __init__(self):
        self.led = Spark(LED_Constants.CAN)

    def set_color(self, value: float):
        return self.runOnce(self.led.set(value))

    def set_default_color(self):
        return self.set_color(LED_Constants.WHITE)
    
    def set_has_coral(self):
        return self.set_color(LED_Constants.PURPLE)
    
    def set_has_algea(self):
        return self.set_color(LED_Constants.BLUE)
    
    def set_is_ready(self):
        return self.set_color(LED_Constants.GREEN)
    
    def set_is_not_ready(self):
        return self.set_color(LED_Constants.RED)

    def set_is_aligning(self):
        return self.set_color(LED_Constants.RAINBOW_PATTERN)
    
    def set_is_auto(self):
        return self.set_color(LED_Constants.RAINBOW_PATTERN)

    def periodic(self):
        SmartDashboard.putNumber("LED/LED Value", self.led.get())

        if DriverStation.isDisabled() or RobotState.getIsZeroed():
            self.set_default_color()
        elif DriverStation.isAutonomous():
            self.set_is_auto
        elif RobotState.getIsReady():
            self.set_is_ready()
        elif RobotState.getAutoAligning():
            self.set_is_aligning()
        elif RobotState.getCoralInCannon():
            self.set_has_coral()
        else:
            self.set_default_color()    