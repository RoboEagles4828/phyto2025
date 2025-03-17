from wpilib import Spark
from commands2 import Subsystem, Command
from led_Constants import ConstantsLED

class LED(Subsystem):
    
    def _init_(self):

        self.spark = Spark(ConstantsLED.CAN)
        self.spark.set(ConstantsLED.WHITE)
    
    def set_color(self, color) -> Command:
       return self.run(lambda: self.spark.set(color))