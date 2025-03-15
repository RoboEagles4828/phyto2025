from commands2 import Subsystem
from wpilib import Spark

class LED(Subsystem):  
    def __init__(self):
        self.spark = Spark(9)

    def green(self):
        self.spark.set(0.75)
        
