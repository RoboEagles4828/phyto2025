from phoenix5 import TalonSRX, TalonSRXControlMode
from commands2 import Subsystem

class Hopper(Subsystem):
    def _init_(self):
        self.motor = TalonSRX

    def sethopperspeed(self, percentOutput):
        self.motor.set(TalonSRXControlMode.PercentOutput, percentOutput)

    def stop(self):
        return self.run(lambda: self.sethopperspeed(0))
    
    def move(self):
        return self.run(lambda: self.sethopperspeed(0.1))