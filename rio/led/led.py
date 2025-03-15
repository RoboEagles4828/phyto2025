from commands2 import Subsystem
from wpilib import PneumaticsControlModule

class LED(Subsystem):
    def __init__(self):
        self.kLedPort = 9
        self.pcm = PneumaticsControlModule(self.kLedPort)

        self.redChan = self.pcm.makeSolenoid(1)
        self.greenChan = self.pcm.makeSolenoid(0)
        self.blueChan = self.pcm.makeSolenoid(2)
        # self.redChan = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, 1)
        # self.greenChan = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, 0)
        # self.blueChan = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, 2)

        self.BLACK = 0
        self.OFF = 0
        self.RED = 1
        self.YELLOW = 2
        self.GREEN = 3
        self.TEAL = 4
        self.BLUE = 5
        self.PURPLE = 6
        self.WHITE = 7

    def set(self, color): # BLACK - 0, RED - 1, YELLOW - 2, GREEN - 3, TEAL - 4, BLUE - 5, PURPLE - 6, WHITE - 7
        if color == 0: # BLACK / OFF
            self.redChan.set(False)
            self.greenChan.set(False)
            self.blueChan.set(False)
        elif color == 1: # RED
            self.redChan.set(True)
            self.greenChan.set(False)
            self.blueChan.set(False)
        elif color == 2: # YELLOW
            self.redChan.set(True)
            self.greenChan.set(True)
            self.blueChan.set(False)
        elif color == 3: # GREEN
            self.redChan.set(False)
            self.greenChan.set(True)
            self.blueChan.set(False)
        elif color == 4: # TEAL
            self.redChan.set(False)
            self.greenChan.set(True)
            self.blueChan.set(True)
        elif color == 5: # BLUE
            self.redChan.set(False)
            self.greenChan.set(False)
            self.blueChan.set(True)
        elif color == 6: # PURPLE
            self.redChan.set(True)
            self.greenChan.set(False)
            self.blueChan.set(True)
        elif color == 7: # WHITE / ON
            self.redChan.set(True)
            self.greenChan.set(True)
            self.blueChan.set(True)


        self.last = None

    def red(self):
        self.set(1)
        print("Changed color to RED")

    def yellow(self):  
        self.set(2)
        print("Changed color to YELLOW")

    def green(self):
        self.set(3)
        print("Changed color to GREEN")

    def teal(self):
        self.set(4)
        print("Changed color to TEAL")

