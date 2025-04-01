class RobotState:

    coralInCannon = True
    deAlgaefying = False
    algaeScoring = False
    isautoAligning = False
    isReady = False
    isZeroed = True
    isAlignLeft = True # True if the robot is aligning to the right, False if aligning to the left
    automationMode = True


    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        return cls.instance
    def setCoralInCannon(self, newCoralInCannon : bool):
        RobotState.coralInCannon = newCoralInCannon
    
    def setAutoAligning(self, newAutoAligning : bool):
        RobotState.isautoAligning = newAutoAligning
    
    def setIsReady(self, newIsReady : bool):
        RobotState.isReady = newIsReady
    
    def setIsZeroed(self, newIsZeroed : bool):
        RobotState.isZeroed = newIsZeroed

    def setAlignLeft(self, newAlignLeft : bool):
        RobotState.isAlignLeft = newAlignLeft
        print(RobotState.isAlignLeft)

    def setDeAlgaefyingMode(self):
        RobotState.deAlgaefying = True
        RobotState.algaeScoring = False
    
    def setAlgaeScoringMode(self):
        RobotState.algaeScoring = True
        RobotState.deAlgaefying = False
    
    def setCoralScoringMode(self):
        RobotState.algaeScoring = False
        RobotState.deAlgaefying = False

    def setAutomationMode(self, newAutomationMode : bool):
        RobotState.automationMode = newAutomationMode
    
    def getAlignLeft(self):
        return RobotState.isAlignLeft

    def getCoralInCannon(self):
        return RobotState.coralInCannon
    
    def getAutoAligning(self):
        return RobotState.isautoAligning
    
    def getIsReady(self):
        return RobotState.isReady
    
    def getIsZeroed(self):
        return RobotState.isZeroed
    
    def getDeAlgaefyingMode(self):
        return RobotState.deAlgaefying
    
    def getAlgaeScoringMode(self):
        return RobotState.algaeScoring
    
    def getAutomationMode(self):
        return RobotState.automationMode
    

    
