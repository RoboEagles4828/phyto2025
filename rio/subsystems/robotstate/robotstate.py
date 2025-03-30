class RobotState:

    coralInCannon = True
    isautoAligning = False
    isReady = False
    isZeroed = True
    isAlignLeft = True # True if the robot is aligning to the right, False if aligning to the left


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

    def isReadyToIntake(self):
        RobotState.isReady = False
        RobotState.isZeroed = True
        RobotState.coralInCannon = False
        RobotState.isautoAligning = False
    
    def isReadyToShoot(self):
        RobotState.isReady = True
        RobotState.isZeroed = False
        RobotState.coralInCannon = True
        RobotState.isautoAligning = False
    
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
    
