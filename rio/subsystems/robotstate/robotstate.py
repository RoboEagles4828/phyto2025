class RobotState:

    coralInCannon = True
    isautoAligning = False
    isReady = False
    isZeroed = True


    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        return cls.instance
    
    def setCoralInCannon(newCoralInCannon : bool):
        RobotState.coralInCannon = newCoralInCannon
    
    def setAutoAligning(newAutoAligning : bool):
        RobotState.isautoAligning = newAutoAligning
    
    def setIsReady(newIsReady : bool):
        RobotState.isReady = newIsReady
    
    def setIsZeroed(newIsZeroed : bool):
        RobotState.isZeroed = newIsZeroed

    def getCoralInCannon():
        return RobotState.coralInCannon
    
    def getAutoAligning():
        return RobotState.isautoAligning
    
    def getIsReady():
        return RobotState.isReady
    
    def getIsZeroed():
        return RobotState.isZeroed
    
