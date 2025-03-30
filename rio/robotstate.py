class RobotState:
    
    coralInCannon = True
    alignRight = False

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        
        return cls.instance
    
    def setCoralInCannon(newCoralInCannon : bool):
        RobotState.coralInCannon = newCoralInCannon

    def setAlignRight(newAlignRight : bool):
        RobotState.alignRight = newAlignRight

    
    def getAlignRight():
        return RobotState.alignRight