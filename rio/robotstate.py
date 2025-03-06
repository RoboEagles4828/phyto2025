class RobotState:
    
    coralInCannon = True

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        
        return cls.instance
    
    def setCoralInElevator(newCoralInCannon : bool):
        RobotState.coralInCannon = newCoralInCannon