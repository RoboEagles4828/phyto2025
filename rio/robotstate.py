class RobotState:
    
    coralInElevator = True

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        
        return cls.instance
    
    def setCoralInElevator(newCoralInElevator):
        RobotState.coralInElevator = newCoralInElevator