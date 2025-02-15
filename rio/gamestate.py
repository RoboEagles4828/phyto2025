from general_constants.field_constants import FieldConstants, ReefPoints
from wpilib import DriverStation
        
class GameState:


    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(GameState, cls).__new__(cls)
        return cls.instance
    
    m_coralLoaded = True
    m_coralInHopper = False
    m_pathCompleted = False

    #default to point 
    m_nextPlacement = ReefPoints.PointAL1


    def setNextPlacement(self, nextPlacement):
        """
        Sets the next shot to take. If the placement is null, it will default to Point A L1
        """

        if isinstance(nextPlacement, ReefPoints):
            self.m_nextPlacement = nextPlacement
        else:
            self.m_nextPlacement = ReefPoints.PointAL1
    
    def getNextPlacement(self):
        """
        Gets the next placement to be made
        """

        return self.m_nextPlacement
    
    def getNextPlacementElevatorPosition(self):
        """"
        Returns the elevator position of the next placement
        """

        return self.m_nextPlacement.m_elevatorHeight
    
    def getNextPlacementPath(self):
        """
        Returns the path of the next placement
        """

        return self.m_nextPlacement.m_pathName
    
    def getNextPlacementTag(self):
        """
        Return the tag ID for the next shot. This value is alliance adjusted. If
        the FMS is misbehaving, we assume blue.
        """
        alliance = DriverStation.getAlliance()
        nextShot = self.getNextPlacement()
        if alliance == DriverStation.Alliance.kRed:
            return nextShot.m_redTagID
        return nextShot.m_blueTagID
    
    def setHasCoralInHopper(self, coralInHopper):
        self.m_coralInHopper = coralInHopper  

    def hasCoralInHopper(self):
        return self.m_coralInHopper

    def setHasCoralLoaded(self, coralLoaded):
        self.m_coralLoaded = coralLoaded

    def hasCoralLoaded(self):
        return self.m_coralLoaded
    
    def setHasCompletedPath(self, pathCompleted):
        self.m_pathCompleted = pathCompleted


    










