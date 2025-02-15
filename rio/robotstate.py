import math

from gamestate import GameState


class RobotState:
    """
    The single robot can evaluate if the robot subsystems are ready to
    execute a game task.
    """

    kRobotHeadingTolerance = 2.0
    kArmAngleTolerance = 1.0
    kShooterVelocityTolerance = 3.0

    m_gameState = GameState()

    def __new__(cls):
        """
        To initialize the singleton RobotState, call this from
        RobotContainer soon after creating subsystems and
        then immediately call initialize() the returned instance.

        Other clients of the singleton only call this constructor.
        """
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        return cls.instance
    
    def initialize(self, elevatorPositionSupplier, cannonLoadedSupplier):

        self.m_elevatorPositionSupplier = elevatorPositionSupplier
        self.m_cannonLoadedSupplier = cannonLoadedSupplier
    
    def isclose(self, target, actual, tolerance):
        """
        Returns True if the actual value is within the tolerance of the target value.
        """
        return abs(target-actual) < tolerance
    
    def isElevatorReady(self):
        """
        Returns True if the elevator is at the desired position.
        """
        return self.isclose(self.m_gameState.getNextPlacementElevatorPosition(), self.m_elevatorPositionSupplier, 0.5)
    
    def isCannonReady(self):
        """
        Returns True if the cannon is loaded.
        """
        return self.m_cannonLoadedSupplier
    
    def isPathCompleted(self):
        """
        Returns True if the robot has completed the path.
        """
        return self.m_gameState.m_pathCompleted
    
    def isPlacementReady(self):
        """
        Returns True if the robot is ready to place a ball.
        """
        return self.isElevatorReady() and self.isCannonReady() and self.isPathCompleted()
