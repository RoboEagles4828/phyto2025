from enum import Enum

from subsystems.elevator.elevator_constants import Elevator_Constants
from subsystems.elevator.reef import Reef


class Floor(Enum):
    """
    An enumeration of known elevator stops (floors) with rotations for reaching the floor.
    """

    START = "START", Elevator_Constants.CARRIAGE_HEIGHT_AT_BOTTOM_M
    INTAKE = "INTAKE", Elevator_Constants.CARRIAGE_HEIGHT_AT_BOTTOM_M
    L1 = "L1", Reef.L1_SCORE_HEIGHT_M
    L2 = "L2", Reef.L2_SCORE_HEIGHT_M
    L3 = "L3", Reef.L3_SCORE_HEIGHT_M
    L4 = "L4", Reef.L4_SCORE_HEIGHT_M

    def __init__(self, value, height):
        self._value_ = value
        self.rotations = (
            height - Elevator_Constants.CARRIAGE_HEIGHT_AT_BOTTOM_M
        ) / Elevator_Constants.MECH_M_PER_ROT