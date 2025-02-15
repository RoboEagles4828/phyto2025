from wpimath.geometry import Pose2d, Rotation2d
from lib.util.units import Units
from enum import Enum
import mmap
from subsystems.elevator.elevator_constants import Elevator_Constants

class FieldConstants:
    processorPositionBlue = Pose2d(5.987542, 0.5, Rotation2d(90))
    processorPositionRed = Pose2d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Rotation2d(270))
    topCoralStationPositionBlue = Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Rotation2d(306))
    topCoralStationPositionRed = Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Rotation2d(234))
    bottomCoralStationPositionBlue = Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Rotation2d(54))
    bottomCoralStationPositionRed = Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Rotation2d(126))

    reefFaceOneBlue = Pose2d(Units.inchesToMeters(144.0), Units.inchesToMeters(158.50), Rotation2d(180))
    reefFaceOneRed = Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d(0))
    reefFaceTwoBlue = Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d(240))
    reefFaceTwoRed = Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d(60))
    reefFaceThreeBlue = Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d(300))
    reefFaceThreeRed = Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d(120))
    reefFaceFourBlue = Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d(0))
    reefFaceFourRed = Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d(180))
    reefFaceFiveBlue = Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d(60))
    reefFaceFiveRed = Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d(240))
    reefFaceSixBlue = Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d(120))
    reefFaceSixRed = Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d(300))

class ReefPoints(Enum):
    # point = (elevator height, "path name", red tag, blue tag)
    PointAL1 = (0, 0, 26-Elevator_Constants.kbaseHeight, "pointAPath", 7, 18)  
    PointAL2 = (1, 1, 34-Elevator_Constants.kbaseHeight, "pointAPath", 7, 18)
    PointAL3 = (2, 2, 50-Elevator_Constants.kbaseHeight, "pointAPath", 7, 18)
    PointAL4 = (3, 3, 77-Elevator_Constants.kbaseHeight, "pointAPath", 7, 18) 

    PointBL1 = (4, 4, 26-Elevator_Constants.kbaseHeight, "pointBPath", 7, 18)
    PointBL2 = (5, 5, 34-Elevator_Constants.kbaseHeight, "pointBPath", 7, 18)
    PointBL3 = (6, 6, 50-Elevator_Constants.kbaseHeight, "pointBPath", 7, 18)
    PointBL4 = (7, 7, 77-Elevator_Constants.kbaseHeight, "pointBPath", 7, 18)

    PointCL1 = (8, 26-Elevator_Constants.kbaseHeight, "pointCPath", 8, 17)
    PointCL2 = (9, 34-Elevator_Constants.kbaseHeight, "pointCPath", 8, 17)
    PointCL3 = (10, 50-Elevator_Constants.kbaseHeight, "pointCPath", 8, 17)
    PointCL4 = (11, 77-Elevator_Constants.kbaseHeight, "pointCPath", 8, 17)

    PointDL1 = (12, 26-Elevator_Constants.kbaseHeight, "pointDPath", 8, 17)
    PointDL2 = (13, 34-Elevator_Constants.kbaseHeight, "pointDPath", 8, 17)
    PointDL3 = (14, 50-Elevator_Constants.kbaseHeight, "pointDPath", 8, 17)
    PointDL4 = (15, 77-Elevator_Constants.kbaseHeight, "pointDPath", 8, 17)

    PointEL1 = (16, 26-Elevator_Constants.kbaseHeight, "pointEPath", 9, 22)
    PointEL2 = (17, 34-Elevator_Constants.kbaseHeight, "pointEPath", 9, 22)
    PointEL3 = (18, 50-Elevator_Constants.kbaseHeight, "pointEPath", 9, 22)
    PointEL4 = (19, 77-Elevator_Constants.kbaseHeight, "pointEPath", 9, 22)

    PointFL1 = (20, 26-Elevator_Constants.kbaseHeight, "pointFPath", 9, 22)
    PointFL2 = (21, 34-Elevator_Constants.kbaseHeight, "pointFPath", 9, 22)
    PointFL3 = (22, 50-Elevator_Constants.kbaseHeight, "pointFPath", 9, 22)
    PointFL4 = (23, 77-Elevator_Constants.kbaseHeight, "pointFPath", 9, 22)

    PointGL1 = (24, 26-Elevator_Constants.kbaseHeight, "pointGPath", 10, 21)
    PointGL2 = (25, 34-Elevator_Constants.kbaseHeight, "pointGPath", 10, 21)
    PointGL3 = (26, 50-Elevator_Constants.kbaseHeight, "pointGPath", 10, 21)
    PointGL4 = (27, 77-Elevator_Constants.kbaseHeight, "pointGPath", 10, 21)

    PointHL1 = (28, 26-Elevator_Constants.kbaseHeight, "pointHPath", 10, 21)
    PointHL2 = (29, 34-Elevator_Constants.kbaseHeight, "pointHPath", 10, 21)
    PointHL3 = (30, 50-Elevator_Constants.kbaseHeight, "pointHPath", 10, 21)
    PointHL4 = (31, 77-Elevator_Constants.kbaseHeight, "pointHPath", 10, 21)

    PointIL1 = (32, 26-Elevator_Constants.kbaseHeight, "pointIPath", 11, 20)
    PointIL2 = (33, 34-Elevator_Constants.kbaseHeight, "pointIPath", 11, 20)
    PointIL3 = (34, 50-Elevator_Constants.kbaseHeight, "pointIPath", 11, 20)
    PointIL4 = (35, 77-Elevator_Constants.kbaseHeight, "pointIPath", 11, 20)

    PointJL1 = (36, 26-Elevator_Constants.kbaseHeight, "pointJPath", 11, 20)
    PointJL2 = (37, 34-Elevator_Constants.kbaseHeight, "pointJPath", 11, 20)
    PointJL3 = (38, 50-Elevator_Constants.kbaseHeight, "pointJPath", 11, 20)
    PointJL4 = (39, 77-Elevator_Constants.kbaseHeight, "pointJPath", 11, 20)

    PointKL1 = (40, 26-Elevator_Constants.kbaseHeight, "pointKPath", 6, 19)
    PointKL2 = (41, 34-Elevator_Constants.kbaseHeight, "pointKPath", 6, 19)
    PointKL3 = (42, 50-Elevator_Constants.kbaseHeight, "pointKPath", 6, 19)
    PointKL4 = (43, 77-Elevator_Constants.kbaseHeight, "pointKPath", 6, 19)

    PointLL1 = (44, 26-Elevator_Constants.kbaseHeight, "pointLPath", 6, 19)
    PointLL2 = (45, 34-Elevator_Constants.kbaseHeight, "pointLPath", 6, 19)
    PointLL3 = (46, 50-Elevator_Constants.kbaseHeight, "pointLPath", 6, 19)
    PointLL4 = (47, 77-Elevator_Constants.kbaseHeight, "pointLPath", 6, 19)

    def __init__(self, value, elevatorHeight, pathName, redTag, blueTag):
        self.m_value = value
        self.m_elevatorHeight = elevatorHeight
        self.m_pathName = pathName
        self.m_redTagID = redTag
        self.m_blueTagID = blueTag