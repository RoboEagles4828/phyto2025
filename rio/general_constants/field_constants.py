from wpimath.geometry import Pose2d, Rotation2d
from lib.util.units import Units

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