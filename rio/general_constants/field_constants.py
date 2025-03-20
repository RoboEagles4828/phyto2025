from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from lib.util.units import Units
from lib.util.convenientmath import inputModulus
from enum import Enum
import mmap
from subsystems.elevator.elevator_constants import Elevator_Constants
import math

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
 
    bumperWidth = Units.inchesToMeters(3)
    reefStandOff = Units.inchesToMeters(0.0)
    reefOffSet = Units.inchesToMeters(28 / 2.0) + bumperWidth + reefStandOff
    reefExtraOffSet = Units.inchesToMeters(4.0)

    fieldWidth = Units.inchesToMeters(26*12+5)
    reefCenter = Translation2d(Units.inchesToMeters(176.75), fieldWidth/2)
    reefToFaceDistance = reefCenter.X() - Units.inchesToMeters(144.0)
    branchSeperation = Units.inchesToMeters(12.0 + 15.0 / 16.0)
    centerOffset = Translation2d(reefToFaceDistance + reefOffSet, 0.0)
    leftOffset = Translation2d(reefToFaceDistance + reefOffSet, (-branchSeperation/2.0)-0.0889)
    rightOffset = Translation2d(reefToFaceDistance + reefOffSet, (branchSeperation/2.0)-0.0889)
    extraOffset  = Translation2d(reefExtraOffSet, 0.0)

    centerApproachOffset = centerOffset.__add__(extraOffset)
    leftApproachOffset = leftOffset.__add__(extraOffset)
    rightApproachOffset = rightOffset.__add__(extraOffset)

class ReefFace(Enum):
    AB = (-180)
    CD = (-120)
    EF = (-60)
    GH = (0)
    IJ = (60)
    KL = (120)

    def __init__(self, directionDegrees):
        self.directionFromCenter = Rotation2d.fromDegrees(directionDegrees)
        self.alignMiddle = Pose2d(FieldConstants.reefCenter.__add__(
            FieldConstants.centerOffset).rotateAround(
                FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                    Rotation2d.fromDegrees(180)))   
        self.alignLeft = Pose2d(FieldConstants.reefCenter.__add__(FieldConstants.leftOffset).rotateAround(
            FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                Rotation2d.fromDegrees(180)))
        self.alignRight = Pose2d(FieldConstants.reefCenter.__add__(FieldConstants.rightOffset).rotateAround(
            FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                Rotation2d.fromDegrees(180)))
        self.alignCenterApproach = Pose2d(FieldConstants.reefCenter.__add__(FieldConstants.centerApproachOffset).rotateAround(
            FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                Rotation2d.fromDegrees(180)))
        self.alignLeftApproach = Pose2d(FieldConstants.reefCenter.__add__(FieldConstants.leftApproachOffset).rotateAround(
            FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                Rotation2d.fromDegrees(180)))
        self.alignRightApproach = Pose2d(FieldConstants.reefCenter.__add__(FieldConstants.rightApproachOffset).rotateAround(
            FieldConstants.reefCenter, self.directionFromCenter), self.directionFromCenter.__add__(
                Rotation2d.fromDegrees(180)))