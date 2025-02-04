from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera, VisionLEDMode, Packet
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from subsystems.vision.photon_utils import PhotonUtils

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Pose2d, Transform2d, Transform3d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d

import wpimath.units as Units

from wpilib import SmartDashboard, DriverStation

from typing import Callable

import math


#self.camerea = self.frontLeftCamera
#self.camera2 = self.frontRightCamera
#added two more cameras

class Vision(Subsystem):
    instance = None

    def __init__(self):
        try:
            self.frontLeftCamera : PhotonCamera | None = PhotonCamera("FrontLeftModule")
            self.frontRightCamera : PhotonCamera | None = PhotonCamera("FrontRightModule")
            self.backLeftCamera : PhotonCamera | None = PhotonCamera("BackLeftModule")
            self.backRightCamera : PhotonCamera | None = PhotonCamera("BackRightModule")
        except:
            self.frontLeftCamera : PhotonCamera | None = None
            self.frontRightCamera : PhotonCamera | None = None
            print("========= NO PHOTON CAMERAS FOUND =========")

        self.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape) # changed the field to 2025

        self.speakerPositionBlue = Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Rotation2d())
        self.speakerPositionRed = Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180.0))
        
        self.processorPositionBlue = Pose2d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Rotation2d(90))
        self.processorPositionRed = Pose2d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Rotation2d(270))
        self.topCoralStationPositionBlue = Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Rotation2d(306))
        self.topCoralStationPositionRed = Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Rotation2d(234))
        self.bottomCoralStationPositionBlue = Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Rotation2d(54))
        self.bottomCoralStationPositionRed = Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Rotation2d(126))

        self.reefFaceOneBlue = Pose2d(Units.inchesToMeters(144.0), Units.inchesToMeters(158.50), Rotation2d(180))
        self.reefFaceOneRed = Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d(0))
        self.reefFaceTwoBlue = Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d(240))
        self.reefFaceTwoRed = Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d(60))
        self.reefFaceThreeBlue = Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d(300))
        self.reefFaceThreeRed = Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d(120))
        self.reefFaceFourBlue = Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d(0))
        self.reefFaceFourRed = Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d(180))
        self.reefFaceFiveBlue = Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d(60))
        self.reefFaceFiveRed = Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d(240))
        self.reefFaceSixBlue = Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d(120))
        self.reefFaceSixRed = Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d(300))


        self.distanceSpeakerFieldToCamera = 0.0

        # Right = 6.5 in
        # Up = 11.5 in
        # Froward = (frame / 2.0) - 1.25 in

        # self.robotToCamera = Transform3d(
        #     Translation3d(-Units.inchesToMeters((31.125 / 2.0) - 1.25), -Units.inchesToMeters(6.5), Units.inchesToMeters(11.5)),
        #     Rotation3d.fromDegrees(0.0, -45.0, 180.0)
        # )
        self.robotToFrontLeftCamera = Transform3d(
            Translation3d(-Units.inchesToMeters(9.8614), Units.inchesToMeters(8.267717), Units.inchesToMeters(5.70866)),
            Rotation3d.fromDegrees(0, -30.0, 0)
        )
        self.robotToFrontRightCamera = Transform3d(
            Translation3d(Units.inchesToMeters(9.8614), Units.inchesToMeters(8.267717), Units.inchesToMeters(5.70866)),
            Rotation3d.fromDegrees(0, -30.0, 0)
        )
        self.robotToBackLeftCamera = Transform3d(
            Translation3d(-Units.inchesToMeters(9.8614), -Units.inchesToMeters(8.267717), Units.inchesToMeters(5.70866)), 
            Rotation3d.fromDegrees(0, -30.0, 135)
        )
        self.robotToBackRightCamera = Transform3d(
            Translation3d(Units.inchesToMeters(9.8614), -Units.inchesToMeters(8.267717), Units.inchesToMeters(5.70866)),
            Rotation3d.fromDegrees(0, -30.0, -135)
        )
        self.fieldToCamera = Transform3d()

        if self.frontLeftCamera is not None and self.frontRightCamera is not None and self.backLeftCamera and self.backRightCamera:
            try:
                self.photonPoseEstimatorFrontLeft = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.frontLeftCamera,
                    self.robotToFrontLeftCamera 
                )
                self.photonPoseEstimatorFrontLeft.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

                self.photonPoseEstimatorFrontRight = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.frontRightCamera,
                    self.robotToFrontRightCamera 
                )
                self.photonPoseEstimatorFrontRight.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

                self.photonPoseEstimatorBackLeft = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.backLeftCamera,
                    self.robotToBackLeftCamera 
                )
                self.photonPoseEstimatorBackLeft.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

                self.photonPoseEstimatorBackRight = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.backRightCamera,
                    self.robotToBackRightCamera 
                )
                self.photonPoseEstimatorBackRight.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY
            except:
                self.photonPoseEstimatorFrontLeft = None
                self.photonPoseEstimatorFrontRight = None
                self.photonPoseEstimatorBackLeft = None
                self.photonPoseEstimatorBackRight = None
                print("===== PHOTON PROBLEM (POSE ESTIMATORS) =======")

        self.CAMERA_HEIGHT_METERS = Units.inchesToMeters(11.5)
        self.SPEAKER_HEIGHT_METERS = 1.45 # meters

        self.CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-30.0)

        self.instance : Vision = None

    @classmethod
    def getInstance(cls):
        if cls.instance == None:
            cls.instance = Vision()
        return cls.instance

    def getEstimatedGlobalPoseFrontLeft(self):
        if self.frontLeftCamera is None or self.photonPoseEstimatorFrontLeft is None:
            return None
        return self.photonPoseEstimatorFrontLeft.update()
    
    def getEstimatedGlobalPoseFrontRight(self):
        if self.frontRightCamera is None or self.photonPoseEstimatorFrontRight is None:
            return None
        return self.photonPoseEstimatorFrontRight.update()
    
    def getEstimatedGlobalPoseBackLeft(self):
        if self.backLeftCamera is None or self.photonPoseEstimatorBackLeft is None:
            return None
        return self.photonPoseEstimatorBackLeft.update()
    
    def getEstimatedGlobalPoseBackRight(self):
        if self.backRightCamera is None or self.photonPoseEstimatorBackRight is None:
            return None
        return self.photonPoseEstimatorBackRight.update()
    
    
    def getDistanceToReefOneFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceOnePos = self.reefFaceOneBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceOnePos = self.reefFaceOneRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceOnePos = self.reefFaceOneBlue

        distanceToReefOneFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceOnePos)
        )
        return distanceToReefOneFieldToCamera - (36.37 / 12.0)
    
    def getDistanceToReefTwoFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceTwoPos = self.reefFaceTwoBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceTwoPos = self.reefFaceTwoRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceTwoPos = self.reefFaceTwoBlue

        distanceToReefTwoFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceTwoPos)
        )
        return distanceToReefTwoFieldToCamera - (36.37 / 12.0)
    
    def getDistanceToReefThreeFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceThreePos = self.reefFaceThreeBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceThreePos = self.reefFaceThreeRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceThreePos = self.reefFaceThreeBlue

        distanceToReefThreeFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceThreePos)
        )
        return distanceToReefThreeFieldToCamera - (36.37 / 12.0)
    
    def getDistanceToReefFourFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceFourPos = self.reefFaceFourBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceFourPos = self.reefFaceFourRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceFourPos = self.reefFaceFourBlue

        distanceToReefFourFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceFourPos)
        )
        return distanceToReefFourFieldToCamera - (36.37 / 12.0)
    
    def getDistanceToReefFiveFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceFivePos = self.reefFaceFiveBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceFivePos = self.reefFaceFiveRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceFivePos = self.reefFaceFiveBlue

        distanceToReefFiveFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceFivePos)
        )
        return distanceToReefFiveFieldToCamera - (36.37 / 12.0)
    
    def getDistanceToReefSixFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        # speakerPos = self.speakerPositionBlue
        reefFaceSixPos = self.reefFaceSixBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # speakerPos = self.speakerPositionRed
            reefFaceSixPos = self.reefFaceSixRed
        else:
            # speakerPos = self.speakerPositionBlue
            reefFaceSixPos = self.reefFaceSixBlue

        distanceToReefSixFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, reefFaceSixPos)
        )
        return distanceToReefSixFieldToCamera - (36.37 / 12.0)
    

    
    def getDistanceVectorToSpeaker(self, pose: Pose2d):
        speakerPos = self.speakerPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            speakerPos = self.speakerPositionRed
        else:
            speakerPos = self.speakerPositionBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, speakerPos)
        return distanceVector
    
    def getDistanceVectorToReefOne(self, pose:Pose2d):
        reefFaceOnePos = self.reefFaceOneBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceOnePos = self.reefFaceOneRed
        else:
            reefFaceOnePos = self.reefFaceOneBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceOnePos)
        return distanceVector
    
    def getDistanceVectorToReefTwo(self, pose:Pose2d):
        reefFaceTwoPos = self.reefFaceTwoBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceTwoPos = self.reefFaceTwoRed
        else:
            reefFaceTwoPos = self.reefFaceTwoBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceTwoPos)
        return distanceVector
    
    def getDistanceVectorToReefThree(self, pose:Pose2d):
        reefFaceThreePos = self.reefFaceThreeBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceThreePos = self.reefFaceThreeRed
        else:
            reefFaceThreePos = self.reefFaceThreeBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceThreePos)
        return distanceVector
    
    def getDistanceVectorToReefFour(self, pose:Pose2d):
        reefFaceFourPos = self.reefFaceFourBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceFourPos = self.reefFaceFourRed
        else:
            reefFaceFourPos = self.reefFaceFourBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceFourPos)
        return distanceVector
    
    def getDistanceVectorToReefFive(self, pose:Pose2d):
        reefFaceFivePos = self.reefFaceFiveBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceFivePos = self.reefFaceFiveRed
        else:
            reefFaceFivePos = self.reefFaceFiveBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceFivePos)
        return distanceVector
    
    def getDistanceVectorToReefSix(self, pose:Pose2d):
        reefFaceSixPos = self.reefFaceSixBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFaceSixPos = self.reefFaceSixRed
        else:
            reefFaceSixPos = self.reefFaceSixBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, reefFaceSixPos)
        return distanceVector
    

    def getAngleToReefOneToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefOnePos = self.reefFaceOneBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefOnePos = self.reefFaceOneRed
        else:
            reefOnePos = self.reefFaceOneBlue

        dx = reefOnePos.X() - pose.X()
        dy = reefOnePos.Y() - pose.Y()

        angleToReefOneToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefOneToCamera
    
    def getAngleToReefTwoToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefTwoPos = self.reefFaceTwoBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefTwoPos = self.reefFaceTwoRed
        else:
            reefTwoPos = self.reefFaceTwoBlue

        dx = reefTwoPos.X() - pose.X()
        dy = reefTwoPos.Y() - pose.Y()

        angleToReefTwoToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefTwoToCamera
    
    def getAngleToReefThreeToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefThreePos = self.reefFaceThreeBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefThreePos = self.reefFaceThreeRed
        else:
            reefThreePos = self.reefFaceThreeBlue

        dx = reefThreePos.X() - pose.X()
        dy = reefThreePos.Y() - pose.Y()

        angleToReefThreeToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefThreeToCamera
    
    def getAngleToReefFourToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefFourPos = self.reefFaceFourBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFourPos = self.reefFaceFourRed
        else:
            reefFourPos = self.reefFaceFourBlue

        dx = reefFourPos.X() - pose.X()
        dy = reefFourPos.Y() - pose.Y()

        angleToReefFourToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefFourToCamera
    
    def getAngleToReefFiveToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefFivePos = self.reefFaceFiveBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefFivePos = self.reefFaceFiveRed
        else:
            reefFivePos = self.reefFaceFiveBlue

        dx = reefFivePos.X() - pose.X()
        dy = reefFivePos.Y() - pose.Y()

        angleToReefFiveToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefFiveToCamera
    
    def getAngleToReefSixToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        reefSixPos = self.reefFaceSixBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            reefSixPos = self.reefFaceSixRed
        else:
            reefSixPos = self.reefFaceSixBlue

        dx = reefSixPos.X() - pose.X()
        dy = reefSixPos.Y() - pose.Y()

        angleToReefSixToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToReefSixToCamera
    
    def getAngleToTopCoralStationToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        topCoralStationPos = self.topCoralStationPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            topCoralStationPos = self.topCoralStationPositionRed
        else:
            topCoralStationPos = self.topCoralStationPositionBlue

        dx = topCoralStationPos.X() - pose.X()
        dy = topCoralStationPos.Y() - pose.Y()

        angleToTopCoralStationToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToTopCoralStationToCamera
    
    def getAngleToBottomCoralStationToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        bottomCoralStationPos = self.bottomCoralStationPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            bottomCoralStationPos = self.bottomCoralStationPositionRed
        else:
            bottomCoralStationPos = self.bottomCoralStationPositionBlue

        dx = bottomCoralStationPos.X() - pose.X()
        dy = bottomCoralStationPos.Y() - pose.Y()

        angleToBottomCoralStationToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToBottomCoralStationToCamera
    
    def getAngleToProcessorToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        processorPos = self.processorPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            processorPos = self.processorPositionRed
        else:
            processorPos = self.processorPositionBlue

        dx = processorPos.X() - pose.X()
        dy = processorPos.Y() - pose.Y()

        angleToProcessorToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToProcessorToCamera

    
    def getFrontLeftCamera(self):
        return self.frontLeftCamera
    
    def getFrontRightCamera(self):
        return self.frontRightCamera
    
    def getBackLeftCamera(self):
        return self.backLeftCamera
    
    def getBackRightCamera(self):
        return self.backRightCamera
    
    def hasTargetBooleanSupplier(self):
        return lambda: self.frontLeftCamera.getLatestResult().hasTargets() or self.frontRightCamera.getLatestResult().hasTargets() or self.backLeftCamera.getLatestResult().hasTargets() or self.backRightCamera.getLatestResult().hasTargets()
    
    def takeSnapshot(self):
        self.frontLeftCamera.takeInputSnapshot()
        self.frontRightCamera.takeInputSnapshot()
        self.backLeftCamera.takeInputSnapshot()
        self.backRightCamera.takeInputSnapshot()

    def setPipeline(self, pipelineIdx):
        self.frontLeftCamera.setPipelineIndex(pipelineIdx)
        self.frontRightCamera.setPipelineIndex(pipelineIdx)
        self.backLeftCamera.setPipelineIndex(pipelineIdx)
        self.backRightCamera.setPipelineIndex(pipelineIdx)

    def setTagMode(self):
        self.setPipeline(0)

    def getBestTarget(self, result : PhotonPipelineResult):
        targets = result.getTargets()

        # sort targets by area largest to smallest
        targets.sort(key=lambda target: target.area, reverse=True)

        if len(targets) <= 0:
            return None

        return targets[0]
    
    def isTargetSeen(self, tagID) -> bool:
        if self.frontLeftCamera is None or self.frontRightCamera is None or self.backLeftCamera is None or self.backRightCamera is None:
            return False
        resultFrontLeft = self.frontLeftCamera.getLatestResult()
        resultFrontRight = self.frontRightCamera.getLatestResult()
        resultBackLeft = self.backLeftCamera.getLatestResult()
        resultBackRight = self.backRightCamera.getLatestResult()
        if resultFrontLeft.hasTargets() == False and resultFrontRight.hasTargets() == False and resultBackLeft.hasTargets() == False and resultBackRight.hasTargets() == False:
            return False
        best_target = self.getBestTarget(resultFrontLeft)
        second_best_target = self.getBestTarget(resultFrontRight)
        third_best_target = self.getBestTarget(resultBackLeft)
        fourth_best_target = self.getBestTarget(resultBackRight)
        return best_target.getFiducialId() == tagID or second_best_target.getFiducialId() == tagID or third_best_target.getFiducialId() == tagID or fourth_best_target.getFiducialId() == tagID
    
    def isTargetSeenLambda(self, tagIDSupplier: Callable[[], int]) -> bool:
        if self.frontLeftCamera is None or self.frontRightCamera is None or self.backLeftCamera is None or self.backRightCamera is None:
            return False
        resultFrontLeft = self.frontLeftCamera.getLatestResult()
        resultFrontRight = self.frontRightCamera.getLatestResult()
        resultBackLeft = self.backLeftCamera.getLatestResult()
        resultBackRight = self.backRightCamera.getLatestResult()
        if resultFrontLeft.hasTargets() == False and resultFrontRight.hasTargets() == False and resultBackLeft.hasTargets() == False and resultBackRight.hasTargets() == False:
            return False
        best_target = self.getBestTarget(resultFrontLeft)
        second_best_target = self.getBestTarget(resultFrontRight)
        third_best_target = self.getBestTarget(resultBackLeft)
        fourth_best_target = self.getBestTarget(resultBackRight)
        return best_target.getFiducialId() == tagIDSupplier() or second_best_target.getFiducialId() == tagIDSupplier() or third_best_target.getFiducialId() == tagIDSupplier() or fourth_best_target.getFiducialId() == tagIDSupplier()
    
    def getSortedTargetsList(self, result: PhotonPipelineResult):
        targets = result.getTargets()
        targets.sort(key=lambda target: target.area, reverse=True)
        return targets

    def getAngleToTag(self, tagIDSupplier: Callable[[], int]):
        if self.frontLeftCamera is None:
            return 0.0
        if self.frontRightCamera is None:
            return 0.0
        if self.backLeftCamera is None:
            return 0.0
        if self.backRightCamera is None:
            return 0.0
        
        if self.isTargetSeenLambda(tagIDSupplier):
            if (tagIDSupplier() == 18 or tagIDSupplier() == 19 or 
                tagIDSupplier() == 20 or tagIDSupplier() == 21 or
                tagIDSupplier() == 22 or tagIDSupplier() == 17 or
                tagIDSupplier() == 11 or tagIDSupplier() == 6 or
                tagIDSupplier() == 7 or tagIDSupplier() == 8 or
                tagIDSupplier() == 9 or tagIDSupplier() == 10):
                # if the tag is a reef face we want to use the front two cameras
                resultFrontLeft = self.frontLeftCamera.getLatestResult()
                resultFrontRight = self.frontRightCamera.getLatestResult()
                best_target_front_left = self.getBestTarget(resultFrontLeft)
                best_target_front_right = self.getBestTarget(resultFrontRight)
                if best_target_front_left is not None and best_target_front_right is not None:
                    return (best_target_front_left.getYaw() + best_target_front_right.getYaw()) / 2.0
                elif best_target_front_left is not None:
                    return best_target_front_left.getYaw()
                elif best_target_front_right is not None:
                    return best_target_front_right.getYaw()
                else:
                    return 0.0
            if (tagIDSupplier() == 1 or tagIDSupplier() == 2 or
                tagIDSupplier()== 13 or tagIDSupplier() == 12 or
                tagIDSupplier() == 13 or tagIDSupplier() == 16):
                # if the tag is a coral station or processor we want to use the back two cameras
                resultBackLeft = self.backLeftCamera.getLatestResult()
                resultBackRight = self.backRightCamera.getLatestResult()
                best_target_back_left = self.getBestTarget(resultBackLeft)
                best_target_back_right = self.getBestTarget(resultBackRight)
                if best_target_back_left is not None and best_target_back_right is not None:
                    return (best_target_back_left.getYaw() + best_target_back_right.getYaw()) / 2.0
                elif best_target_back_left is not None:
                    return best_target_back_left.getYaw()
                elif best_target_back_right is not None:
                    return best_target_back_right.getYaw()
                else:
                    return 0.0
        else:
            return 0.0

    # def periodic(self):
    #     # if self.camera is not None:
        #     result = self.camera.getLatestResult()

        #     if result.multiTagResult.estimatedPose.isPresent:
        #         self.fieldToCamera = result.multiTagResult.estimatedPose.best

        #     hasTargets = result.hasTargets()

        #     if hasTargets:        
        #         # get the best tag based on largest areaprint(f"================ {Units.inchesToMeters(self.s_Vision.getDistanceToSpeakerFieldToCameraInches(Transform3d(0.0, 0.0, 0.0, Rotation3d())))}")
        #         bestTarget = self.getBestTarget(result)
        #         if bestTarget is not None:
        #             SmartDashboard.putNumber("tag ID", bestTarget.getFiducialId())
        #             SmartDashboard.putNumber("pose ambiguity", bestTarget.getPoseAmbiguity())
        #             SmartDashboard.putNumber("tag transform X", bestTarget.getBestCameraToTarget().X())
        #             SmartDashboard.putNumber("tag transform Y", bestTarget.getBestCameraToTarget().Y())
        #             SmartDashboard.putNumber("tag transform Z", bestTarget.getBestCameraToTarget().Z())
        #             SmartDashboard.putNumber("tag transform angle", bestTarget.getBestCameraToTarget().rotation().angle_degrees)
        #             SmartDashboard.putNumber("tag yaw", bestTarget.getYaw())()
                # SmartDashboard.putNumber("Vision Pose X", self.getEstimatedGlobalPose().estimatedPose.X())
                # SmartDashboard.putNumber("Vision Pose Y", self.getEstimatedGlobalPose().estimatedPose.Y())
                # SmartDashboard.putNumber("Vision Pose Angle", self.getEstimatedGlobalPose().estimatedPose.rotation().angle_degrees)








