from collections import deque
from math import hypot, sin, tan, atan

# import numpy as np

from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting import PhotonTrackedTarget, PhotonPipelineResult
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import SmartDashboard
from wpilib import RobotBase, Timer, Field2d
from wpimath.geometry import (
    Transform3d,
    Pose3d,
    Pose2d,
    Rotation2d,
    Translation3d,
    Transform2d,
    Rotation3d,
)

from lib.util import advantagescope_convert as advantagescopeconvert
from lib.util.convenientmath import pose3dFrom2d
from subsystems.vision.vision_constants import Vision_Constants as constants
from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain
from typing import Optional

class VisionSubsystem(Subsystem):

    def __init__(self, swerve: CommandSwerveDrivetrain):
        self.frontLeftCamera = PhotonCamera("frontLeft")
        self.lastEstimatedTimestamp = 0.0
        self.lastPose = Pose2d()
        self.swerve = swerve
        self.updateDashboard = True
        self.field = Field2d()
        kTagLayout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeAndyMark)

        self.photonEsimtator = PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.frontLeftCamera, constants.kRobotToFrontLeftCameraTransform)
        self.photonEsimtator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        SmartDashboard.putData("vision/Field", self.field)

    
    def updatePoseEstimation(self):
        result = self.frontLeftCamera.getLatestResult()
        optVisionEstimate = self.photonEsimtator.update(result)
        

        if optVisionEstimate==None:
            return False
        else:
            visionEstimate = optVisionEstimate

        latestTimestamp = visionEstimate.timestampSeconds
        newResult = abs(latestTimestamp-self.lastEstimatedTimestamp) > 0.00001

        if self.updateDashboard:
            SmartDashboard.putBoolean("visioin/ New Result", newResult)
        
        if not newResult:
            return False
        
        self.lastEstimatedTimestamp = latestTimestamp
        self.lastPose = visionEstimate.estimatedPose.toPose2d()
        self.field.setRobotPose(self.lastPose)

        if self.swerve.get_state().pose != None:
            self.swerve.add_vision_measurement(self.lastPose, latestTimestamp)
        return True
    
    def getLastPose(self):
        return self.lastPose
    
    def periodic(self):
        result = self.frontLeftCamera.getLatestResult()
        haveTarget = result.hasTargets()

        self.updatePoseEstimation()

        SmartDashboard.putString("Vision/Result", str(result))
        SmartDashboard.putBoolean("Vision/ Have Target", haveTarget)
        SmartDashboard.putBoolean("Vision/ Have Result", result != None)
