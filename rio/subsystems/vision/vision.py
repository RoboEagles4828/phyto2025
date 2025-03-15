from collections import deque
from math import hypot, sin, tan, atan

# import numpy as np

from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting import PhotonTrackedTarget, PhotonPipelineResult
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from phoenix6.utils import fpga_to_current_time
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import SmartDashboard
from wpilib import RobotBase, Timer, Field2d
from wpimath import units
import math
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

        self.photonEstimator = PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.frontLeftCamera, constants.kRobotToFrontLeftCameraTransform)
        self.photonEstimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        

    
    # def updatePoseEstimation(self):
    #     result = self.frontLeftCamera.getAllUnreadResults()
    #     optVisionEstimate = self.photonEsimtator.update(result)
        

    #     if optVisionEstimate==None:
    #         return False
    #     else:
    #         visionEstimate = optVisionEstimate

    #     latestTimestamp = visionEstimate.timestampSeconds
    #     newResult = abs(latestTimestamp-self.lastEstimatedTimestamp) > 0.00001

    #     if self.updateDashboard:
    #         pass
        
    #     if not newResult:
    #         return False
        
    #     self.lastEstimatedTimestamp = latestTimestamp
    #     self.lastPose = visionEstimate.estimatedPose.toPose2d()
    #     self.field.setRobotPose(self.lastPose)

    #     if self.swerve.get_state().pose != None:
    #         self.swerve.add_vision_measurement(self.lastPose, fpga_to_current_time(latestTimestamp))
    #     return True

    def updatePoseEstimation(self):
        listofResults = self.frontLeftCamera.getAllUnreadResults()
        newResult = not(listofResults)
        updated = False


        for result in listofResults:
            lastResult = result

            optRobotPose = self.photonEstimator.update(lastResult)

            if optRobotPose == None:
                print("ignoring")
                continue

            lastRobotPose = optRobotPose.estimatedPose.toPose2d()
            self.field.setRobotPose(lastRobotPose)

            if self.swerve.get_state().pose!= None:
                self.swerve.add_vision_measurement(lastRobotPose, fpga_to_current_time(optRobotPose.timestampSeconds))
                updated = True
        return updated

    
    def getLastPose(self):
        return self.lastPose
    
    def periodic(self):
        result = self.frontLeftCamera.getLatestResult()
        haveTarget = result.hasTargets()

        self.updatePoseEstimation()

        
        
        
