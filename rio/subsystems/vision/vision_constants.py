from wpimath.geometry import Pose3d, Pose2d, Rotation3d, Transform3d
from lib.util.keyorginization import OptionalValueKeys
import math

class Vision_Constants:
    kPhotonvisionCameraName = "camcam"
    kPhotonvisionCameraArray = ["frontLeft", "frontRight", "backLeft"]

    kPhotonvisionFrontLeftCameraKey = "cameras/frontLeftCamera"
    kPhotonvisionFrontRightCameraKey = "cameras/frontRightCamera"
    kPhotonvisionBackLeftCameraKey = "cameras/backLeftCamera"
    kPhotonvisionBackRightCameraKey = "cameras/backRightCamera"


    kSimRobotPoseArrayKey = "SimRobotPoseArray"
    kSimRobotVelocityArrayKey = "SimRobotVelocityArray"


    kPhotonvisionKeyArray = [
        kPhotonvisionFrontLeftCameraKey,
        kPhotonvisionFrontRightCameraKey,
        kPhotonvisionBackLeftCameraKey,
    ]
    kMetersPerInch = 0.0254
    kRadiansPerDegree = 0.0174533


    kRobotPoseArrayKeys = OptionalValueKeys("RobotOdometryPose")

    kRobotVisionPoseWeight = 0.00  # 5% vision data

    kDriveVelocityKeys = "robotVelocity"
    kDriveAccelLimit = 7
    kRobotUpdatePeriod = 1 / 50
    """seconds"""
    kLimelightUpdatePeriod = 1 / 10
    """seconds"""

    # Vision parameters
    kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
    kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
    kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
        "TargetFacingAngleRelativeToRobot"
    )
    kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
    kRobotVisionPoseArrayKeys = OptionalValueKeys("EstimatedRobotPose")
    kRobotToTagPoseKey = "vision/poses"
    kRobotToTagIdKey = "vision/ids"
    kRobotToTagAmbiguityKey = "vision/ambiguity"

    kTargetName = "Target"

    kApriltagPositionDict = {
    1: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 455.15),
        (kMetersPerInch * 317.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 235.73),
        (kMetersPerInch * -0.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}

    kPhotonvisionNoteCameraKey = "noteCamera"
    kNoteInViewKey = OptionalValueKeys("noteInView")

    kNoteCameraPitch = 30 * kRadiansPerDegree  # below horizontal
    kNoteCameraYaw = 20 * kRadiansPerDegree
    kRobotToNoteCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            0.330296, -0.333443, 0.570646, Rotation3d(0, kNoteCameraPitch, kNoteCameraYaw)
        ),
    )

    kCameraFOVHorizontal = 75  # degrees
    kCameraFOVVertical = 47.4  # degrees

    kSimulationVariation = 0.001  # meters, as a standard deviation


    kRobotToFrontLeftCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            10 * kMetersPerInch,
            10 * kMetersPerInch,
            8.25 * kMetersPerInch,
            Rotation3d(0.0, -4.125 * kRadiansPerDegree, 0.0)
            ),
        )
        
    kRobotToFrontRightCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            0.25048 * kMetersPerInch,
            -0.21 * kMetersPerInch,
            0.145 * kMetersPerInch,
            Rotation3d(0.0, -30 * kRadiansPerDegree, 0.0),
        ),
    )
    kRobotToBackLeftCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            -0.25048 * kMetersPerInch,
            0.21 * kMetersPerInch,
            0.145 * kMetersPerInch,
            Rotation3d(0.0, -30 * kRadiansPerDegree, 0.0).rotateBy(
                Rotation3d(0.0, 0.0, (0) * kRadiansPerDegree)
            ),
        ),
    )
    kRobotToBackRightCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            -0.25048 * kMetersPerInch,
            -0.21 * kMetersPerInch,
            0.145 * kMetersPerInch,
            Rotation3d(0.0, -30 * kRadiansPerDegree, 0.0).rotateBy(
                Rotation3d(0.0, 0.0, (0) * kRadiansPerDegree)
            ),
        ),
    )
    kCameraTransformsArray = [
        kRobotToFrontLeftCameraTransform,
        kRobotToFrontRightCameraTransform,
        kRobotToBackLeftCameraTransform,
        kRobotToBackRightCameraTransform,
    ]