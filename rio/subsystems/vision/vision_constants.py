from wpimath.geometry import Pose3d, Pose2d, Rotation3d, Transform3d
from lib.util.keyorginization import OptionalValueKeys
import math

class Vision_Constants:
    kPhotonvisionCameraName = "camcam"
    kPhotonvisionCameraArray = ["frontLeft", "frontRight", "backLeft", "backRight"]

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
        kPhotonvisionBackRightCameraKey,
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

    kApriltagPositionDict = {  # thanks 6328 for FieldConstants!
        1: Pose3d(
            (kMetersPerInch * 593.68),
            (kMetersPerInch * 9.68),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        2: Pose3d(
            (kMetersPerInch * 637.21),
            (kMetersPerInch * 34.79),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        3: Pose3d(
            (kMetersPerInch * 652.73),
            (kMetersPerInch * 196.17),
            (kMetersPerInch * 57.13),
            Rotation3d(0.0, 0.0, math.pi),
        ),
        4: Pose3d(
            (kMetersPerInch * 652.73),
            (kMetersPerInch * 218.42),
            (kMetersPerInch * 57.13),
            Rotation3d(0.0, 0.0, math.pi),
        ),
        5: Pose3d(
            (kMetersPerInch * 578.77),
            (kMetersPerInch * 323.00),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
        ),
        6: Pose3d(
            (kMetersPerInch * 72.5),
            (kMetersPerInch * 323.00),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
        ),
        7: Pose3d(
            (kMetersPerInch * -1.50),
            (kMetersPerInch * 218.42),
            (kMetersPerInch * 57.13),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
        ),
        8: Pose3d(
            (kMetersPerInch * -1.50),
            (kMetersPerInch * 196.17),
            (kMetersPerInch * 57.13),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
        ),
        9: Pose3d(
            (kMetersPerInch * 14.02),
            (kMetersPerInch * 34.79),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
        ),
        10: Pose3d(
            (kMetersPerInch * 57.54),
            (kMetersPerInch * 9.68),
            (kMetersPerInch * 53.38),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
        ),
        11: Pose3d(
            (kMetersPerInch * 468.69),
            (kMetersPerInch * 146.19),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 300),
        ),
        12: Pose3d(
            (kMetersPerInch * 468.69),
            (kMetersPerInch * 177.10),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
        ),
        13: Pose3d(
            (kMetersPerInch * 441.74),
            (kMetersPerInch * 161.62),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 180),
        ),
        14: Pose3d(
            (kMetersPerInch * 209.48),
            (kMetersPerInch * 161.62),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
        ),
        15: Pose3d(
            (kMetersPerInch * 182.73),
            (kMetersPerInch * 177.10),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 120),
        ),
        16: Pose3d(
            (kMetersPerInch * 182.73),
            (kMetersPerInch * 146.19),
            (kMetersPerInch * 52.00),
            Rotation3d(0.0, 0.0, kRadiansPerDegree * 240),
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

    kCameraFOVHorizontal = 75.9  # degrees
    kCameraFOVVertical = 47.4  # degrees

    kSimulationVariation = 0.001  # meters, as a standard deviation


    kRobotToFrontLeftCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            11.306 * kMetersPerInch,
            10.256 * kMetersPerInch,
            9.238 * kMetersPerInch,
            Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
                Rotation3d(0.0, 0.0, 30.0 * kRadiansPerDegree)
            ),
        ),
    )
    kRobotToFrontRightCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            11.306 * kMetersPerInch,
            -12.749 * kMetersPerInch,
            9.238 * kMetersPerInch,
            Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0),
        ),
    )
    kRobotToBackLeftCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            -11.306 * kMetersPerInch,
            10.256 * kMetersPerInch,
            9.238 * kMetersPerInch,
            Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
                Rotation3d(0.0, 0.0, (180 - 30.0) * kRadiansPerDegree)
            ),
        ),
    )
    kRobotToBackRightCameraTransform = Transform3d(
        Pose3d(),
        Pose3d(
            -11.306 * kMetersPerInch,
            -10.256 * kMetersPerInch,
            9.238 * kMetersPerInch,
            Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
                Rotation3d(0.0, 0.0, (180 + 30.0) * kRadiansPerDegree)
            ),
        ),
    )
    kCameraTransformsArray = [
        kRobotToFrontLeftCameraTransform,
        kRobotToFrontRightCameraTransform,
        kRobotToBackLeftCameraTransform,
        kRobotToBackRightCameraTransform,
    ]