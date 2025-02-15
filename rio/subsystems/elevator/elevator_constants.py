from wpimath.geometry import Transform3d, Rotation3d
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration, CurrentLimitsConfigs
import math

class Elevator_Constants:

    # Motor IDs
    kMotor1ID = 21
    kMotor2ID = 22

    # Configurations
    kGravity = 0.0
    kStatic = 0.0
    kVelocity = 0.0
    kAcceleration = 0.0
    kPorportional = 0.5
    kIntegral = 0.0
    kDerivative = 0.1

    kGearRatio = 20.0

    # Current Limits
    kCurrentLimit = 120
    kCurrentLimitEnable = True

    # Motion Magic Configurations
    kCruiseVelocity = 40
    kMagicAcceleration = 80
    kMagicJerk = 800

    kMetersPerInch = 0.0254

    kElevatorHeightAboveGround = 8.526

    # Mechanism 2d information
    kThickness = 2.0
    kSetBack = 9.0
    kBellyHeight = 1.537
    kBaseHeight = 13 + kBellyHeight
    kMaxHeight = 80 + kBellyHeight
    kCannonHeight = 6.0


    kRotationsPerInch = 1.2119 #formula is rotations*gear ratio*diameter*pi
    kElevatorPositionKey = "elevator/position"
    kElevatorStateKey = "elevator/state"

    kElevatorPoseArrayKey = "elevator/pose"

    kRobotToElevatorTransform = Transform3d(
        9.75 * kMetersPerInch,
        -6.75 * kMetersPerInch,
        13.875 * kMetersPerInch,
        Rotation3d(0, 0, math.pi),
    )

    kMotorPulleyGearRatio = 60 / 18 * 4 / 1

    kPulleyGearPitchDiameter = 1.504 * kMetersPerInch
    """meters"""

    kBottomPositionBeltPosition = 0
    kAmpPositionBeltPosition = 19.125 * kMetersPerInch
    kTopPositionBeltPosition = 26.5 * kMetersPerInch
    """meters"""

    kBeltPullDownSpeed = 3
    """inches per second"""

    kPullDownBandLimit = 0.1
    """Revolutions"""

    kElevatorPositionKey = "ElevatorPosition"

    kElevatorManualChange = 0.01
    """.01 meters / (1/50) seconds = 0.5 m/s"""