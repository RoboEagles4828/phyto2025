from wpimath.geometry import Transform3d, Rotation3d
import math
from wpimath import units
class Elevator_Constants:

    # Motor IDs
    kLeftMotorID = 21
    kRightMotorID = 22

    kBottomLimitSwitchID = 9
    kTopLimitSwitchID = 0

    # PID Constants
    kGravity = 0.0
    kStatic = 0.0
    kVelocity = 0.1
    kAcceleration = 0.0
    kPorportional = 9.0
    kIntegral = 0
    kDerivative = 0.0

    # Motion Magic Configurations
    kCruiseVelocity = 1.5 # _kCruiseVelocityMPS / kMetersPerRotation
    kMagicAcceleration = kCruiseVelocity * 2.0
    kMagicJerk = kMagicAcceleration * 10.0


    # Current Limits
    kCurrentLimit = 120
    kCurrentLimitEnable = True

    # XTRA
    kGearRatio = 18.0


    # Manual control speeds
    kManualOut = 0.2

    kTolerance = 0.05