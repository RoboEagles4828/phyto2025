from wpimath.geometry import Transform3d, Rotation3d
import math
from wpimath import units
class Elevator_Constants:

    # Motor IDs
    kMotor1ID = 21
    kMotor2ID = 22

    # Configurations
    kGravity = 0.0
    kStatic = 0.0
    kVelocity = 0.0
    kAcceleration = 0.0
    kPorportional = 0.0
    kIntegral = 0.0
    kDerivative = 0.0

    kGearRatio = 20.0

    MECH_M_PER_ROT = .4

    # Current Limits
    kCurrentLimit = 120
    kCurrentLimitEnable = True

    # Motion Magic Configurations
    _kElevatorStageCount = 3
    _kElevatorSprocketDiameter = units.inchesToMeters(1.25) # TODO find value

    # TODO check this math. Coming out higher than we discussed. Maybe check at robot.
    kMetersPerRotation = _kElevatorSprocketDiameter * math.pi * _kElevatorStageCount
    """ Vertical delta from one rotation of the mechanism shaft coming out of the gearbox. """

    _kCruiseVelocityMPS = 0.5

    kCruiseVelocity = _kCruiseVelocityMPS / kMetersPerRotation
    kMagicAcceleration = kCruiseVelocity * 2.0
    kMagicJerk = kMagicAcceleration * 10.0

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

    kManualUpDutyCycle = 0.2
    """ Power going up manually"""

    kManualDownDutyCycle = -0.2
    """ Power going down manually"""

    kManualNoPower = 0
    """ Turn off motors """

    CARRIAGE_HEIGHT_AT_BOTTOM_M = units.inchesToMeters(8.0) # TODO measure
    """ Height of the carriage (bottom edge) at elevator bottom. """
