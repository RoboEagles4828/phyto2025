import math

from wpimath import units


class Elevator_Constants:

    Motor_One_ID = 21
    """Motor ID 21"""

    Motor_Two_ID = 22
    """Motor ID 22"""

    Ampere_Stator_Limit = 120
    """Stator Limit is 120 Amperes"""

    Gear_Ratio = 20.0
    """Gear Ratio is 20.0"""

    CARRIAGE_HEIGHT_AT_BOTTOM_M = units.inchesToMeters(8.0) # TODO measure
    """ Height of the carriage (bottom edge) at elevator bottom. """

    ELEVATOR_STAGE_COUNT = 3
    """ Number of elevator stages. """

    ELEVATOR_SPROCKET_DIAMETER_M = units.inchesToMeters(1.25) # TODO find
    """ Diameter in meters of the chain sprockets. """

    # TODO check this math. Coming out higher than we discussed. Maybe check at robot.
    MECH_M_PER_ROT = ELEVATOR_SPROCKET_DIAMETER_M * math.pi * ELEVATOR_STAGE_COUNT
    """ Vertical delta from one rotation of the mechanism shaft coming out of the gearbox. """

    MM_CRUISE_VELOCITY_M_PER_SEC = 0.5
    """ Max cruise velocity in meters per second. """

    MM_CRUISE_VELOCITY_ROT_PER_SEC = MM_CRUISE_VELOCITY_M_PER_SEC / MECH_M_PER_ROT
    """ Max mechanism rotations per second cruise """

    MM_ACCELERATION_ROT_PER_SEC2 = MM_CRUISE_VELOCITY_ROT_PER_SEC * 2.0
    """ Take approximately 0.5 seconds to reach max vel """

    MM_JERK_ROT_PER_SEC3 = MM_ACCELERATION_ROT_PER_SEC2 * 10.0
    """ Take approximately 0.1 seconds to reach max accel """

    MM_KG_AMPS = 0.0 # TODO measure current draw when holding elevator manually.
    """ Gravity feedforward in amps """

    MM_KS_AMPS = 0.0 # Only use a small amount if elevator seems "sticky"
    """ Feedforward to overcome static friction in amps """

    # TODO after KG set, use 1.0 as a first try. Tune until the constant
    # velocity segment of motion looks good.
    MM_KV_AMPS_PER_UNIT_OF_TARGET_VELOCITY = 0.0
    """ Closed-loop gain amps per target velocity unit: amps/(rot/sec) """

    # TODO after KV set, use 0.5 as first try. Tune until accel to the
    # constant velocity segment looks good.
    MM_KA_AMPS_PER_UNIT_OF_TARGET_ACCEL = 0.0
    """ Closed-loop gain amps per target acceleration unit: amps/(rot/sec^2) """

    # TODO May not need this, probably will not need KI or KD.
    # To try after KA set, start small and increase until overshoot.
    # Then backoff until no overshoot.
    MM_KP_AMPS_PER_UNIT_OF_ERROR_IN_POSITION = 0.0
    """ Closed-loop gain amps per unit of error in position: amp/rot """

    MM_KI_AMPS_PER_UNIT_OF_INTEGRATED_ERROR_IN_POSITION = 0.0
    """ Closed-loop gain amps per unit of integrated error in position: amp/rot^2 """

    MM_KD_AMPS_PER_UNIT_OF_ERROR_IN_VELOCITY = 0.0
    """ Closed-loop gain amps per unit of error in velocity: amp/(rot/sec) """

    MANUAL_UP_DUTY_CYCLE = 0.2
    """ Power going up manually"""

    MANUAL_DOWN_DUTY_CYCLE = -0.2
    """ Power going down manually"""

    MANUAL_NO_POWER = 0
    """ Turn off motors """
