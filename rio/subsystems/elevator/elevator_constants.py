class Elevator_Constants:

    Motor_One_ID = 21
    """Motor ID 21"""

    Motor_Two_ID = 22
    """Motor ID 22"""

    Gear_Ratio = 20.0
    """Gear Ratio is 20.0"""

    Ampere_Stator_Limit = 120
    """Stator Limit is 120 Amperes"""

    MM_CRUISE_VELOCITY_ROT_PER_SEC = 40
    """ 5 mechanism rotations per second cruise """
    
    MM_ACCELERATION_ROT_PER_SEC2 = 400
    """ Take approximately 0.5 seconds to reach max vel (10 r/sec^2 * 0.5 sec = 5 r/sec) """
    
    MM_JERK_ROT_PER_SEC3 = 800
    """Take approximately ___ seconds to reach max accel ()"""
    
    MM_KG_AMPS = 0.0
    """ Gravity feedforward in amps """

    MM_KS_AMPS = 0.0
    """ Feedforward to overcome static friction in amps """

    MM_KV_AMPS_PER_UNIT_OF_TARGET_VELOCITY = 0.0
    """ Closed-loop gain amps per target velocity unit: amps/(rot/sec) """

    MM_KA_AMPS_PER_UNIT_OF_TARGET_ACCEL = 0.0
    """ Closed-loop gain amps per target acceleration unit: amps/(rot/sec^2) """

    MM_KP_AMPS_PER_UNIT_OF_ERROR_IN_POSITION = 0.5
    """ Closed-loop gain amps per unit of error in position: amp/rot """

    MM_KI_AMPS_PER_UNIT_OF_INTEGRATED_ERROR_IN_POSITION = 0.0
    """ Closed-loop gain amps per unit of integrated error in position: amp/rot^2 """

    MM_KD_AMPS_PER_UNIT_OF_ERROR_IN_VELOCITY = 0.1
    """ Closed-loop gain amps per unit of error in velocity: amp/(rot/sec) """

    Percent_Power = 0.2
    """20% power going up and down manually"""

    Neg_Percent_Power = -0.2
    """20% power going up and down manually"""

    No_Percent_Power = 0
    """0% Power in motors"""

    L1_Rotations = 1.0
    """L1 Preset"""

    L2_Rotations = 2.0
    """L2 Preset"""

    L3_Rotations = 3.0
    """L3_Preset"""

    L4_Rotations = 4.0
    """L4_Preset"""

    At_Base = 0
