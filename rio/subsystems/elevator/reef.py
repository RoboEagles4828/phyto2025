from wpimath import units


class Reef:
    """
    Constants to describe the reef and its vertical relationship to the cannon for scoring.
    """

    # May need to adjust these four during competition field calibration time.
    L1_EDGE_HEIGHT_M = units.inchesToMeters(17.88)
    """ L1 edge height from field drawings (V1, updated Jan 10) +- 0.5 inch. """
    L2_MAX_HEIGHT_M = units.inchesToMeters(31.72)
    """ L2 max height from field drawings (V1, updated Jan 10) +- 0.5 inch. """
    L3_MAX_HEIGHT_M = units.inchesToMeters(47.59)
    """ L3 max height from field drawings (V1, updated Jan 10) +- 0.5 inch. """
    L4_MAX_HEIGHT_M = units.inchesToMeters(71.87)
    """ L4 max height from field drawings (V1, updated Jan 10) +- 0.5 inch. """

    # Test to find these in lab. Should not need adjusted at competition.
    TROUGH_CANNON_CLEARANCE_M = units.inchesToMeters(6.0)
    """ How high above the trough, the cannon (carriage bottom) must be for scoring. """
    L2_L3_CANNON_CLEARANCE_M = units.inchesToMeters(6.0)
    """ How high above angled pipes, the cannon (carriage bottom) must be for scoring. """
    L4_CANNON_CLEARANCE_M = units.inchesToMeters(6.0)
    """ How high above vertical pipe, the cannon (carriage bottom) must be for scoring. """

    L1_SCORE_HEIGHT_M = L1_EDGE_HEIGHT_M + TROUGH_CANNON_CLEARANCE_M
    """ Cannon (carriage bottom) height from floor for L1 scoring. """
    L2_SCORE_HEIGHT_M = L2_MAX_HEIGHT_M + L2_L3_CANNON_CLEARANCE_M
    """ Cannon (carriage bottom) height from floor for L2 scoring. """
    L3_SCORE_HEIGHT_M = L3_MAX_HEIGHT_M + L2_L3_CANNON_CLEARANCE_M
    """ Cannon (carriage bottom) height from floor for L3 scoring. """
    L4_SCORE_HEIGHT_M = L4_MAX_HEIGHT_M + L4_CANNON_CLEARANCE_M
    """ Cannon (carriage bottom) height from floor for L4 scoring. """