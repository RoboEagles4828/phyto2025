from phoenix5 import SupplyCurrentLimitConfiguration
class ConstantsHopper:

    HOPPERMOTOR_ID = 35  # TODO find out what this really is
    BEAMBREAK_ID = 0 # TODO find out what this really is

    # TODO: Run tests to find appropriate values for these
    CURRENT_LIMIT = 30
    CURRENT_THRESHOLD= 50
    CURRENT_THRESHOLD_TIME= 3.0

    SUPPLY_CONFIG = SupplyCurrentLimitConfiguration(CURRENT_LIMIT, CURRENT_THRESHOLD, CURRENT_THRESHOLD_TIME)


