from phoenix5 import SupplyCurrentLimitConfiguration
class ConstantsHopper:

    hopperMotorID = 13  # TODO find out what this really is
    beamBreakID = 0 # TODO find out what this really is

    # TODO: Run tests to find appropriate values for these
    current_limit = 30
    current_threshold= 50
    current_threshold_time= 3.0

    supply_config = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)

    #TODO: these values have to be checked
    intake_duty_cycle = 0.5
    agitation_duty_cycle = -0.5