from phoenix5 import SupplyCurrentLimitConfiguration


class ConstantsHopper:

    hopperMotorID = 13
    """Hopper motor CAN id"""

    # TODO: Run tests to find appropriate values for these
    current_limit = 30
    current_threshold = 50
    current_threshold_time = 3.0

    supply_config = SupplyCurrentLimitConfiguration(
        True, current_limit, current_threshold, current_threshold_time
    )

    intake_duty_cycle = 0.5
    """Intake action duty cycle."""
    agitation_duty_cycle = -0.5
    """Stuck coral agitation duty cycle."""
    stuck_coral_current_threshold = 15
    """Current threshold for detecting stuck coral. More than this for debounce time."""
    stuck_detection_debounce_sec = 0.5
    """How long (seconds) to debounce stuck detection."""
    unstuck_coral_current_threshold = 10
    """Current threshold for detecting coral now unstuck. Less than this for debounce time."""
    unstuck_detection_debounce_sec = 0.1
    """How long (seconds) to debounce unstuck detection."""
