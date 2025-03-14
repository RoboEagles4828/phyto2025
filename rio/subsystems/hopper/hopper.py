from commands2 import Command, Subsystem
from phoenix5 import TalonSRX, TalonSRXControlMode
from subsystems.hopper.hopper_constants import ConstantsHopper
from wpilib import SmartDashboard
from wpimath.filter import Debouncer


class Hopper(Subsystem):
    def __init__(self):
        self.hopperMotor = TalonSRX(ConstantsHopper.hopperMotorID)
        self.hopperMotor.configSupplyCurrentLimit(ConstantsHopper.supply_config)
        self.hopperMotor.setInverted(False)

        self.stuckDebouncer = Debouncer(
            ConstantsHopper.stuck_detection_debounce_sec, Debouncer.DebounceType.kRising
        )
        self.unstuckDebouncer = Debouncer(
            ConstantsHopper.unstuck_detection_debounce_sec,
            Debouncer.DebounceType.kRising,
        )

    def setHopperSpeed(self, percentOutput):
        """Sets the speed of the Hopper Motor"""
        self.hopperMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)

    def stop(self) -> Command:
        """Stops the hopper motor"""
        return self.run(lambda: self.setHopperSpeed(0))

    def intake(self) -> Command:
        """
        Intakes until success or stall. If stalled, then agitate until freed.
        Then go back to intaking. Designed to run whileTrue.
        """
        return (
            self.run(lambda: self.setHopperSpeed(ConstantsHopper.intake_duty_cycle))
            .until(lambda: self.hopper_stall())
            .andThen(self.agitate().until(lambda: self.re_run_intake()))
            .andThen(lambda: self.setHopperSpeed(ConstantsHopper.intake_duty_cycle))
        )

    def agitate(self) -> Command:
        """Reverses motor incase a coral gets stuck"""
        return self.run(
            lambda: self.setHopperSpeed(ConstantsHopper.agitation_duty_cycle)
        )

    def hopper_stall(self) -> bool:
        """Returns true when hopper stall (stuck coral) detected."""
        return self.stuckDebouncer.calculate(
            abs(self.hopperMotor.getStatorCurrent())
            > ConstantsHopper.stuck_coral_current_threshold
        )

    def re_run_intake(self) -> bool:
        """Returns true when hopper no longer stalled (coral unstuck) detected."""
        return self.unstuckDebouncer.calculate(
            abs(self.hopperMotor.getStatorCurrent())
            < ConstantsHopper.unstuck_coral_current_threshold
        )

    def periodic(self):
        SmartDashboard.putNumber(
            "Hopper/Motor Stator Current", self.hopperMotor.getStatorCurrent()
        )
