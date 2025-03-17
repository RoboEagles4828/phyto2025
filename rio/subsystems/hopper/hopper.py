from commands2 import Command, Subsystem
from phoenix5 import TalonSRX, TalonSRXControlMode, SupplyCurrentLimitConfiguration
from subsystems.hopper.hopper_constants import ConstantsHopper
from wpilib import DigitalInput
from wpilib import SmartDashboard
from wpimath.filter import Debouncer


class Hopper(Subsystem):
    def __init__(self):
        self.hopperMotor = TalonSRX(ConstantsHopper.hopperMotorID)
        self.stallDebouncer = Debouncer(0.5, Debouncer.DebounceType.kBoth)
        # self.beamBreak = DigitalInput(ConstantsHopper.beamBreakID)

        # self.hopperMotor.configSupplyCurrentLimit(ConstantsHopper.supply_config)
        self.hopperMotor.setInverted(False)  #TODO: Check to see if this is correct

        self.coralInHopper = False

    def setHopperSpeed(self, percentOutput):
        """ Sets the speed of the Hopper Motor"""
        self.hopperMotor.set(TalonSRXControlMode.PercentOutput, percentOutput)

    def stop(self) -> Command:
        """Stops the hopper motor"""
        return self.run(lambda: self.setHopperSpeed(0))

    def intake(self) -> Command:
        """Runs the hopper motor at max speed"""
        return self.run(lambda: self.setHopperSpeed(ConstantsHopper.intake_duty_cycle))

    # def hasCoral(self) -> bool:
    #     """Returns whether the hopper has coral"""
    #     return not(self.beamBreak.get())  # self.breakBeam.get() returns True if the beam is not broken, so we negate it to return False if the beam is not broken

    def agitate(self) -> Command:
        """Reverses motor incase a coral gets stuck"""
        return self.run(lambda: self.setHopperSpeed(ConstantsHopper.agitation_duty_cycle))
    

    def hopper_stall(self):
        return self.stallDebouncer.calculate(self.hopperMotor.getStatorCurrent() > 15)
    
    def re_run_intake(self):
        return self.stallDebouncer.calculate(self.hopperMotor.getStatorCurrent() < 10)
    # def periodic(self):
    #     

    def periodic(self):
        SmartDashboard.putNumber("Hopper/ Supply Current", self.hopperMotor.getStatorCurrent())
        SmartDashboard.putNumber("Hopper / Stator Current", self.hopperMotor.getSupplyCurrent())
        
        