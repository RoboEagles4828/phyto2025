from commands2 import Command, Subsystem
from phoenix5 import TalonSRX, TalonSRXControlMode
from subsystems.hopper.constants_hopper import ConstantsHopper


class Hopper(Subsystem):
    def _init_(self):
        self.motor = TalonSRX(ConstantsHopper.motorID)

    def sethopperspeed(self, percentOutput):
        self.motor.set(TalonSRXControlMode.PercentOutput, percentOutput)

    def stop(self) -> Command:
        return self.run(lambda: self.sethopperspeed(0))

    def move(self) -> Command:
        """Used for normal Coral intaking agitator movement."""
        return self.run(lambda: self.sethopperspeed(ConstantsHopper.intakeDutyCycle))

    def agitate(self) -> Command:
        """Maybe used for Coral stuck between agitator and back panel."""
        return self.run(lambda: self.sethopperspeed(ConstantsHopper.agitationDutyCycle))
