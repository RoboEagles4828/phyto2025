from commands2 import Command, Subsystem
from phoenix5 import NeutralMode, TalonSRX, TalonSRXControlMode
from subsystems.cannon.constants_cannon import Constants_Cannon
from wpilib import SmartDashboard


class Cannon(Subsystem):
    def __init__(self) -> None:
        self.leftMotor = TalonSRX(Constants_Cannon.leftMotorID)
        self._configure_motor(self.leftMotor)
        self.rightMotor = TalonSRX(Constants_Cannon.rightMotorID)
        self._configure_motor(self.rightMotor)

        self.leftMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)
        self.rightMotor.configSupplyCurrentLimit(Constants_Cannon.supply_config)

        # check to see which one should be inverted
        self.leftMotor.setInverted(True)
        self.rightMotor.setInverted(False)

        self.loaded = True  # Start match with pre-loaded game piece

    def _configure_motor(self, motor: TalonSRX):
        motor.configFactoryDefault()
        motor.setNeutralMode(NeutralMode.Brake)

    def loadCoral(self) -> Command:
        return (
            self.runEnd(
                lambda: self._runStraight(Constants_Cannon.loadPercentOutput),
                lambda: self._stop(),
            )
            .until(self.stopLoading)
            .andThen(self.runOnce(self.setLoaded(True)))
        )

    def placeCoral(self) -> Command:
        """Currently designed for whileTrue, but could easily be changed to timed."""
        return self.runEnd(
            lambda: self._runStraight(Constants_Cannon.placePercentOutput),
            lambda: self._stop(),
        ).andThen(self.runOnce(self.setLoaded(False)))

    def placeCoralTimed(self) -> Command:
        """The same as placeCoral but with a timeout. Suitable for automodes."""
        return (
            self.runEnd(
                lambda: self._runStraight(Constants_Cannon.placePercentOutput),
                lambda: self._stop(),
            )
            .withTimeout(Constants_Cannon.placeCoralAutoTimeoutSec)
            .andThen(self.runOnce(self.setLoaded(False)))
        )

    def stop(self) -> Command:
        """
        Stops the motors
        """
        return self.run(lambda: self._stop())

    def stopLoading(self):
        return abs(self.rightMotor.getStatorCurrent()) > 10

    def backup_command(self) -> Command:
        return self.runEnd(
            lambda: self._runStraight(Constants_Cannon.backupPercentOutput),
            lambda: self._stop(),
        )

    def getLoaded(self):
        """
        Returns wether the robot thinks it has a coral in the cannon
        """
        return self.loaded

    def setLoaded(self, newLoaded):
        self.loaded = newLoaded

    def placeL1(self) -> Command:
        return self.runEnd(
            lambda: self._runSpec(0.7, 0.4),
            lambda: self._stop(),
        ).andThen(self.runOnce(self.setLoaded(False)))

    def placeL1Timed(self) -> Command:
        """Same as placeL1 but with a timeout. Suitable for use in automodes."""
        return (
            self.runEnd(
                lambda: self._runSpec(0.7, 0.4),
                lambda: self._stop(),
            )
            .withTimeout(Constants_Cannon.placeCoralAutoTimeoutSec)
            .andThen(self.runOnce(self.setLoaded(False)))
        )

    def _runStraight(self, leftPercentOutput) -> None:
        self._runSpec(leftPercentOutput, leftPercentOutput * 1.05)

    def _runSpec(self, leftPercentOutput, rightPercentOutput) -> None:
        self.leftMotor.set(TalonSRXControlMode.PercentOutput, leftPercentOutput)
        self.rightMotor.set(TalonSRXControlMode.PercentOutput, rightPercentOutput)

    def _stop(self) -> None:
        self.leftMotor.set(TalonSRXControlMode.PercentOutput, 0.0)
        self.rightMotor.set(TalonSRXControlMode.PercentOutput, 0.0)

    def periodic(self):
        SmartDashboard.putBoolean("Cannon/loaded", self.loaded)
