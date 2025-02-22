import math

from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6.hardware import TalonFX
from phoenix6.signal_logger import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from commands2 import Command
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicTorqueCurrentFOC, DutyCycleOut, VoltageOut
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, TalonFXConfigurator
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from lib.util.units import Units
from phoenix6.controls import StrictFollower, MotionMagicVoltage
from phoenix6 import SignalLogger, ampere, StatusSignal
from wpilib.shuffleboard import Shuffleboard

from subsystems.elevator.floor import Floor
from subsystems.elevator.elevator_constants import Elevator_Constants


class Elevator(Subsystem):

    motor_one: TalonFX
    motor_two: TalonFX

    def __init__(self):

        # Creating Motors and Configurators
        self.motor_one = TalonFX(Elevator_Constants.kMotor1ID)  # CAN ID 1
        self.motor_two = TalonFX(Elevator_Constants.kMotor2ID)  # CAN ID 2
        cfg = TalonFXConfiguration()
        cfg2 = self.motor_one.configurator

        # Gear Ratio and PID values
        # cfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        cfg.feedback.sensor_to_mechanism_ratio = Elevator_Constants.kGearRatio
        cfg.slot0.k_g = Elevator_Constants.kGravity
        cfg.slot0.k_s = Elevator_Constants.kStatic
        cfg.slot0.k_v = Elevator_Constants.kVelocity
        cfg.slot0.k_a = Elevator_Constants.kAcceleration
        cfg.slot0.k_p = Elevator_Constants.kPorportional # An error of 1 rotation results in 2.4 V output
        cfg.slot0.k_i = Elevator_Constants.kIntegral
        cfg.slot0.k_d = Elevator_Constants.kDerivative

        self.motor_one.setNeutralMode(NeutralModeValue.BRAKE)

        # Applying Limit Configurations
        limit_configs = CurrentLimitsConfigs()
        limit_configs.stator_current_limit = Elevator_Constants.kCurrentLimit  # Note that this is in AMPERES
        limit_configs.stator_current_limit_enable = True

        # Motion Magic Configurations
        motion_magic_configs = cfg.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = (
            Elevator_Constants.kCruiseVelocity
        )
        motion_magic_configs.motion_magic_acceleration = (
            Elevator_Constants.kMagicAcceleration
        )
        motion_magic_configs.motion_magic_jerk = Elevator_Constants.kMagicJerk

        # Applying all of Settings (and Checking if Error Occurs.)
        self.motor_two.set_control(StrictFollower(Elevator_Constants.kMotor1ID))
        cfg2.apply(limit_configs)
        cfg2.apply(cfg)

        # Misc
        self.request = MotionMagicTorqueCurrentFOC(0).with_slot(0)

        self.dutyCycle = DutyCycleOut(0.0)
        self.voltageOut = VoltageOut(0.0)

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V (TODO may need more)
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdElevator_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.motor_one.set_control(
                    self.voltageOut.with_output(output),
                ),
                lambda log: None,
                self,
            ),
        )

        # Create Mechanism Canvas for SmartDashboard
        canvasWidth = 21.0
        canvasHeight = Units.inchesToMeters(Elevator_Constants.kMaxHeight)
        canvas = Mechanism2d(canvasWidth, canvasHeight, Color8Bit(Color.kLightGray))
        origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0.0)
        offset = origin.appendLigament(
            "elevator-offset",
            canvasWidth / 2.0 - Units.inchesToMeters(Elevator_Constants.kSetBack),
            0.0,
            1.0,
            Color8Bit(),
        )
        mechanism = offset.appendLigament(
            "elevator",
            Units.inchesToMeters(Elevator_Constants.kBaseHeight),
            90.0,
            Units.inchesToMeters(Elevator_Constants.kThickness),
            Color8Bit(Color.kBrown),
        )

        SmartDashboard.putData("Elevator/mechanism", canvas)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction)

    def move_to_floor(self, floor: Floor) -> Command:
        """
        Returns a new command to sets the target rotations to the given Floor.
        Designed for button click or as part of a command group.
        """
        return self.runOnce(
            lambda: self.motor_one.set_control(
                self.request.with_position(floor.rotations)
            )
        )

    def move_up_gradually(self) -> Command:
        """
        Returns a new command to manually move up.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.motor_one.set_control(
                self.dutyCycle.with_output(Elevator_Constants.kManualUpDutyCycle)
            )
        )

    def move_down_gradually(self) -> Command:
        """
        Returns a new command to manually move down.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.motor_one.set_control(
                self.dutyCycle.with_output(Elevator_Constants.kManualDownDutyCycle)
            )
        )

    def stop(self) -> Command:
        """
        Returns a new command to remove power from motors. Elevator should drift downward.
        Designed for a button click or part of a command group. Also good for the release
        of the button used with manual motion.
        """
        return self.runOnce(
            lambda: self.motor_one.set_control(
                DutyCycleOut(Elevator_Constants.kManualNoPower)
            )
        )

    def periodic(self) -> None:
        """
        Overridden to update dashboard.
        """
        current: StatusSignal[ampere] = self.motor_one.get_torque_current()
        SmartDashboard.putNumber("Elevator/Elevator left amps", current.value)
        current = self.motor_two.get_torque_current()
        SmartDashboard.putNumber("Elevator/Elevator right amps", current.value)
        SmartDashboard.updateValues()
    # Functions below this point may be archived or deleted later

    def setPosition(self, position: float):
        self.desired_position = position
        self.motor_one.set_control(MotionMagicVoltage, position)

    def setHeight(self, height: float):
        position = (
            height - Elevator_Constants.kBaseHeight
        ) * Elevator_Constants.kRotationsPerInch
        self.setPosition(position)

    def closeEnough(self, position: float):
        return self.desired_position >= 0.0 and math.fabs(
            self.desired_position - position
        )

    def getHeight(self, position: float):
        return (
            position / Elevator_Constants.kRotationsPerInch
            + Elevator_Constants.kBaseHeight
        )

    def getHeight(self):
        return self.getHeight(self.motor_one.get_position())
