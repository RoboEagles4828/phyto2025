import math

from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6.hardware import TalonFX
from phoenix6.signal_logger import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from commands2 import Command
from phoenix6.signals import NeutralModeValue, GravityTypeValue
from phoenix6.controls import MotionMagicTorqueCurrentFOC, DutyCycleOut, VoltageOut, TorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, TalonFXConfigurator
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from lib.util.units import Units
from phoenix6.controls import StrictFollower, MotionMagicVoltage, Follower
from phoenix6 import SignalLogger, ampere, StatusSignal
from wpilib.shuffleboard import Shuffleboard
from wpilib import DigitalInput

from subsystems.elevator.floor import Floor
from subsystems.elevator.elevator_constants import Elevator_Constants


class Elevator(Subsystem):

    def __init__(self):

        # Creating Motors and Configurators
        self.rightMotorLeader = TalonFX(Elevator_Constants.kRightMotorID)  # CAN ID 1
        self.leftMotorFollower = TalonFX(Elevator_Constants.kLeftMotorID)  # CAN ID 2
        self.rightMotorCfg = TalonFXConfiguration()

        # Gear Ratio and PID values
        # self.rightMotorCfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.rightMotorCfg.feedback.sensor_to_mechanism_ratio = Elevator_Constants.kGearRatio
        self.rightMotorCfg.slot0.gravity_type = GravityTypeValue.ELEVATOR_STATIC
        self.rightMotorCfg.slot0.k_g = Elevator_Constants.kGravity
        self.rightMotorCfg.slot0.k_s = Elevator_Constants.kStatic
        self.rightMotorCfg.slot0.k_v = Elevator_Constants.kVelocity
        self.rightMotorCfg.slot0.k_a = Elevator_Constants.kAcceleration
        self.rightMotorCfg.slot0.k_p = Elevator_Constants.kPorportional # An error of 1 rotation results in 2.4 V output
        self.rightMotorCfg.slot0.k_i = Elevator_Constants.kIntegral
        self.rightMotorCfg.slot0.k_d = Elevator_Constants.kDerivative

        self.rightMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE

        

        # Applying Limit Configurations
        self.rightMotorCfg.current_limits.stator_current_limit = Elevator_Constants.kCurrentLimit  # Note that this is in AMPERES
        self.rightMotorCfg.current_limits.stator_current_limit_enable = True

        # Motion Magic Configurations
        self.rightMotorCfg.motion_magic.motion_magic_cruise_velocity = (
            Elevator_Constants.kCruiseVelocity
        )
        self.rightMotorCfg.motion_magic.motion_magic_acceleration = (
            Elevator_Constants.kMagicAcceleration
        )
        self.rightMotorCfg.motion_magic.motion_magic_jerk = Elevator_Constants.kMagicJerk

        self.leftMotorCfg = self.rightMotorCfg
        self.leftMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.leftMotorFollower.set_control(Follower(Elevator_Constants.kRightMotorID, True))

        # Applying all of Settings (and Checking if Error Occurs.)
        self.rightMotorLeader.configurator.apply(self.rightMotorCfg)
        self.leftMotorFollower.configurator.apply(self.leftMotorCfg)

        # Misc
        self.request = MotionMagicTorqueCurrentFOC(0.0)

        self.dutyCycle = DutyCycleOut(0.0)
        self.voltageOut = VoltageOut(0.0)
        self.torqueCurrent = TorqueCurrentFOC(0.0)

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
                lambda output: self.rightMotorLeader.set_control(
                    self.voltageOut.with_output(output),
                ),
                lambda log: None,
                self,
            ),
        )

        self.limitSwitch = DigitalInput(Elevator_Constants.kLimitSwitchID) #true means activated
        self.targetRotation = 0.0
        self.requestedAmps = 0.0

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
        SmartDashboard.putNumber("Elevator/Torque_Amps", self.requestedAmps)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction)
    
    def move_to_position(self, position:float)-> Command:
        """
        Returns a new command to sets the target rotations to the given position.
        Designed for button click or as part of a command group.
        """
        self.targetRotation = position
        return self.runOnce(
            lambda: self.rightMotorLeader.set_control(
                self.request.with_position(position)
            )
        )

    def move_to_floor(self, floor: Floor) -> Command:
        """
        Returns a new command to sets the target rotations to the given Floor.
        Designed for button click or as part of a command group.
        """
        return self.runOnce(
            lambda: self.rightMotorLeader.set_control(
                self.request.with_position(floor.rotations).with_slot(0)
            )
        )

    def move_up_gradually(self) -> Command:
        """
        Returns a new command to manually move up.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.dutyCycle.with_output(Elevator_Constants.kManualUpDutyCycle)
            )
        )

    def move_down_gradually(self) -> Command:
        """
        Returns a new command to manually move down.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.dutyCycle.with_output(Elevator_Constants.kManualDownDutyCycle).with_limit_reverse_motion(self.limitSwitch.get())
            )
        )
    
    def move_up_through_torque(self)-> Command:
        """
        Returns a new command to move up through torque
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.torqueCurrent.with_output(6)
            )
        )
    
    def move_down_through_torque(self)-> Command:
        """
        Returns a new command to move down through torque
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.torqueCurrent.with_output(-0.5).with_limit_reverse_motion(self.limitSwitch.get())
            )
        )
    
    def move_to_current_position(self)->Command:

        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.request.with_position(self.rightMotorLeader.get_position().value).with_limit_reverse_motion(self.limitSwitch.get())
            )
        ) 
    def updateAmps(self, addedAmps):
        """
        Returns a new command to update the requested amps
        """
        self.requestedAmps = self.requestedAmps + addedAmps

    def stop(self) -> Command:
        """
        Returns a new command to remove power from motors. Elevator should drift downward.
        Designed for a button click or part of a command group. Also good for the release
        of the button used with manual motion.
        """
        return self.runOnce(
            lambda: self.rightMotorLeader.set_control(
                DutyCycleOut(Elevator_Constants.kManualNoPower)
            )
        )
    
    def zero_rotations(self) -> Command:
        """
        Returns a new command to zero the elevator's position.
        """
        return self.runOnce(lambda: self.rightMotorLeader.set_position(0.0))
    
    def zero_position(self):
        """
        Moves the robot down until the limit switch is activated
        """
        return self.move_down_gradually().until(lambda: self.limitSwitch.get())
    
    def set_motor_zero(self):
        if self.limitSwitch.get():
            self.rightMotorLeader.set_position(0.0)
    
    def isSwitchTriggered(self):
        return not(self.limitSwitch.get())

    def periodic(self) -> None:
        """
        Overridden to update dashboard.
        """
        self.set_motor_zero()
        current: StatusSignal[ampere] = self.rightMotorLeader.get_torque_current()
        SmartDashboard.putNumber("Elevator/Elevator left amps", current.value)
        current = self.leftMotorFollower.get_torque_current()
        SmartDashboard.putNumber("Elevator/Elevator right amps", current.value)
        SmartDashboard.putNumber("Elevator/Elevator position", self.rightMotorLeader.get_position().value)
        SmartDashboard.putBoolean("Elevator/Limit Switch Value", self.limitSwitch.get())
        SmartDashboard.putNumber("Elevator/Desired Position", self.targetRotation)
        SmartDashboard.putNumber("Elevator/Torque_Amps", self.requestedAmps)
    # Functions below this point may be archived or deleted later

    def setPosition(self, position: float):
        self.desired_position = position
        self.rightMotorLeader.set_control(MotionMagicVoltage, position)

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
        return self.getHeight(self.rightMotorLeader.get_position())
