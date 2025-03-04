import math

from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from commands2.conditionalcommand import ConditionalCommand
from phoenix6.hardware import TalonFX
from phoenix6.signal_logger import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from commands2 import Command
from phoenix6.signals import NeutralModeValue, GravityTypeValue
from phoenix6.controls import MotionMagicTorqueCurrentFOC, DutyCycleOut, VoltageOut, TorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, TalonFXConfigurator, ClosedLoopRampsConfigs, OpenLoopRampsConfigs
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from lib.util.units import Units
from phoenix6.controls import StrictFollower, MotionMagicVoltage, Follower, PositionVoltage
from phoenix6 import SignalLogger, ampere, StatusSignal
from wpilib.shuffleboard import Shuffleboard
from wpilib import DigitalInput
from wpimath.filter import Debouncer
from wpilib.sysid import SysIdRoutineLog

from subsystems.elevator.elevator_constants import Elevator_Constants


class Elevator(Subsystem):

    def __init__(self):
        

        # Creating Motors and Configurators
        self.rightMotorLeader = TalonFX(Elevator_Constants.kRightMotorID)  # CAN ID 1
        self.leftMotorFollower = TalonFX(Elevator_Constants.kLeftMotorID)  # CAN ID 2
        self.motorCfg = TalonFXConfiguration()
        
        # Configure values
        self.motorCfg.feedback.sensor_to_mechanism_ratio = Elevator_Constants.kGearRatio
        self.motorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motorCfg.slot0.gravity_type = GravityTypeValue.ELEVATOR_STATIC
        self.motorCfg.slot0.k_g = Elevator_Constants.kGravity
        self.motorCfg.slot0.k_s = Elevator_Constants.kStatic
        self.motorCfg.slot0.k_v = Elevator_Constants.kVelocity
        self.motorCfg.slot0.k_a = Elevator_Constants.kAcceleration
        self.motorCfg.slot0.k_p = Elevator_Constants.kPorportional # An error of 1 rotation results in 2.4 V output
        self.motorCfg.slot0.k_i = Elevator_Constants.kIntegral
        self.motorCfg.slot0.k_d = Elevator_Constants.kDerivative

        #Configure for L4
        self.motorCfg.slot1.k_v = 0.1
        self.motorCfg.slot1.k_p = 5.6

        # Apply limit configurations here
        limit_configs = CurrentLimitsConfigs()
        limit_configs.stator_current_limit = Elevator_Constants.kCurrentLimit   # Note that this is in AMPERES
        limit_configs.stator_current_limit_enable = Elevator_Constants.kCurrentLimitEnable

        # Motion Magic Configurations
        self.motorCfg.motion_magic.motion_magic_cruise_velocity = Elevator_Constants.kCruiseVelocity
        self.motorCfg.motion_magic.motion_magic_acceleration = Elevator_Constants.kMagicAcceleration
        self.motorCfg.motion_magic.motion_magic_jerk = Elevator_Constants.kMagicJerk
        self.motorCfg.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.5
        self.motorCfg.open_loop_ramps.voltage_open_loop_ramp_period = 0.5

        # Add motor controls (follow and motionmagic)
        self.leftMotorFollower.set_control(Follower(Elevator_Constants.kRightMotorID, True))

        self.request = PositionVoltage(0.0).with_enable_foc(True)
        self.dutyCycle = DutyCycleOut(0.0)


        # Apply configs
        self.rightMotorLeader.configurator.apply(self.motorCfg)
        self.leftMotorFollower.configurator.apply(self.motorCfg)

        self.bottomLimitSwitch = DigitalInput(Elevator_Constants.kBottomLimitSwitchID)
        self.topLimitSwitch = DigitalInput(Elevator_Constants.kTopLimitSwitchID)

        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

        self.desiredPosition = 0.0
        self.nextTargetPosition = 0.0
        self.levelingSlot = 0

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
                lambda output: self.rightMotorLeader.set_control(
                    self.voltageOut.with_output(output),
                ),
                lambda log: None,
                self,
            ),
        )

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction)

    
    def move_to_position_execute(self):
        """
        The command that is run during manual cycle. Doesnt require a position input
        """

        return self.startRun(
            lambda: self.setTargetRotation(self.nextTargetPosition),
            lambda: self.rightMotorLeader.set_control(
                self.request.with_position(self.nextTargetPosition).with_slot(self.levelingSlot).with_limit_forward_motion(not(self.topLimitSwitch.get())).with_limit_reverse_motion(self.bottomLimitSwitch.get())
            ))
        # return ConditionalCommand(
        #     self.startRun(
        #     lambda: self.setTargetRotation(self.nextTargetPosition),
        #     lambda: self.rightMotorLeader.set_control(
        #         self.request.with_position(self.nextTargetPosition).with_slot(1).with_limit_forward_motion(not(self.topLimitSwitch.get())).with_limit_reverse_motion(self.bottomLimitSwitch.get())
        #     )),
        #     self.startRun(
        #     lambda: self.setTargetRotation(self.nextTargetPosition),
        #     lambda: self.rightMotorLeader.set_control(
        #         self.request.with_position(self.nextTargetPosition).with_slot(0).with_limit_forward_motion(not(self.topLimitSwitch.get())).with_limit_reverse_motion(self.bottomLimitSwitch.get())
        #     )),
        #     self.movingToL4()
        # )
    
    def move_to_zero(self):
        return self.startRun(
            lambda: self.setTargetRotation(self.nextTargetPosition),
            lambda: self.rightMotorLeader.set_control(
                self.dutyCycle.with_output(-.5).with_limit_reverse_motion(self.bottomLimitSwitch.get())
            )
        )


    def move_to_position(self, position : float, slot: int = 0) -> Command:
        """
        Returns a new command to sets the target rotations to the given position.
        Designed for button click or as part of a command group.
        """
        return self.startRun(
            lambda: self.setTargetRotation(position),
            lambda: self.rightMotorLeader.set_control(
                self.request.with_position(position).with_slot(slot)
            )
        )
    
    def stop(self) -> Command:
        """
        Returns a new command to remove power from motors. Elevator should drift downward.
        Designed for a button click or part of a command group. Also good for the release
        of the button used with manual motion.
        """
        return self.runOnce(
            lambda: self.rightMotorLeader.set_control(
                DutyCycleOut(0)
            )
        )
    
    def move_up_gradually(self) -> Command:
        """
        Returns a new command to manually move up.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.dutyCycle.with_output(Elevator_Constants.kManualOut).with_limit_forward_motion(not(self.topLimitSwitch.get()))
            )
        )

    def move_down_gradually(self) -> Command:
        """
        Returns a new command to manually move down.
        Designed to run while a button is held. Use the stop command on release.
        """
        return self.run(
            lambda: self.rightMotorLeader.set_control(
                self.dutyCycle.with_output(-Elevator_Constants.kManualOut).with_limit_reverse_motion(self.bottomLimitSwitch.get())
            )
        )
    
    def setTargetRotation(self, position):
        self.desiredPosition = position

    def setNextTargetRotation(self, position):
        self.nextTargetPosition = position
        self.levelingSlot = 0
        if self.nextTargetPosition>=3.5:
            self.levelingSlot = 1

    def periodic(self):
        self.set_motor_zero()
        SmartDashboard.putNumber("Elevator/Position", self.getPosition())
        SmartDashboard.putNumber("Elevator/Velocity", self.getVelocity())
        SmartDashboard.putNumber("Elevator/Desired Position", self.desiredPosition)
        SmartDashboard.putNumber("Elevator/Voltage", self.rightMotorLeader.get_motor_voltage().value)
        SmartDashboard.putBoolean("Elevator/Bottom Limit", self.bottomLimitSwitch.get())
        SmartDashboard.putBoolean("Elevator/Top Limit", self.topLimitSwitch.get())

        SmartDashboard.putNumber("Elevator/Next Target", self.nextTargetPosition)
        



    def set_motor_zero(self):
        if self.debouncer.calculate(self.bottomLimitSwitch.get()):
            self.rightMotorLeader.set_position(0.0)

    def getPosition(self) -> float:
        return self.rightMotorLeader.get_position().value

    def getVelocity(self) -> float:
        return self.rightMotorLeader.get_velocity().value