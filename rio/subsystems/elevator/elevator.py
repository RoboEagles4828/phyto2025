from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6.hardware import TalonFX   
from phoenix6.signal_logger import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from commands2 import Command
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicTorqueCurrentFOC, DutyCycleOut, VoltageOut
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from phoenix6.controls import StrictFollower
from phoenix6 import SignalLogger, ampere, StatusSignal
from rio.subsystems.elevator.floor import Floor
from subsystems.elevator.elevator_constants import Elevator_Constants

class Elevator(Subsystem):

    motor_one : TalonFX
    motor_two: TalonFX

    def __init__(self):

        # Creating Motors and Configurators
        self.motor_one = TalonFX(Elevator_Constants.Motor_One_ID)  # CAN ID 1
        self.motor_two = TalonFX(Elevator_Constants.Motor_Two_ID)
        cfg = TalonFXConfiguration()
        cfg2 = self.motor_one.configurator

        # Gear Ratio and PID values
        cfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        cfg.feedback.sensor_to_mechanism_ratio = Elevator_Constants.Gear_Ratio
        cfg.slot0.k_g = Elevator_Constants.MM_KG_AMPS
        cfg.slot0.k_s = Elevator_Constants.MM_KS_AMPS
        cfg.slot0.k_v = Elevator_Constants.MM_KV_AMPS_PER_UNIT_OF_TARGET_VELOCITY
        cfg.slot0.k_a = Elevator_Constants.MM_KA_AMPS_PER_UNIT_OF_TARGET_ACCEL
        cfg.slot0.k_p = Elevator_Constants.MM_KP_AMPS_PER_UNIT_OF_ERROR_IN_POSITION
        cfg.slot0.k_i = Elevator_Constants.MM_KI_AMPS_PER_UNIT_OF_INTEGRATED_ERROR_IN_POSITION
        cfg.slot0.k_d = Elevator_Constants.MM_KD_AMPS_PER_UNIT_OF_ERROR_IN_VELOCITY

        self.motor_one.setNeutralMode(NeutralModeValue.BRAKE)

        # Applying Limit Configurations
        limit_configs = CurrentLimitsConfigs()
        limit_configs.stator_current_limit = Elevator_Constants.Ampere_Stator_Limit # Note that this is in AMPERES
        limit_configs.stator_current_limit_enable = True

        # Motion Magic Configurations
        motion_magic_configs = cfg.motion_magic
        motion_magic_configs.motion_magic_cruise_velocity = Elevator_Constants.MM_CRUISE_VELOCITY_ROT_PER_SEC
        motion_magic_configs.motion_magic_acceleration = Elevator_Constants.MM_ACCELERATION_ROT_PER_SEC2
        motion_magic_configs.motion_magic_jerk = Elevator_Constants.MM_JERK_ROT_PER_SEC3

        # Applying all of Settings (and Checking if Error Occurs.)
        self.motor_two.set_control(StrictFollower(Elevator_Constants.Motor_One_ID))
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

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction)

    def move_to_floor(self, floor: Floor) -> Command:
        """
        Returns a new command to sets the target rotations to the given Floor.
        Designed for button click or as part of a command group.
        """
        self.runOnce(
            lambda: self.motor_one.set_control(
                self.request.with_position(floor.rotations)
            )
        )

    def move_up_gradually(self) -> Command:
        """
        Returns a new command to manually move up.
        Designed to run while a button is held. Use the stop command on release.
        """
        self.run(
            lambda: self.motor_one.set_control(
                self.dutyCycle.with_output(Elevator_Constants.MANUAL_UP_DUTY_CYCLE)
            )
        )

    def move_down_gradually(self) -> Command:
        """
        Returns a new command to manually move down.
        Designed to run while a button is held. Use the stop command on release.
        """
        self.run(
            lambda: self.motor_one.set_control(
                self.dutyCycle.with_output(Elevator_Constants.MANUAL_DOWN_DUTY_CYCLE)
            )
        )

    def stop(self) -> Command:
        """
        Returns a new command to remove power from motors. Elevator should drift downward.
        Designed for a button click or part of a command group. Also good for the release
        of the button used with manual motion.
        """
        self.runOnce(
            lambda: self.motor_one.set_control(
                DutyCycleOut(Elevator_Constants.MANUAL_NO_POWER)
            )
        )

    def periodic(self) -> None:
        """
        Overridden to update dashboard.
        """
        current: StatusSignal[ampere] = self.motor_one.get_torque_current()
        SmartDashboard.putNumber("Elevator left amps", current.value())
        current = self.motor_two.get_torque_current()
        SmartDashboard.putNumber("Elevator right amps", current.value())
