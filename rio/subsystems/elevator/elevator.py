from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6.hardware import TalonFX   
from phoenix6.signal_logger import SignalLogger
from wpilib.sysid import SysIdRoutineLog
from commands2 import Command
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut, VoltageOut
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from phoenix6.controls import StrictFollower
from subsystems.elevator import elevator_constants
from phoenix6 import SignalLogger, ampere, StatusSignal

class Elevator(Subsystem):

    motor_one : TalonFX
    motor_two: TalonFX

    def __init__(self):
         
         # Creating Motors and Configurators
         self.motor_one = TalonFX(elevator_constants.Elevator_Constants.Motor_One_ID)  # CAN ID 1
         self.motor_two = TalonFX(elevator_constants.Elevator_Constants.Motor_Two_ID)
         cfg = TalonFXConfiguration()
         cfg2 = self.motor_one.configurator

         # Gear Ratio and PID values
         cfg.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
         cfg.feedback.sensor_to_mechanism_ratio = elevator_constants.Elevator_Constants.Gear_Ratio
         cfg.slot0.k_g = elevator_constants.Elevator_Constants.MM_KG_AMPS
         cfg.slot0.k_s = elevator_constants.Elevator_Constants.MM_KS_AMPS
         cfg.slot0.k_v = elevator_constants.Elevator_Constants.MM_KV_AMPS_PER_UNIT_OF_TARGET_VELOCITY
         cfg.slot0.k_a = elevator_constants.Elevator_Constants.MM_KA_AMPS_PER_UNIT_OF_TARGET_ACCEL
         cfg.slot0.k_p = elevator_constants.Elevator_Constants.MM_KP_AMPS_PER_UNIT_OF_ERROR_IN_POSITION # An error of 1 rotation results in 2.4 V output
         cfg.slot0.k_i = elevator_constants.Elevator_Constants.MM_KI_AMPS_PER_UNIT_OF_INTEGRATED_ERROR_IN_POSITION
         cfg.slot0.k_d = elevator_constants.Elevator_Constants.MM_KD_AMPS_PER_UNIT_OF_ERROR_IN_VELOCITY

         self.motor_one.setNeutralMode(NeutralModeValue.BRAKE)
        

         # Applying Limit Configurations
         limit_configs = CurrentLimitsConfigs()
         limit_configs.stator_current_limit = elevator_constants.Elevator_Constants.Ampere_Stator_Limit # Note that this is in AMPERES
         limit_configs.stator_current_limit_enable = True

         # Motion Magic Configurations
         motion_magic_configs = cfg.motion_magic
         motion_magic_configs.motion_magic_cruise_velocity = elevator_constants.Elevator_Constants.MM_CRUISE_VELOCITY_ROT_PER_SEC # Target cruise velocity of 80 rps
         motion_magic_configs.motion_magic_acceleration = elevator_constants.Elevator_Constants.MM_ACCELERATION_ROT_PER_SEC2 # Target acceleration of 160 rps/s (0.5 seconds)
         motion_magic_configs.motion_magic_jerk = elevator_constants.Elevator_Constants.MM_JERK_ROT_PER_SEC3 # Target jerk of 1600 rps/s/s (0.1 seconds)

         # Applying all of Settings (and Checking if Error Occurs.)
         self.motor_two.set_control(StrictFollower(elevator_constants.Elevator_Constants.Motor_One_ID))
         cfg2.apply(limit_configs)
         cfg2.apply(cfg)

         # Misc
         self.request = MotionMagicVoltage(0).with_slot(0)
         self.output = DutyCycleOut(elevator_constants.Elevator_Constants.Percent_Power)
         self.output2 = DutyCycleOut(elevator_constants.Elevator_Constants.Neg_Percent_Power)

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
    
    def move_to_l1(self):
        print("Moved to L1")        
        self.motor_one.set_control(self.request.with_position(elevator_constants.Elevator_Constants.L1_Rotations))

    def move_to_l2(self):
        self.motor_one.set_control(self.request.with_position(elevator_constants.Elevator_Constants.L2_Rotations))
        print("Moved to L2")
   
    def move_to_l3(self):
        self.motor_one.set_control(self.request.with_position(elevator_constants.Elevator_Constants.L3_Rotations))
        print("Move to L3")

    def move_to_l4(self):
        self.motor_one.set_control(self.request.with_position(elevator_constants.Elevator_Constants.L4_Rotations))
        print("Moved to L4")

    def move_to_origin(self):
        self.motor_one.set_control(self.request.with_position(elevator_constants.Elevator_Constants.At_Base))
        print("Moved To Base")

    def move_up_gradually(self):
        self.motor_one.set_control(self.output)
        print("Moving Up")

    def move_down_gradually(self):
        self.motor_one.set_control(self.output2)  
        print("Moving Down")

    def stop(self):
        self.motor_one.set_control(DutyCycleOut(elevator_constants.Elevator_Constants.No_Percent_Power))
        print("Stopped")
    
    def periodic(self) -> None:
        """
        Overridden to update dashboard.
        """
        current: StatusSignal[ampere] = self.motor_one.get_torque_current()
        SmartDashboard.putNumber("Elevator left amps", current.value())
        current = self.motor_two.get_torque_current()
        SmartDashboard.putNumber("Elevator right amps", current.value())
