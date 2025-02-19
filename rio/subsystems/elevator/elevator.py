from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut, PositionVoltage
from phoenix6.configs import Slot0Configs, TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6 import StatusCode
from wpilib import *
from phoenix6.controls import StrictFollower
from lib.util.units import Units

from subsystems.elevator.elevator_constants import Elevator_Constants
from general_constants.field_constants import FieldConstants

from wpilib import Mechanism2d, MechanismLigament2d, MechanismRoot2d, Color8Bit
import math

class Elevator(Subsystem):

    motor_one : TalonFX
    motor_two: TalonFX
    desired_position: float

    def __init__(self):
         
         # Creating Motors and Configurators, change the names of motor1 and two after we get the final robot
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

         # Applying Limit Configurations
         limit_configs = CurrentLimitsConfigs()
         limit_configs.stator_current_limit = Elevator_Constants.kCurrentLimit   # Note that this is in AMPERES
         limit_configs.stator_current_limit_enable = Elevator_Constants.kCurrentLimitEnable

         # Motion Magic Configurations
         motion_magic_configs = cfg.motion_magic
         motion_magic_configs.motion_magic_cruise_velocity = Elevator_Constants.kCruiseVelocity # Target cruise velocity of 80 rps
         motion_magic_configs.motion_magic_acceleration = Elevator_Constants.kMagicAcceleration # Target acceleration of 160 rps/s (0.5 seconds)
         motion_magic_configs.motion_magic_jerk = Elevator_Constants.kMagicJerk # Target jerk of 1600 rps/s/s (0.1 seconds)

         # Applying all of Settings (and Checking if Error Occurs.)
         self.motor_two.set_control(StrictFollower(Elevator_Constants.kMotor1ID))
         cfg2.apply(limit_configs)
         cfg2.apply(cfg)

         self.motor_one.stopMotor()

         # Misc
         self.request = MotionMagicVoltage(0).with_slot(0).with_enable_foc(True)
         self.output = DutyCycleOut(0.2)
         self.output2 = DutyCycleOut(-0.2)


         # Create Mechanism Canvas for SmartDashboard
         canvasWidth = 0.533
         canvasHeight = Units.inchesToMeters(Elevator_Constants.kMaxHeight)
         canvas = Mechanism2d(canvasWidth, canvasHeight, Color8Bit(Color.kBlack))
         origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0.0)
         offset = origin.appendLigament("elevator-offset", canvasWidth/2.0 - Units.inchesToMeters(Elevator_Constants.kSetBack), 0.0, 4.0, Color8Bit(Color.kYellow))
         self.mechanism = offset.appendLigament("elevator", Units.inchesToMeters(Elevator_Constants.kBaseHeight), 90.0, 4.0, Color8Bit(Color.kOrange))

         SmartDashboard.putData("Elevator/mechanism", canvas)

    def zero(self):
        self.motor_one.set_control(self.request.with_position(0.0))
        self.mechanism.setLength(Units.inchesToMeters(Elevator_Constants.kBaseHeight))

    def goToZero(self):
        return self.runOnce(self.zero())
    
    def stop(self):
        self.motor_one.stopMotor()

    def stopCommand(self):
        return self.runOnce(self.stop())

    def move_up_gradually(self):
        self.motor_one.set_control(self.output)
        print("Moving Up")

    def move_down_gradually(self):
        self.motor_one.set_control(self.output2)  
        print("Moving Down")

    def setPosition(self, position: float):
        self.desired_position = position
        self.motor_one.set_control(self.request.with_position(position))

    def setHeight(self, height: float):
        position = (height - Elevator_Constants.kBaseHeight) * Elevator_Constants.kRotationsPerInch
        self.mechanism.setLength(Units.inchesToMeters(height))
        self.setPosition(position)
    
    def closeEnough(self, position: float):
        return self.desired_position>=0.0 and math.fabs(self.desired_position-position)
    
    def getHeight(self):
        return self.motor_one.get_position().value_as_double / Elevator_Constants.kRotationsPerInch + Elevator_Constants.kBaseHeight
    
    def getHeightPosition(self):
        return self.motor_one.get_position().value_as_double
    
    def simSetPosition(self, position: float):
        talon = self.motor_one
        talon.__sim_rotor_pos = position
    
    def periodic(self):
        SmartDashboard.putNumber("Elevator/Height", self.getHeight())
        SmartDashboard.putNumber("Elevator/Position", self.motor_one.get_position().value_as_double)