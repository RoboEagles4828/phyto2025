import math
from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.configs.config_groups import InvertedValue
from wpilib import *
from phoenix6.controls import StrictFollower
from lib.util.units import Units
from subsystems.elevator.elevator_constants import Elevator_Constants

class Elevator(Subsystem):

    motor_one : TalonFX
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

         # Applying Limit Configurations
         limit_configs = CurrentLimitsConfigs()
         limit_configs.stator_current_limit = Elevator_Constants.kCurrentLimit # Note that this is in AMPERES
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
         self.request = MotionMagicVoltage(0).with_slot(0)
         self.output = DutyCycleOut(0.2)
         self.output2 = DutyCycleOut(-0.2)

         # Create Mechanism Canvas for SmartDashboard
         canvasWidth = 21.0
         canvasHeight = Units.inchesToMeters(Elevator_Constants.kMaxHeight)
         canvas = Mechanism2d(canvasWidth, canvasHeight, Color8Bit(Color.kLightGray))
         origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0.0)
         offset = origin.appendLigament("elevator-offset", canvasWidth/2.0 - Units.inchesToMeters(Elevator_Constants.kSetBack), 0.0, 1.0, Color8Bit())
         mechanism = offset.appendLigament("elevator", Units.inchesToMeters(Elevator_Constants.kBaseHeight), 90.0, Units.inchesToMeters(Elevator_Constants.kThickness), Color8Bit(Color.kBrown))

         SmartDashboard.putData("Elevator/mechanism", canvas)

    def zero(self):
        self.motor_one.set_position(0.0)

    def goToZero(self):
        return self.runOnce(self.goToZero())
    
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
        self.motor_one.set_control(MotionMagicVoltage, position)

    def setHeight(self, height: float):
        position = (height - Elevator_Constants.kBaseHeight) * Elevator_Constants.kRotationsPerInch
        self.setPosition(position)
    
    def closeEnough(self, position: float):
        return self.desired_position>=0.0 and math.fabs(self.desired_position-position)
    
    def getHeight(self, position: float):
        return position / Elevator_Constants.kRotationsPerInch + Elevator_Constants.kBaseHeight
    
    def getHeight(self):
        return self.getHeight(self.motor_one.get_position())