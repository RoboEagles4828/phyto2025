from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import MotionMagicVoltage
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from climber_constants import ClimberConstants

class Climber(Subsystem):
    motor : TalonFX
    
    def __init__(self):
        cfgs = TalonFXConfiguration()
        talonfx_configurator = self.motor.configurator
        limit_configs = CurrentLimitsConfigs()
        limit_configs.stator_current_limit = ClimberConstants.stator_current_limit
        limit_configs.stator_current_limit_enable = ClimberConstants.stator_current_limit_enabler
        talonfx_configurator.apply(limit_configs)
        cfgs.slot0.k_p = ClimberConstants.k_p
        cfgs.slot0.k_i = ClimberConstants.k_i
        cfgs.slot0.k_d = ClimberConstants.k_d
        motion_magic = cfgs.motion_magic
        motion_magic.motion_magic_cruise_velocity = ClimberConstants.motion_magic_cruise_velocity
        motion_magic.motion_magic_acceleration = ClimberConstants.motion_magic_acceleration
        motion_magic.motion_magic_jerk = ClimberConstants.motion_magic_jerk
        talonfx_configurator.apply(cfgs)
        self.motor = TalonFX(ClimberConstants.motor_id)
        self.request = MotionMagicVoltage(ClimberConstants.motion_magic_id)
        

    def climb(self):
        self.motor.set_control(self.request.with_position(ClimberConstants.climb_position_encoder_value))
    def returnToOrigin(self):
        self.motor.set_control(self.request.with_position(ClimberConstants.climb_origin_encoder_value))