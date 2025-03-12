from commands2 import Command

from wpilib import SmartDashboard

from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.controller import PIDController
from wpimath.units import radiansToDegrees

from lib.util.units import Units

from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.pose.pose import Pose

from subsystems.swerve.tuner_constants import TunerConstants

import math


class PID_Swerve(Command):

    s_Swerve: CommandSwerveDrivetrain
    targetPose: Pose2d
    presice: bool

    # xPID: PIDController = PIDController(0.02, 0.0, 0.0)
    # yPID: PIDController = PIDController(0.02, 0.0, 0.0)
    
    positionTolerance = 1.0
    roughPositionTolerance = 2.5
    maxSpeed = 1.0
    positionKs = 0.02
    positionIZone = 4.0

    # rotationPID = PIDController(0.003, 0.0, 0.0)
    angleTolerance = 1.0
    roughAngleTolerance = 2.5
    maxAngularVelociy = 5.21 / 0.5

    def __init__(self, swerve: CommandSwerveDrivetrain, targetPose: Pose2d, presice: bool):
        super().__init__()
        self.s_Swerve = swerve
        self.targetPose = targetPose
        self.presice = presice

        self.xPID = PIDController(0.005, 0.0, 0.0)
        self.yPID = PIDController(0.005, 0.0, 0.0)
        self.rotationPID = PIDController(0.003, 0.0, 0.0)

        self.xPID.setIZone(PID_Swerve.positionIZone)
        self.xPID.setIntegratorRange(-PID_Swerve.positionKs * 2, PID_Swerve.positionKs * 2)
        self.xPID.setSetpoint(Units.metersToInches(targetPose.X()))
        self.xPID.setTolerance(PID_Swerve.positionTolerance if self.presice else PID_Swerve.roughPositionTolerance)

        self.yPID.setIZone(PID_Swerve.positionIZone)
        self.yPID.setIntegratorRange(-PID_Swerve.positionKs * 2, PID_Swerve.positionKs * 2)
        self.yPID.setSetpoint(Units.metersToInches(targetPose.Y()))
        self.yPID.setTolerance(PID_Swerve.positionTolerance if self.presice else PID_Swerve.roughPositionTolerance)

        self.rotationPID.enableContinuousInput(-180, 180)
        self.rotationPID.setIZone(2.0)
        self.rotationPID.setIntegratorRange(-PID_Swerve.positionKs * 2, PID_Swerve.positionKs * 2)
        self.rotationPID.setSetpoint(targetPose.rotation().degrees())
        self.rotationPID.setTolerance(PID_Swerve.angleTolerance if self.presice else PID_Swerve.roughAngleTolerance)

        self.addRequirements(self.s_Swerve)
    
    def intialize(self):
        super().initialize()
        self.xPID.reset()
        self.yPID.reset()
        self.rotationPID.reset()

    def execute(self):
        self.__init__(self.s_Swerve, self.targetPose, self.presice)
        
        pose: Pose2d = self.s_Swerve.get_state().pose
        # 
        position: Translation2d = pose.translation()
        rotation: Rotation2d = pose.rotation()

        xCorrection = self.xPID.calculate(Units.metersToInches(position.X()))
        xFeedForward = self.positionKs * math.copysign(1, xCorrection)
        xVal = max(-1, min(xCorrection+xFeedForward, 1))
        
        yCorrection = self.yPID.calculate(Units.metersToInches(position.Y()))
        yFeedForward = self.positionKs * math.copysign(1, yCorrection)
        yVal = max(-1, min(yCorrection+yFeedForward, 1))

        corection = self.rotationPID.calculate(rotation.degrees())
        feedForward = 0.02 * math.copysign(1, corection)
        rotationVal = max(-1.0, min(corection+feedForward, 1.0))

        self.s_Swerve.drive(Translation2d(xVal, yVal).__mul__(2), rotationVal*PID_Swerve.maxAngularVelociy, True)

    def isFinished(self):
        return self.xPID.atSetpoint() and self.yPID.atSetpoint() and self.rotationPID.atSetpoint()