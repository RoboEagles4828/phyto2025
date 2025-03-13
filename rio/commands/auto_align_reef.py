from commands2 import Command

from phoenix6.swerve import requests

from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain

from pathplannerlib.auto import AutoBuilder, PathConstraints, PathPlannerPath

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from wpimath import units

import math


import numpy as np

class AutoAlignReef(Command):

    def __init__(self, swerve: CommandSwerveDrivetrain, targetPose: Pose2d, presice: bool):

        self.s_Swerve = swerve
        self.targetPose = targetPose
        self.presice = presice

        self.drive = requests.FieldCentric().with_drive_request_type(requests.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)

        self.translationController = ProfiledPIDController(5, 0.0, 0.0, TrapezoidProfile.Constraints(5.0, 3.0))
        self.rotationController = ProfiledPIDController(7.5, 0.0, 0.0, TrapezoidProfile.Constraints(5.0, 3.0))

        #declare tolerance
        self.translationController.setTolerance(units.inchesToMeters(1.0))
        self.rotationController.setTolerance(units.degreesToRadians(1.0))

        self.addRequirements(self.s_Swerve)

    def initialize(self):
        self.currentPose = self.s_Swerve.get_state().pose
        
        self.rotationController.enableContinuousInput(-math.pi, math.pi)

        self.velocity = 1* self.projection(Translation2d(self.s_Swerve.get_state().speeds.vx, self.s_Swerve.get_state().speeds.vy), self.targetPose.translation()-self.currentPose.translation())
        
        self.distance = self.targetPose.translation().distance(self.currentPose.translation())

        self.translationController.reset(self.distance, self.velocity)
        self.rotationController.reset(self.angle_modulus(self.currentPose.rotation().radians()), self.s_Swerve.get_state().speeds.omega)

    def execute(self):
        self.position = self.s_Swerve.get_state().pose
        self.distance = self.targetPose.translation().distance(self.position.translation())

        self.translationController.reset(self.distance, self.translationController.getSetpoint().velocity)

        self.rotationPIDOutput = self.rotationController.calculate(self.angle_modulus(self.position.rotation().radians()), self.targetPose.rotation().radians())
        self.omega = self.rotationController.getSetpoint().velocity + self.rotationPIDOutput

        scalar = self.scalar(self.distance)

        self.drivePIDOutput = self.translationController.calculate(self.distance, 0)
        self.driveSpeed =   -1 * scalar * self.translationController.getSetpoint().velocity + self.drivePIDOutput
        self.direction = Rotation2d(self.currentPose.X()-self.targetPose.X(), self.currentPose.Y()-self.targetPose.Y())


        self.s_Swerve.set_control(self.drive.with_velocity_x(self.driveSpeed*self.direction.cos()).with_velocity_y(self.driveSpeed*self.direction.sin()).with_rotational_rate(self.omega))

        print(str(self.translationController.getGoal()))
        print("Distance" + str(self.distance))


    def isFinished(self):

        return self.translationController.atGoal() and self.rotationController.atGoal()






    def projection(self, v1, onto):
        velocity = np.array([v1[0], v1[1]])
        translation = np.array([onto[0], onto[1]])
        
        projection = (np.dot(velocity, translation) / np.dot(translation, translation)) * translation
        
        if np.dot(projection, translation) > 0:
            return -np.sqrt(np.dot(projection, projection))
        else:
            return np.sqrt(np.dot(projection, projection))
        
    def angle_modulus(self, angle_deg):
        """Normalizes an angle to the range -180 to 180 degrees."""
        mod_angle = angle_deg % 360
        if mod_angle > 180:
            return mod_angle - 360
        return mod_angle
    
    def scalar(self, distance, min_distance=-0.1, max_distance=0.1):
        if distance > max_distance:
            return 1.0
        elif min_distance < distance < max_distance:
            return np.clip((1 / (max_distance - min_distance)) * (distance - min_distance), 0, 1)
        else:
            return 0.0
