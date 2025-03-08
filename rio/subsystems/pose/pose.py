from commands2 import Subsystem

from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain

from general_constants.field_constants import FieldConstants, ReefFace

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from lib.util.units import Units
from lib.util.convenientmath import inputModulus

from wpilib import SmartDashboard
class Pose(Subsystem):

    def __init__(self, swerve: CommandSwerveDrivetrain):
        self.swerve = swerve
        self.pose = self.swerve.getPose()
        self.bearingAngle = 0
        self.closestFace = ReefFace.AB

    def otherAlliance(self, position: Translation2d)-> Translation2d:
        """
        Converts a blue alliance pose2d to a red alliance pose2d
        """
        return Translation2d(FieldConstants.fieldWidth - position.X(), FieldConstants.fieldWidth - position.Y())

    def flipIfRed(self, position: Translation2d, allianceColor: bool)->Translation2d:
        """
        Checks to see if the position needs to be flip based on the robot alliance.
        If robot is red, set the allianceColor parameter to true.
        If robot is blue, set the allianceColor Paramter to false.
        """
        return self.otherAlliance(position) if allianceColor else position

    def reefBearing(self, robotTranslation: Translation2d, allianceColor: bool):
        """
        Finds the bearing angle of the robot relative to the reef center.
        """
        reefCenter = self.flipIfRed(FieldConstants.reefCenter, allianceColor)
        relativePosition =  reefCenter-robotTranslation

        return relativePosition.angle()

    def neartestFace(self, position: Translation2d, allianceColor: bool):

        
        reefBearing = self.reefBearing(position, allianceColor)

        if(allianceColor):
            reefBearing = reefBearing.__add__(Rotation2d.fromDegrees(180))
        
        bearingAngle = inputModulus(reefBearing.degrees(), -180, 180)

        
        

        # self.bearingAngle = bearingAngle

        if bearingAngle>150 or bearingAngle< -150:
            
            return ReefFace.GH
        elif bearingAngle>90:
            
            return ReefFace.EF
        elif bearingAngle>30:
            
            return ReefFace.CD
        elif bearingAngle>-30:
            
            return ReefFace.AB
        elif bearingAngle>-90:
            
            return ReefFace.KL
        else:
            
            return ReefFace.IJ
    
    def getPose(self):
        return self.swerve.get_state().pose

    
    def getTranslation(self):
        return self.getPose().translation()
    
    def closeFace(self):
        return self.closestFace
        
    def periodic(self):
        # 
        self.pose = self.swerve.getPose()
        self.getPose()
        self.bearingAngle = self.reefBearing(self.getTranslation(), False)
        self.closestFace = self.neartestFace(self.getTranslation(), False)

        
        
        
        