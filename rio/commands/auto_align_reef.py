from commands2 import Command

from phoenix6.swerve.requests import SwerveRequest

from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath import units

from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain

class AutoAlignReef(Command):

    def __init__(self, swerve: CommandSwerveDrivetrain):

        self.swerve = swerve
        
        self.translationalController = ProfiledPIDController(5.0, 0.0, 0.0, TrapezoidProfile.Constraints(3,3))
        self.rotationalController = ProfiledPIDController(7.5, 0.0, 0.0, TrapezoidProfile.Constraints(3,3))

        self.translationalController.setTolerance(units.inchesToMeters(1))
        self.rotationalController.setTolerance(units.degreesToRadians(1))

        self.addRequirements(self.swerve)