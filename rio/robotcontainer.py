#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from commands2.instantcommand import InstantCommand
from commands2.command import Command
from commands2.selectcommand import SelectCommand
from commands2.sequentialcommandgroup import SequentialCommandGroup

from subsystems.swerve.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians
from wpilib import Joystick, RobotBase, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from pathplannerlib.auto import AutoBuilder, PathPlannerPath

from general_constants.field_constants import FieldConstants
from general_constants.field_constants import ReefFace, ReefPoints
from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.elevator import Elevator
from subsystems.pose.pose import Pose

from phoenix6.swerve import requests

from commands.pid_swerve import PID_Swerve

from lib.util.units import Units

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.85
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.2)
            .with_rotational_deadband(
                self._max_angular_rate * 0.2
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.elevator = Elevator()
        self.pose = Pose(self.drivetrain)
        # Configure the button bindings
        self.pathFind = self.drivetrain.driveToPoseThenFollowPath("PegA_Alignment")

        print("ab pose" + str(ReefFace.AB.alignLeft))

        self.alignLeftCommands: dict[ReefFace, Command] = {}

        for face in ReefFace:
            self.populateCommandList(face)

        # self.pid_test = PID_Swerve(self.drivetrain, ReefFace.AB.alignLeft, True)

        print(self.alignLeftCommands)

        # for face in ReefFace:


        self.configureButtonBindings()

        self.autoChooser = AutoBuilder.buildAutoChooser("None")
        SmartDashboard.putData("AutoChooser",self.autoChooser)

    
    def driveToPose(self)-> Command:
        return SequentialCommandGroup(PID_Swerve(self.drivetrain, self.pose.neartestFace(self.drivetrain.getPose().translation(), False), False), True).andThen(lambda: self.drivetrain.manualOperatorPerspectiveOverride())
    
    def populateCommandList(self, face: ReefFace):
        self.alignLeftCommands[face] = SequentialCommandGroup(PID_Swerve(self.drivetrain, face.alignLeft, True))

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        if RobotBase.isSimulation():
            negative_value = 1
        else:
            negative_value=-1
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        negative_value*self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        negative_value*self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        negative_value*self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        # self._joystick.b().whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._point.with_module_direction(
        #             Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
        #         )
        #     )
        # )

        self._joystick.a().whileTrue((self.pathFind.alongWith(self.elevator.runOnce(lambda: self.elevator.setHeight(ReefPoints.PointAL4.m_elevatorHeight)))).andThen(self.elevator.runOnce(lambda: self.elevator.zero())))
        # self._joystick.y().onTrue(PID_Swerve(self.drivetrain, lambda: self.pose.getPose(), lambda: self.pose.neartestFace(self.pose.getPose().translation(), False), True))
        # self._joystick.x().onTrue(PID_Swerve(self.drivetrain, ReefFace.GH.alignLeft, False))
        self._joystick.x().onTrue(SelectCommand(self.alignLeftCommands, lambda: self.pose.neartestFace(self.pose.pose.translation(), False)))
        self._joystick.b().onTrue(self.elevator.runOnce(lambda: self.elevator.zero()))

        # self._joystick.rightBumper().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.set_operator_perspective_forward(-self.drivetrain.getPigeonRotation2d())))
        self._joystick.leftTrigger().whileTrue(self.drivetrain.run(lambda: self.drivetrain.pigeon2.set_yaw(-55)))
        # self._joystick.rightBumper().whileTrue(self.pid_test)
        # self._joystick.rightBumper().onTrue(InstantCommand(lambda: self.drivetrain.zeroPigeon()))
        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        auto = self.autoChooser.getSelected()

        return auto
    

