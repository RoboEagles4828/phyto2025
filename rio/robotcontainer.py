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
from commands2.conditionalcommand import ConditionalCommand
from commands2.sequentialcommandgroup import SequentialCommandGroup
from commands2.selectcommand import SelectCommand

from commands.pid_swerve import PID_Swerve

from subsystems.swerve.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians
from wpilib import Joystick, RobotBase, SmartDashboard
from wpilib.shuffleboard import Shuffleboard

from pathplannerlib.auto import AutoBuilder

from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.elevator import Elevator
from subsystems.cannon.cannon import Cannon
from subsystems.hopper.hopper import Hopper
from subsystems.vision.vision  import VisionSubsystem
from subsystems.led.led import LED

from general_constants.field_constants import ReefFace
from subsystems.pose.pose import Pose

from pathplannerlib.auto import NamedCommands

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    elevatorL1 = 1.093
    elevatorL2 = 1.6
    elevatorL3 = 2.355
    elevatorL4 = 3.7
    cannonL1Top = (elevatorL1 + elevatorL2) / 2

    def __init__(self) -> None:

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.85
        )  # 3/4 of a rotation per second max angular velocity

        driveDeadbandFactor = 0.07
        """Max speed (m/s) multipled by this value to set a m/s deadband."""
        rotationalDeadbandFactor = 0.07
        """Max angular rate (rad/s) multipled by this value to set a rad/s deadband."""
        self.reefAlignSlownessFactor = 4.0
        """
        In robot oriented reef alignment slow down by this factor.
        Used to reduce speed in the command. Also used to keep the
        deadbands aligned on the same stick defection being dead.
        """

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * driveDeadbandFactor)
            .with_rotational_deadband(
                self._max_angular_rate * rotationalDeadbandFactor
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._reefAlign = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * (driveDeadbandFactor / self.reefAlignSlownessFactor))
            .with_rotational_deadband(
                self._max_angular_rate * (rotationalDeadbandFactor / self.reefAlignSlownessFactor)
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self._operator_joystick = commands2.button.CommandXboxController(1)

        self._test_joystick = commands2.button.CommandXboxController(2)

        self.operator1 = commands2.button.CommandGenericHID(3)
        self.operator2 = commands2.button.CommandGenericHID(4)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.elevator = Elevator()
        self.led = LED()
        self.hopper = Hopper()
        self.cannon = Cannon()
        self.vision = VisionSubsystem(self.drivetrain)
        self.pose = Pose(self.drivetrain)

        self.alignLeftCommands: dict[ReefFace, Command] = {}
        self.alignRightCommands: dict[ReefFace, Command] = {}

        for face in ReefFace:
            self.populateCommandList(face)
        
        NamedCommands.registerCommand("Elevator to L1", self.elevator.move_to_position(1.093, 0).withTimeout(3.0))
        NamedCommands.registerCommand("Elevator to Zero", self.elevator.move_to_zero())
        NamedCommands.registerCommand("Hopper Intake", self.hopper.intake())
        NamedCommands.registerCommand("Cannon L1", self.cannon.createPlaceCoralCommand(self.isPlaceCoralL1).withTimeout(1.0))
        NamedCommands.registerCommand("Load Coral to Cannon", self.cannon.loadCoral())
        NamedCommands.registerCommand("Elevator Stop", self.elevator.stop())
        NamedCommands.registerCommand("Cannon Stop", self.cannon.stop())

        # Configure the button bindings

        self.configureButtonBindings()
        self.configureOperatorBindings()

        self.autoChooser = AutoBuilder.buildAutoChooser("None")
        self.addNoPathAutos()
        SmartDashboard.putData("AutoChooser",self.autoChooser)

    def populateCommandList(self, face: ReefFace):
        self.alignLeftCommands[face] = SequentialCommandGroup(PID_Swerve(self.drivetrain, face.alignLeft, False))
        self.alignRightCommands[face] = SequentialCommandGroup(PID_Swerve(self.drivetrain, face.alignRight, False))

    def isPlaceCoralL1(self) -> bool:
        """Returns true if the elevator height is proper for an L1 placement."""
        return (self.elevator.getPosition() < RobotContainer.cannonL1Top)

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

        self.hopper.setDefaultCommand(self.hopper.stop())
        self.cannon.setDefaultCommand(self.cannon.stop())
        self.elevator.setDefaultCommand(self.elevator.stop())

        # self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        # self._joystick.b().whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._point.with_module_direction(
        #             Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
        #         )
        #     )
        # )

        # operator buttons
        self._operator_joystick.a().onTrue(self.elevator.runOnce(lambda: self.elevator.setNextTargetRotation(RobotContainer.elevatorL1)))
        self._operator_joystick.x().onTrue(self.elevator.runOnce(lambda: self.elevator.setNextTargetRotation(RobotContainer.elevatorL2)))
        self._operator_joystick.b().onTrue(self.elevator.runOnce(lambda: self.elevator.setNextTargetRotation(RobotContainer.elevatorL3)))
        self._operator_joystick.y().onTrue(self.elevator.runOnce(lambda: self.elevator.setNextTargetRotation(RobotContainer.elevatorL4)))
        self._operator_joystick.rightTrigger().whileTrue(self.elevator.move_up_gradually())
        self._operator_joystick.leftTrigger().whileTrue(self.elevator.move_down_gradually())
        self._operator_joystick.povDown().whileTrue(self.elevator.move_to_zero())

        # driver buttons
        self._joystick.leftTrigger().whileTrue(self.cannon.loadCoral().deadlineFor(self.hopper.intake()))
        self._joystick.back().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.zeroHeading()))
        self._joystick.rightTrigger().whileTrue(self.elevator.move_to_position_execute())
        self._joystick.leftBumper().whileTrue(self.cannon.createPlaceCoralCommand(self.isPlaceCoralL1))
        self._joystick.rightBumper().whileTrue(self.hopper.agitate())
        self._joystick.povDown().whileTrue(self.elevator.move_to_zero())

        self._joystick.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._reefAlign.with_velocity_x(
                        negative_value
                        * self._joystick.getLeftY()
                        * self._max_speed
                        / self.reefAlignSlownessFactor
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        negative_value
                        * self._joystick.getLeftX()
                        * self._max_speed
                        / self.reefAlignSlownessFactor
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        negative_value
                        * self._joystick.getRightX()
                        * self._max_angular_rate
                        / self.reefAlignSlownessFactor
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Tester Joystick buttons
        self._test_joystick.leftBumper().onTrue(SelectCommand(self.alignLeftCommands, lambda: self.pose.neartestFace(self.drivetrain.getPose().translation(), False)))
        self._test_joystick.rightBumper().onTrue(SelectCommand(self.alignRightCommands, lambda: self.pose.neartestFace(self.drivetrain.getPose().translation(), False)))
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
        # self._joystick.leftBumper().onTrue(
        #     self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        # )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def configureOperatorBindings(self) -> None:
        self.operator1.axisLessThan(1, -0.99) # Go to L4
        self.operator1.axisGreaterThan(0, 0.99) # Go to L3
        self.operator1.axisLessThan(0, -0.99) # Go to L2
        self.operator1.axisGreaterThan(1, 0.99) # Go to L1

        self.operator2.axisGreaterThan(0, 0.99) # Reef side A
        self.operator2.axisLessThan(0, -0.99) # Reef side B
        self.operator2.axisLessThan(1, -0.99) # Reef side C
        self.operator2.axisGreaterThan(1, 0.99) # Reef side D
        self.operator2.button(0) # Reef side E
        self.operator2.button(1) # Reef side F
        self.operator2.button(2) # Reef side G
        self.operator2.button(3) # Reef side H
        self.operator2.button(4) # Reef side I
        self.operator2.button(5) # Reef side J
        self.operator2.button(6) # Reef side K
        self.operator2.button(7) # Reef side L

    def getAutonomousCommand(self) -> Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # return commands2.cmd.print_("No autonomous command configured")
        auto = self.autoChooser.getSelected()

        return auto

    def addNoPathAutos(self):
        self.autoChooser.addOption("Score One L1 - Middle", self.scoreMiddleL1Auto())

    def scoreMiddleL1Auto(self) -> Command:
        return (
            self.elevator.runOnce(
                lambda: self.elevator.setNextTargetRotation(RobotContainer.elevatorL1)
            )
            .andThen(
                self.drivetrain.apply_request(
                    lambda: (swerve.requests.RobotCentric().with_velocity_x(1.0))
                ).withTimeout(2.0)
            )
            .andThen(self.drivetrain.stopCommand())
            .andThen(
                self.elevator.move_to_position_execute().raceWith(
                    commands2.WaitUntilCommand(
                        lambda: self.elevator.acceptablyOnTargetForL1()
                    ).andThen(self.cannon.createPlaceCoralCommand(self.isPlaceCoralL1).withTimeout(1.0))
                )
            )
        )