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
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self._operator_joystick = commands2.button.CommandXboxController(1)

        self.operator1 = commands2.button.CommandGenericHID(1)
        self.operator2 = commands2.button.CommandGenericHID(2)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.elevator = Elevator()
        self.hopper = Hopper()
        self.cannon = Cannon()

        # Configure the button bindings

        self.configureButtonBindings()
        self.configureOperatorBindings()

        self.autoChooser = AutoBuilder.buildAutoChooser("None")
        SmartDashboard.putData("AutoChooser",self.autoChooser)


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
        self._joystick.y().whileTrue(self.elevator.move_to_position(1.1)) #l1
        self._joystick.a().whileTrue(self.elevator.move_to_position(2.1)) #l2
        self._joystick.b().whileTrue(self.elevator.move_to_position(3.5, 1)) #l3
        self._joystick.x().whileTrue(self.elevator.move_to_position(0)) #zero
        
        self._joystick.back().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.zeroHeading()))
        # self._joystick.povDown().onTrue(self.elevator.zero_rotations())
        # self._joystick.leftTrigger().whileTrue(self.cannon.loadCoral().deadlineFor(self.hopper.intake()))
        # self._joystick.povUp().whileTrue(self.hopper.agitate())

        self._joystick.rightTrigger().whileTrue(self.elevator.move_up_gradually())
        self._joystick.leftTrigger().whileTrue(self.elevator.move_down_gradually())
        self._joystick.rightBumper().whileTrue(self.cannon.loadCoral().deadlineFor(self.hopper.intake()))
        # self._joystick.a().whileTrue(self.elevator.move_to_current_position())
        # self._joystick.povRight().onTrue(self.elevator.zero_rotations())

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
