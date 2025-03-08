from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController, reportError
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Translation2d, Pose3d
from wpimath.units import degreesToRadians
from wpimath.kinematics import ChassisSpeeds
from pathplannerlib.auto import AutoBuilder, PathConstraints, PathPlannerPath
from auto.auto_constants import AutoConstants
from phoenix6.swerve import requests
from wpilib import SmartDashboard
# from subsystems.vision.vision import VisionSubsystem as Vision
# from subsystems.vision.vision_constants import Vision_Constants as vision_constants
# from generated.tuner_constants import TunerConstants


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        self.autoBuilderConfigure()
        # self.vision = Vision()
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        self.autoBuilderConfigure()
        # self.vision = Vision()
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        # vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        self.autoBuilderConfigure()
        # self.vision = Vision()
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )
        self.autoBuilderConfigure()
        # self.vision = Vision()
        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def stopCommand(self) -> Command:
        """Returns a command that will stop the drive train."""
        return self.runOnce(
            lambda: self.set_control(
                swerve.requests.RobotCentric()
            )
        )

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        SmartDashboard.putNumber("Swerve/Forward Direction", self.get_operator_forward_direction().degrees())
        # self.update_Odom()

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def getPigeonRotation2d(self)->Rotation2d:
        """
        Grabs the Rotation 2d from the pigeon
        """
        return Rotation2d(degreesToRadians(self.pigeon2.get_yaw().value))
    
    def autoBuilderConfigure(self):
            """
            Configures the AutoBuilder for the robot
            """
            AutoBuilder.configure(
                lambda: self.get_state().pose,
                self.reset_pose,
               lambda:  self.get_state().speeds,
                self.swerve_output,
                AutoConstants.holonomicPathConfig,
                AutoConstants.robot_config,
                self.flip,
                self
            )


            
    
    def flip(self):
        """
        Checks to see wether the robot should flip the path based on its current alliance
        """
        return (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed
    
    def swerve_output(self, speeds, feedforwards):
        return self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            )
    
    def getHeading(self):
        """
        Gets the heading of the robot
        """
        return self.get_state().pose.rotation().degrees()
    
    def setOperatorPerspectiveFalse(self):
        """
        Sets the operator perspective to false
        """
        self._has_applied_operator_perspective = False

    def zeroHeading(self):
        """
        Zeros the heading of the robot
        """
        return self.seed_field_centric()
    
    
    def getPose(self):
        """
        Gets the pose of the robot
        """
        return self.get_state().pose
    
    def drive(self, translation: Translation2d, rotation: float, isOpenLoop: bool):
        """
        Drives the robot
        """
        desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.X(), translation.Y(), rotation, self.get_state().pose.rotation())

        self.driveRobotRelative(desiredSpeeds, isOpenLoop)

    def driveRobotRelative(self, desiredSpeeds: ChassisSpeeds, isOpenLoop: bool):
        """
        Drives the robot relative to the robot
        """
        ChassisSpeeds.discretize(desiredSpeeds, 0.02)

        self.set_control(requests.RobotCentric().with_velocity_x(desiredSpeeds.vx).with_velocity_y(desiredSpeeds.vy).with_rotational_rate(desiredSpeeds.omega).with_drive_request_type(requests.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE if isOpenLoop else requests.SwerveModule.DriveRequestType.VELOCITY))


    def driveToPoseThenFollowPath(self, path: PathPlannerPath)-> Command:

        """
        Drives to a starting pose of a pre-generated path, then follows the pre-generated path
        """
        constraints = PathConstraints(3.0, 4, degreesToRadians(540), degreesToRadians(720))

        redPath = path


        bluePath = path.flipPath()

        if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed:
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(redPath, constraints)
        else:
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(bluePath, constraints)

        return pathfindingCommand 
    # def update_Odom(self):
    #     """
    #     Updates the odometry with the vision
    #     """
    #     frontleftEstimatedPose = self.vision.getEstimatedGlobalPoseFrontLeft()
    #     frontrightEstimatedPose = self.vision.getEstimatedGlobalPoseFrontRight()
    #     backleftEstimatedPose = self.vision.getEstimatedGlobalPoseBackLeft()
    #     backrightEstimatedPose = self.vision.getEstimatedGlobalPoseBackRight()


    #     if frontleftEstimatedPose is not None:

    #         tags = frontleftEstimatedPose.targetsUsed
    #         tagPoses: list[Pose3d] = []

    #         distance = 0.0
    #         stddevs = (0.0, 0.0, 0.0)

    #         if len(tags)>0:
    #             for tag in tags:
    #                 id = tag.getFiducialId()
    #                 pose = self.vision.aprilTagFieldLayout.getTagPose(id)
    #                 if pose is not None:
    #                     tagPoses.append(pose)
                
    #             if len(tagPoses)>0:
    #                 for tagPose in tagPoses:
    #                     distance += tagPose.translation().distance(frontleftEstimatedPose.estimatedPose.translation())

    #                 distance = distance/len(tagPoses)
                
    #             singleTagXY = 0.03
    #             multiTagXY  = 0.05
    #             tagRot = math.radians(40)

    #             xyStdDev = (singleTagXY if len(tagPoses) == 1 else multiTagXY) * distance**2
    #             stddevs = (xyStdDev, xyStdDev, tagRot)

    #             self.add_vision_measurement(Pose2d(x=frontleftEstimatedPose.estimatedPose.toPose2d().X(), y=frontleftEstimatedPose.estimatedPose.toPose2d().Y(), rotation=self.getHeading()), utils.get_current_time_seconds, stddevs)
            
    #     if frontrightEstimatedPose is not None:
    #         tags = frontrightEstimatedPose.targetsUsed
    #         tagPoses: list[Pose3d] = []

    #         distance2 = 0.0
    #         stddevs2 = (0.0, 0.0, 0.0)

    #         if len(tags)>0:
    #             for tag in tags:
    #                 id = tag.getFiducialId()
    #                 pose = self.vision.aprilTagFieldLayout.getTagPose(id)
    #                 if pose is not None:
    #                     tagPoses.append(pose)
                
    #             if len(tagPoses)>0:
    #                 for tagPose in tagPoses:
    #                     distance2 += tagPose.translation().distance(frontrightEstimatedPose.estimatedPose.translation())

    #                 distance2 = distance2/len(tagPoses)
                
    #             singleTagXY = 0.03
    #             multiTagXY  = 0.05
    #             tagRot = math.radians(40)

    #             xyStdDev2 = (singleTagXY if len(tagPoses) == 1 else multiTagXY) * distance**2
    #             stddevs2 = (xyStdDev2, xyStdDev2, tagRot)

    #             self.add_vision_measurement(Pose2d(x=frontrightEstimatedPose.estimatedPose.toPose2d().X(), y=frontrightEstimatedPose.estimatedPose.toPose2d().Y(), rotation=self.getHeading()), utils.get_current_time_seconds, stddevs2)
        
    #     if backleftEstimatedPose is not None:
    #         tags = backleftEstimatedPose.targetsUsed
    #         tagPoses: list[Pose3d] = []

    #         distance3 = 0.0
    #         stddevs3 = (0.0, 0.0, 0.0)

    #         if len(tags)>0:
    #             for tag in tags:
    #                 id = tag.getFiducialId()
    #                 pose = self.vision.aprilTagFieldLayout.getTagPose(id)
    #                 if pose is not None:
    #                     tagPoses.append(pose)
                
    #             if len(tagPoses)>0:
    #                 for tagPose in tagPoses:
    #                     distance3 += tagPose.translation().distance(backleftEstimatedPose.estimatedPose.translation())

    #                 distance3 = distance3/len(tagPoses)
                
    #             singleTagXY = 0.03
    #             multiTagXY  = 0.05
    #             tagRot = math.radians(40)

    #             xyStdDev3 = (singleTagXY if len(tagPoses) == 1 else multiTagXY) * distance**2
    #             stddevs3 = (xyStdDev3, xyStdDev3, tagRot)

    #             self.add_vision_measurement(Pose2d(x=backleftEstimatedPose.estimatedPose.toPose2d().X(), y=backleftEstimatedPose.estimatedPose.toPose2d().Y(), rotation=self.getHeading()), utils.get_current_time_seconds, stddevs3)
        
    #     if backrightEstimatedPose is not None:
    #         tags = backrightEstimatedPose.targetsUsed
    #         tagPoses: list[Pose3d] = []

    #         distance4 = 0.0
    #         stddevs4 = (0.0, 0.0, 0.0)

    #         if len(tags)>0:
    #             for tag in tags:
    #                 id = tag.getFiducialId()
    #                 pose = self.vision.aprilTagFieldLayout.getTagPose(id)
    #                 if pose is not None:
    #                     tagPoses.append(pose)
                
    #             if len(tagPoses)>0:
    #                 for tagPose in tagPoses:
    #                     distance4 += tagPose.translation().distance(backrightEstimatedPose.estimatedPose.translation())

    #                 distance4 = distance4/len(tagPoses)
                
    #             singleTagXY = 0.03
    #             multiTagXY  = 0.05
    #             tagRot = math.radians(40)

    #             xyStdDev4 = (singleTagXY if len(tagPoses) == 1 else multiTagXY) * distance**2
    #             stddevs4 = (xyStdDev4, xyStdDev4, tagRot)

    #             self.add_vision_measurement(Pose2d(x=backrightEstimatedPose.estimatedPose.toPose2d().X(), y=backrightEstimatedPose.estimatedPose.toPose2d().Y(), rotation=self.getHeading()), utils.get_current_time_seconds, stddevs4)
        