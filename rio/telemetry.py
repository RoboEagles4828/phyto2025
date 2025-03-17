from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, SupplyCurrentLimitConfiguration


class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        SignalLogger.start()

        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot swerve drive state
        self._drive_state_table = self._inst.getTable("DriveState")
        self._drive_pose = self._drive_state_table.getStructTopic("Pose", Pose2d).publish()
        self._drive_speeds = self._drive_state_table.getStructTopic("Speeds", ChassisSpeeds).publish()
        self._drive_module_states = self._drive_state_table.getStructArrayTopic("ModuleStates", SwerveModuleState).publish()
        self._drive_module_targets = self._drive_state_table.getStructArrayTopic("ModuleTargets", SwerveModuleState).publish()
        self._drive_module_positions = self._drive_state_table.getStructArrayTopic("ModulePositions", SwerveModulePosition).publish()
        self._drive_timestamp = self._drive_state_table.getDoubleTopic("Timestamp").publish()
        self._drive_odometry_frequency = self._drive_state_table.getDoubleTopic("OdometryFrequency").publish()

        # Robot pose for field positioning
        self._table = self._inst.getTable("Pose")
        self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .appendLigament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        ]

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._drive_speeds.set(state.speeds)
        self._drive_module_states.set(state.module_states)
        self._drive_module_targets.set(state.module_targets)
        self._drive_module_positions.set(state.module_positions)
        self._drive_timestamp.set(state.timestamp)
        self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # Also write to log file
        pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]
        module_states_array = []
        module_targets_array = []
        for i in range(4):
            module_states_array.append(state.module_states[i].angle.radians())
            module_states_array.append(state.module_states[i].speed)
            module_targets_array.append(state.module_targets[i].angle.radians())
            module_targets_array.append(state.module_targets[i].speed)

        SignalLogger.write_double_array("DriveState/Pose", pose_array)
        SignalLogger.write_double_array("DriveState/ModuleStates", module_states_array)
        SignalLogger.write_double_array(
            "DriveState/ModuleTargets", module_targets_array
        )
        SignalLogger.write_double(
            "DriveState/OdometryPeriod", state.odometry_period, "seconds"
        )

        # Telemeterize the pose to a Field2d
        self._field_type_pub.set("Field2d")
        self._field_pub.set(pose_array)

        # Telemeterize the module states to a Mechanism2d
        for i, module_state in enumerate(state.module_states):
            self._module_speeds[i].setAngle(module_state.angle.degrees())
            self._module_directions[i].setAngle(module_state.angle.degrees())
            self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))

            SmartDashboard.putData(f"Module {i}", self._module_mechanisms[i])
            
            print("initial targetPose" + str(self.targetPose))
            SmartDashboard.putNumberArray("PID_Swerve/Target Pose", self.targetPose)
            print("target Pose" + str(self.targetPose))
            SmartDashboard.putNumber("PID_Swerve/X Position", Units.inchesToMeters(position.X()))
            SmartDashboard.putNumber("PID_Swerve/X Feed Forward", xFeedForward)
            SmartDashboard.putNumber("PID_Swerve/X Value", xVal)
            SmartDashboard.putNumber("PID_Swerve/X Correction", xCorrection)
            SmartDashboard.putNumber("PID_Swerve/Y Position", Units.inchesToMeters(position.Y()))
            SmartDashboard.putNumber("PID_Swerve/Y Feed Forward", yFeedForward)
            SmartDashboard.putNumber("PID_Swerve/Y Value", yVal)
            SmartDashboard.putNumber("PID_Swerve/Y Correction", yCorrection)

            SmartDashboard.putBoolean("Cannon/loaded", self.loaded)
            SmartDashboard.putNumber("Cannon/leftStatorCurrent", self.leftMotor.getStatorCurrent())
            SmartDashboard.putNumber("Cannon/rightStatorCurrent", self.rightMotor.getStatorCurrent())
            SmartDashboard.putNumber("Cannon/leftSupplyCurrent", self.leftMotor.getSupplyCurrent())
            SmartDashboard.putNumber("Cannon/rightSupplyCurrent", self.rightMotor.getSupplyCurrent())

            SmartDashboard.putNumber("Elevator/Position", self.getPosition())
            SmartDashboard.putNumber("Elevator/Velocity", self.getVelocity())
            SmartDashboard.putNumber("Elevator/Desired Position", self.desiredPosition)
            SmartDashboard.putNumber("Elevator/Voltage", self.rightMotorLeader.get_motor_voltage().value)
            SmartDashboard.putBoolean("Elevator/Bottom Limit", self.bottomLimitSwitch.get())
            SmartDashboard.putBoolean("Elevator/Top Limit", self.topLimitSwitch.get())
            SmartDashboard.putNumber("Elevator/Next Target", self.nextTargetPosition)
            
            SmartDashboard.putNumber("Hopper/Motor Supply Current", self.hopperMotor.getSupplyCurrent())
            SmartDashboard.putNumber("Hopper/Motor Stator Current", self.hopperMotor.getStatorCurrent())

            print("Running Command")
            print("reefBearing"+str(reefBearing))
            print("bearingAngle"+str(bearingAngle))
            print("GH")
            print("EF")
            print("CD")
            print("AB")
            print("KL")
            print("IJ")
            SmartDashboard.putNumber("Pose/Heading", self.swerve.get_state().raw_heading.degrees())
            SmartDashboard.putNumber("Pose/Operator Forward", self.swerve.get_operator_forward_direction().degrees())
            SmartDashboard.putNumber("Pose/bearingAngle", self.reefBearing(self.swerve.get_state().pose.translation(), True).degrees())

            SmartDashboard.putData("vision/Field", self.field)
            SmartDashboard.putBoolean("Vision/ New Result", newResult)
            SmartDashboard.putString("Vision/Result", str(result))
            SmartDashboard.putBoolean("Vision/ Have Target", haveTarget)
            SmartDashboard.putBoolean("Vision/ Have Result", result != None)