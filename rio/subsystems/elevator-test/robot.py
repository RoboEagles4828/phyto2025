from wpilib import TimedRobot
from commands2 import Command
from commands2 import CommandScheduler
from wpimath.geometry import Rotation2d
from wpilib import XboxController
from wpilib.shuffleboard import ShuffleboardTab
from wpilib import DriverStation
from Elevator import Elevator


class Robot(TimedRobot):
	m_autonomousCommand: Command = None

	elevator : Elevator
	joystick : XboxController
	# joystick here too

	def robotInit(self):
		""" Instantiate our `RobotContainer`.  
		This will perform all button bindings and put the auton chooser on the dashboard
		"""
		# wpilib.CameraServer.launch()
		self.elevator = Elevator()

		# Create a joystick XboxController
		self.joystick = XboxController(0)
	

		CommandScheduler.getInstance().setPeriod(0.02)

	def robotPeriodic(self):
		""" Runs the `CommandScheduler`.  
		This is responsible for polling buttons, adding newly-scheduled
		commands, running already-scheduled commands, removing finished or interrupted commands,
		and running subsystem `periodic()` methods.  This must be called from the robot's periodic
		block in order for anything in the Command-based framework to work.
		"""
		# listen to joystick 

		if self.joystick.getAButtonPressed():
			self.elevator.move_to_l1()

		elif self.joystick.getXButtonPressed():
			self.elevator.move_to_l2()

		elif self.joystick.getBButtonPressed():
			self.elevator.move_to_l3()

		elif self.joystick.getYButtonPressed():
			self.elevator.move_to_l4()

		elif self.joystick.getLeftBumperButton():
			self.elevator.move_down_gradually()

		elif self.joystick.getLeftTriggerAxis() >= 0.5:
			self.elevator.move_to_origin()

		elif self.joystick.getRightBumperButton() == 1:
			self.elevator.move_up_gradually()

		elif self.joystick.getLeftBumperButtonReleased():
			self.elevator.stop()

		elif self.joystick.getRightBumperButtonReleased():
			self.elevator.stop()



	CommandScheduler.getInstance().run()

	def autonomousInit(self):
		pass

	def teleopInit(self):
		pass

	def testInit(self):
		CommandScheduler.getInstance().cancelAll()