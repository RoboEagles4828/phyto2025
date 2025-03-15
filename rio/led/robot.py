from wpilib import TimedRobot
from commands2 import Command
from commands2 import CommandScheduler
from wpimath.geometry import Rotation2d
from wpilib import XboxController
from wpilib.shuffleboard import ShuffleboardTab
from wpilib import DriverStation
from led2 import LED


class Robot(TimedRobot):
	m_autonomousCommand: Command = None

	led : LED
	joystick : XboxController
	# joystick here too

	def robotInit(self):
		""" Instantiate our `RobotContainer`.  
		This will perform all button bindings and put the auton chooser on the dashboard
		"""
		# wpilib.CameraServer.launch()
		self.led = LED()

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
			print("Red is Redding")
			self.led.green()

		# elif self.joystick.getXButtonPressed():
		# 	print("Yellow is Yellowing")
		# 	self.led.yellow()

		# elif self.joystick.getBButtonPressed():
		# 	print("Green is Greening")
		# 	self.led.green()

		# elif self.joystick.getYButtonPressed():
		# 	print("Teal is Tealing")
		# 	self.led.teal()

	CommandScheduler.getInstance().run()

	def autonomousInit(self):
		pass

	def teleopInit(self):
		pass

	def testInit(self):
		CommandScheduler.getInstance().cancelAll()