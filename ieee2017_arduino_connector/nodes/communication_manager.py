#!/usr/bin/env python

'''
Communication Manager: This program handles communications with th Arduino
microcontrollers that operate most of the stage end effectors and motors. It
also enables the state machine running on the microcontrollers to trigger
functions on the main computer.
'''


from __future__ import division

from serial import Serial

import rospy
from ieee2017_msgs.msg import ShooterControl

__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2017, UF IEEE"
__license__ = "MIT"


class CommunicationManager(object):

	def __init__(self):
		self.__port = rospy.get_param("~serial_port", "/dev/ttyACM0")
		self.__baud = rospy.get_param("~baud_rate", 9600)
		self.__size = rospy.get_param("~command_size", 6)

		# Set up the interface to the shooter hardware device
		self.__initialize_shooter()
		self.shooter_subscriber = rospy.Subscriber("/shooter", ShooterControl, self.__update_shooter, queue_size=1)

		is_waiting = True
		while (is_waiting and not rospy.is_shutdown()):
			try:
				self.__serial = Serial(self.__port, self.__baud, timeout=1)
				execute_incoming = False
			except:
				rospy.logwarn("Unable to connect to serial device '{}'".format(self.__port))
			rospy.sleep(1)

		self.__incoming_command = ""
		self.__command_mappings = {
		}

		# Read in serial bytes at half the rate they should be coming in
		rospy.Timer(rospy.Duration(1.0 / (self.__baud / 8 / 2)), self.__execute_incoming, oneshot=False)


	def __initialize_shooter(self):
		'''
		Initializes the shooter communications system. Sets up the
		hardware device with some sane defaults and creates a few
		dictionaries to help with updates.
		'''
		self.__shooter_settings = ["pan_angle", "tilt_angle", "dart", "flywheels", "fire"]
		pretexts = ["PAN", "TLT", "POS", "SPN", "SHT"]

		# Create a dictionary of pretexts referencable by their respective settings
		for index, setting in enumerate(self.__shooter_settings)
			self.__shooter_pretexts[setting] = pretexts[index]

		# Creates a shooter configuration with some sane defaults
		self.__shooter = {	"pan_angle":	ShooterControl.pan_center
					"tilt_angle":	ShooterControl.tilt_center
					"dart":		ShooterControl.dart_1
					"flywheels":	False
					"fire":		False

		# Sends the default configuration to the shooter
		for setting in self.__shooter_settings[:4]:
			if (setting == "flywheels"):
				ser.write(b"{}".format(self.__shooter_pretexts[setting] +
					"1" if self.__shooter[setting] else "0")
			else:
				ser.write(b"{}".format(self.__shooter_pretexts[setting] +
					self.__shooter[setting].zfill(3)))

	def __execute_incoming(self):
		'''
		Reads in commands and executes the functions that were mapped to them
		during initialization.
		'''

		self.__incoming_command += ser.read()

		# Once a full command has been recieved, execute it if it exists
		if (len(self.__incoming_command) >= 6):
			if (self.__incoming_command in self.__command_mappings):
				self.__command_mappings[self.__incoming_command]()
			self.__incoming_command = ""

	def __update_shooter(self, msg):
		'''
		Maps the state of the shooter object to the serial commands
		needed to actually control it's hardware. Commands will only be
		sent for values that have changed.
		'''
		for setting in self.__shooter_settings:
			new_value = getattr[msg, setting]
			if (new_value != self.__shooter[setting]):
				self.__shooter[setting] = new_value
				if (setting == "flywheels"):
					ser.write(b"{}".format(self.__shooter_pretexts[setting] +
						"1" if self.__shooter[setting] else "0")
				elif (setting == "fire") and (self.__shooter[setting]):
					ser.write(b"{}".format(self.__shooter_pretexts[setting])
				else:
					ser.write(b"{}".format(self.__shooter_pretexts[setting] +
						self.__shooter[setting].zfill(3)))


if __name__ == "__main__":
	rospy.init_node("communication_manager")
	com = CommunicationManager()
	rospy.spin()
