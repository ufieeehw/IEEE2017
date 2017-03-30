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


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, UF IEEE"
__license__ = "MIT"


class CommunicationManager(object):

	def __init__(self):
		self.__port = rospy.get_param("~serial_port", "/dev/ttyACM0")
		self.__baud = rospy.get_param("~baud_rate", 9600)
		self.__size = rospy.get_param("~command_size", 6)

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


if __name__ == "__main__":
	rospy.init_node("communication_manager")
	com = CommunicationManager()
	rospy.spin()
