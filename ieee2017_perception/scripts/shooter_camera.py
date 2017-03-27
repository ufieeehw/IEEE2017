#!/usr/bin/python2

'''
Shooter Camera: This program acts as a simple frame buffer and broadcaster
for the shooter camera.
'''


from __future__ import division

import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, UF IEEE"
__license__ = "MIT"


class Camera(object):

	def __init__(self):
		self.__index = rospy.get_param("~index", 0)
		self.__width = rospy.get_param("~width", 1920)
		self.__height = rospy.get_param("~height", 1080)
		self.__fps = rospy.get_param("~fps", 60)

		# Leverages CVBridge to easily generate Image messages
		self.__bridge = CvBridge()
		self.__frame_publisher = rospy.Publisher("/shooter_camera", Image, queue_size=1)

		self.__activate()

		# Reads new frames at the desired framerate
		rospy.Timer(rospy.Duration(1.0 / self.__fps), self.__get_frame, oneshot=False)

	def __activate(self):
		'''
		Opens a stream from the capture device and sets the state to
		active. Running activate while the camera is active will not
		recreate the stream thread.
		'''
		self.__stream = cv2.VideoCapture(self.__index)
		self.__stream.set(3, self.__width)
		self.__stream.set(4, self.__height)

		# Waits until the camera starts returning frames
		while (self.__stream.read()[1] == None and not rospy.is_shutdown()):
			rospy.logwarn("The camera at index {} is not responding".format(self.__index))
			rospy.sleep(1)

	def __get_frame(self, event):
		'''
		Returns the frame that is currently stored in the cache if the
		camerais active.
		'''
		image=self.__stream.read()[1]
		self.__frame_publisher.publish(self.__bridge.cv2_to_imgmsg(image, "bgr8"))


if __name__ == "__main__":
	rospy.init_node("shooter_camera")
	shooter_camera = Camera()
	rospy.spin()
