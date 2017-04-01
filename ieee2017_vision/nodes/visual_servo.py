#!/usr/bin/env python

'''
Visual Servo: Uses information from the shooter camera to line the shooter up
with the target for Stage 4 - Fire the Proton Torpedo.
'''


from __future__ import division

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from ieee2017_msgs.msg import ShooterControl


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2017, UF IEEE"
__license__ = "MIT"


class VisualServo(object):

	def __init__(self):
		self.__image = None
		self.__center = None
		self.__darts_in_clip = [ShooterControl.dart_1, ShooterControl.dart_2, ShooterControl.dart_3]

		self.__shooter_publisher = rospy.Publisher("/shooter", ShooterControl, queue_size=1)

		# Leverages CVBridge to easily generate Image messages
		self.__bridge = CvBridge()
 		self.__camera_subscriber = rospy.Subscriber("/shooter_camera", Image, self.__cache_image)

		# Wait for an image to be recieved from the camera
		while (self.__image is None) and (not rospy.is_shutdown()):
			rospy.logwarn("Waiting for images from the shooter camera")
			rospy.sleep(1)

		rospy.sleep(30)

		# Wait for the center of the target to be detected
		while (self.__center is None):
			self.__average_centers(20)

		# Fire all three darts
		rospy.sleep(6)
		self.__center_shooter()
		#self.__shooter_aim()
		self.__shooter_fire(3)

	def __cache_image(self, msg):
		'''
		Process the data received in the Image message and store
		the resulting image.
		'''
		try:
			self.__image = self.__bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print e

	def __find_target_center(self):
		'''
		Identifies the center point of the target by filtering the
		image and detecting a square.
		'''
		working_image = cv2.cvtColor(self.__image, cv2.COLOR_BGR2GRAY)

		# Shrink the image to reduce processing time
		resize_width=320
		origin_size = (working_image.shape[1], working_image.shape[0])
		image_aspect_ratio = float(origin_size[0]) / origin_size[1]
		working_size = (resize_width, int(resize_width / image_aspect_ratio))
		scaling_ratio = float(origin_size[0]) / working_size[0]
		working_image = cv2.resize(working_image, working_size, interpolation = cv2.INTER_AREA)

		# Apply a bilateral filter to the image to reduce noise
		working_image = cv2.bilateralFilter(working_image, 20, 75, 75)

		# Use Otsu's method to threshold the image
		working_image = cv2.threshold(working_image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

		largest_contour = None
		largest_contour_area = 0

		# Iterate over contours in the image
		contours = cv2.findContours(working_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
		for contour in contours:

			# Generate an approximation of the contour with linear faces
			perimeter = cv2.arcLength(contour, True)
			approximation = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

			# Filter out any shape that does not have four faces
			if len(approximation) == 4:

				# Use the bounding box to compute the shape's aspect ratio
				x, y, width, height = cv2.boundingRect(approximation)
				contour_aspect_ratio = width / float(height)

				if ((width * height) > largest_contour_area):
					
					# The aspect ratio of a square should be close to 1
					if (contour_aspect_ratio >= 0.95 and contour_aspect_ratio <= 1.05):
						largest_contour = contour

		if (largest_contour is not None):
			moments = cv2.moments(largest_contour)
			center_x = int((moments["m10"] / moments["m00"]) * scaling_ratio)
			center_y = int((moments["m01"] / moments["m00"]) * scaling_ratio)
			self.__center=(center_x, center_y)

	def __average_centers(self, averaging_number):
		centers = [0, 0]

		for index in range(averaging_number):
			self.__find_target_center()
			if (self.__center is not None):
				centers[0] += self.__center[0]
				centers[1] += self.__center[1]
			else:
				averaging_number -= 1

		if (averaging_number > 0):
			self.__center = (centers[0] / averaging_number, centers[1] / averaging_number)

	def __center_shooter(self):
		'''
		Used to bring the shooter back to it's center position with
		sane defauts
		'''
		self.__shooter_command = ShooterCommand
		self.__shooter_command.pan_angle = ShooterControl.pan_center
		self.__shooter_command.tilt_angle = ShooterControl.tilt_center
		self.__shooter_command.dart = ShooterControl.dart_1
		self.__shooter_command.flywheels = False
		self.__shooter_command.fire = False
		self.__shooter_publisher.publish(self.__shooter_command)

	def __shooter_aim(self):
		'''
		Aims the shooter based on the detected coordinates of the
		target's center point.
		'''
		self.__shooter_command.pan = self.__center(0)
		self.__shooter_command.tilt = self.__center(1)
		self.__shooter_publisher.publish(self.__shooter_command)
		rospy.sleep(2)

	def __shooter_fire(self, darts_to_fire):
		'''
		Aims and fires the shooter based on the detected center of the
		target.
		'''
		for firing_count in darts_to_fire:

			# Will only fire if there are darts left in the clip
			if (len(self.__darts_in_clip) > 0):

				# Spin up the flywheels
				self.__shooter_command.flywheels = True
				self.__shooter_publisher.publish(self.__shooter_command)

				# Increment the clip
				dart_to_fire = self.__darts_in_clip.pop()
				self.__shooter_command.dart = dart_to_fire
				self.__shooter_publisher.publish(self.__shooter_command)
				rospy.sleep(3)

				# Fire the shooter
				self.__shooter_command.fire = True
				self.__shooter_publisher.publish(self.__shooter_command)
				rospy.sleep(1)
				self.__shooter_command.fire = False
				self.__shooter_publisher.publish(self.__shooter_command)

				# Spin down the flywheels
				self.__shooter_command.flywheels = False
				self.__shooter_publisher.publish(self.__shooter_command)


if __name__ == "__main__":
	rospy.init_node("visual_servo")
	visual_servo = VisualServo()
	rospy.spin()
