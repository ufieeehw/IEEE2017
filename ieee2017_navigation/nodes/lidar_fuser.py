#!/usr/bin/env python

'''
LiDAR Fuser: This node takes in the data from each of the LiDARs and combines
it into one large point cloud centered on the vehicle.
'''


from __future__ import division

import math
import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan


__author__ = "Matthew Langford"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2017, UF IEEE"
__license__ = "MIT"


class LidarFuser():

	def __init__(self):
		self.__fused_publisher = rospy.Publisher("/navigation/lidar/lidar_fused", LaserScan, queue_size=3)

		# Define the fused scan based on which LiDARs were enabled in this parameter
		is_enabled = rospy.get_param("~is_enabled")

		# Set a dictionary for stroing information for each LiDAR
		self.__lidars = {}
		for lidar in is_enabled.keys():
			self.__lidars[lidar] = {"enabled": 	is_enabled[lidar],
						"fov":		80,
						"scan":		None
						}

		# Start a TF listener and make sure it gets data before preceding
		# The name of the frame for each LiDAR is the same as it's name
		self.__tf_listener = tf.TransformListener()
		print "Waiting for TF..."
		for frame in self.__lidars.keys():
			self.__tf_listener.waitForTransform("base_link", frame, rospy.Time(0), rospy.Duration(5.0))
		print "TF Found! Starting Fuser."

		# Subscribe to each of the LiDAR topics
		for frame in self.__lidars.keys():
			rospy.Subscriber("/navigation/lidar/{}".format(frame), LaserScan, self.__update_scan, queue_size=3)

		# Define a new LaserScan with constant values for the fused scan
		self.__fused_scan = LaserScan()
		self.__fused_scan.header.frame_id = "lidar_fused"
		self.__fused_scan.angle_increment = .007 # radians per index
		self.__fused_scan.angle_min = -3.14159274101 # radians
		self.__fused_scan.angle_max = 3.14159274101 # radians
		self.__fused_scan.range_max = 5.0 # meters
		self.__fused_scan.range_min = 0.0002 # meters

		# Wait until scans have been received from all enabled LiDARs
		are_scans_received = False
		while (not are_scans_received) and (not rospy.is_shutdown()):
			rospy.sleep(1)
			are_scans_received = True
			for lidar in self.__lidars.keys():
				if (self.__lidars[lidar]["enabled"]) and (self.__lidars[lidar]["scan"] == None):
					are_scans_received = False
					rospy.logwarn("No messages recieved from {}...".format(lidar))

		# Publish the fused scans at the rate specified in the ROS parameter
		publishing_rate = rospy.get_param("~publishing_rate", 10)
		rospy.Timer(rospy.Duration(1.0 / publishing_rate), self.__fuse_scans, oneshot=False)

	def __update_scan(self, msg):
		'''
		Cache scan messages from the LiDARs
		'''
		self.__lidars[msg.header.frame_id]["scan"] = msg

	def __fuse_scans(self, event):
		'''
		Take all four scans, trim them to a new FOV, convert them to
		cartesian, apply transformations, pub pointcloud, tranform back
		to cart, and pub LaserScan.
		'''
		fused_cartesian = np.array([-1, -1])

		# Format all of the active frames, scans, and FOVs as lists
		frames, scans, fovs = [], [], []
		for lidar in self.__lidars.keys():
			if (self.__lidars[lidar]["enabled"] is True):
				frames.append(lidar)
				scans.append(self.__lidars[lidar]["scan"])
				fovs.append(self.__lidars[lidar]["fov"])

		for i, s in enumerate(scans):

			# Define parameters based on scan data
			angle_increment = s.angle_increment
			ranges = np.array(s.ranges)
			ranges_size = ranges.size

			# Trim to FOV
			new_fov = math.radians(fovs[i])
			angle_min = -(new_fov/2.0)
			angle_max = (new_fov/2.0)
			ranges = ranges[int(ranges_size/2 + angle_min/angle_increment): int(ranges_size/2 + angle_max/angle_increment)]
			ranges_size = ranges.size

			# Get TF data about the LIDAR position
			trans, q_rot = self.__tf_listener.lookupTransform('base_link', frames[i], rospy.Time(0))
			rot = tf.transformations.euler_from_quaternion(q_rot)

			# Generate list that maps indices of the ranges to radian angle measures
			thetas = np.linspace(angle_min, angle_max, ranges.size, endpoint=True) + rot[2]
			cartesian_scan = np.apply_along_axis(self.__polar_to_cartesian, 0, np.vstack((ranges, thetas)), trans, frames[i]).T

			# Add this scan's points to the list of x, y points
			fused_cartesian = np.vstack((fused_cartesian, cartesian_scan))

		self.__fused_cartesian = fused_cartesian[1:]

		# Steal a timestamp from one of the real LaserScans so we have the right time
		self.__fused_scan.header.stamp = self.__lidars[frames[0]]["scan"].header.stamp

		# Convert back to polar and publish
		self.__fused_scan.ranges = self.__cartesian_to_polar()
		self.__fused_publisher.publish(self.__fused_scan)

	def __polar_to_cartesian(self, polar, translation, frame):
		'''
		Converts polar coordinates (r, theta) into cartesian coordinates
		(x, y). Since the LiDARs are flipped and rotated in different
		directions, the signs of their x and y coordinates must be
		changed accordingly. Operates on a point that is passed in.
		'''
		if (frame == "lidar_front") or (frame == "lidar_back"):
			x = (polar[0] * np.cos(polar[1]) + translation[0])
			y = -(polar[0] * np.sin(polar[1]) + translation[1])
		else:
			x = -(polar[0] * np.cos(polar[1])) + translation[0]
			y = (polar[0] * np.sin(polar[1])) + translation[1]

		return x, y

	def __cartesian_to_polar(self):
		'''
		Converts cartesian coordinates (x, y) into polar coordinates
		(r, theta). Operates on the fused cartesian scan.
		'''
		cartesian = self.__fused_cartesian

		# Generate the angles associated with each index
		indices = int((self.__fused_scan.angle_max - self.__fused_scan.angle_min) /
			self.__fused_scan.angle_increment) + 1

		polar_point_angles = np.linspace(self.__fused_scan.angle_min, self.__fused_scan.angle_max, indices)
		polar_point_bins = np.zeros(indices) + np.inf

		# Convert cartesian to polar
		polar = np.vstack((np.sqrt(cartesian[:, 0]**2 + cartesian[:, 1]**2),
			np.arctan2(cartesian[:, 1], cartesian[:, 0]))).T

		# Find which index each polar point belongs in and put it there.
		# Note: if there are more then one radius associated with the same bin, the value will get overwritten, NOT averaged
		sorted_indices = np.digitize(polar[:, 1], polar_point_angles)
		polar_point_bins[sorted_indices-1] = polar[:, 0]

		return polar_point_bins


if __name__ == "__main__":
	rospy.init_node('lidar_fuser')
	lidar_fuser = LidarFuser()
	rospy.spin()
