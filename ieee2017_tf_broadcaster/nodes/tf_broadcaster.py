#!/usr/bin/env python

'''
TF Broadcaster: Keeps track of the various coordinate frames on the vehicle in
relation to the map and each other.
'''


from __future__ import division

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2017, UF IEEE"
__license__ = "MIT"


class TFBroadcaster():

	def __init__(self):
		self.__broadcaster = tf.TransformBroadcaster()

		# Subscribe to topics for the positions of dynamic frames
		self.__odom_subscriber = rospy.Subscriber("/odom", Odometry, self.__update_odom, queue_size=2)

		# Publish the transformations at a frequency specified by the rate parameter
		self.__publishing_rate = rospy.get_param("~publishing_rate", 30)
		rospy.Timer(rospy.Rate(self.__publishing_rate), self.__publish_static, oneshot=False)

	def __update_odom(self, msg):
		msg = msg.pose.pose
		self.__broadcaster.sendTransform((msg.position.x, msg.position.y, 0),
			(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
			rospy.Time.now(), "odom", "map")

	def __publish_static(self):
		'''
		Transformations between coordinate frames that are at fixed
		positions relative to each other.
		'''

		# Transformation from odom to base_link
		# The origin of the base_link frame is at the center of the top plate
		# This provides planar coordinates of base_link on the map plus it's elevation
		self.__broadcaster.sendTransform((0, 0, 0.119),
			tf.transformations.quaternion_from_euler(0, 0, 0),
			rospy.Time.now(), "base_link", "odom")

		# Transformations from base_link to each LiDAR
		# All frames share a planar center, but the lidar_fused frame is lower
		# Each LiDAR is rotated such that the beam is emitted away from the vehicle
		self.__broadcaster.sendTransform((0, 0.125368, -0.0775),
			tf.transformations.quaternion_from_euler(0, 3.14159, -1.590796),
			rospy.Time.now(), "lidar_left", "base_link")
		self.__broadcaster.sendTransform((0.1, 0, -0.0775),
			tf.transformations.quaternion_from_euler(3.14159, 0, 0),
			rospy.Time.now(), "lidar_front", "base_link")
		self.__broadcaster.sendTransform((0, -0.125368, -0.0775),
			tf.transformations.quaternion_from_euler(0, 3.14159, 1.570796),
			rospy.Time.now(), "lidar_right", "base_link")
		self.__broadcaster.sendTransform((-0.1, 0, -0.0775),
			tf.transformations.quaternion_from_euler(3.14159, 0, 3.14159),
			rospy.Time.now(), "lidar_back", "base_link")

		# Transformations from base_link to the fused LiDAR pointcloud
		# Both frames share a planar center, but the lidar_fused frame is lower
		self.__broadcaster.sendTransform((0,0,-0.0775),
			tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "lidar_fused", "base_link")

		# Transformations from base_link to the IMU
		#self.__broadcaster.sendTransform((0,0,0),
		#	tf.transformations.quaternion_from_euler(0,0,0),
		#	rospy.Time.now(), "imu", "base_link")


if __name__ == "__main__":
	rospy.init_node('tf_broadcaster')
	tf_broadcaster = TFBroadcaster()
	rospy.spin()
