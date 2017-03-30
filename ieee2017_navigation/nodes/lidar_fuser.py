#!/usr/bin/env python

'''
LiDAR Fuser: This node takes in the data from each of the LiDARs and combines
it into one large point cloud centered on the vehicle.
'''


from __future__ import division

import os
import time
import math
import numpy as np
import pyopencl as cl

import rospy
import tf
import message_filters
from std_msgs.msg import Header, Int8, Bool
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud


__author__ = "Matthew Langford"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2017, UF IEEE"
__license__ = "MIT"


class LaserFuser():

	def __init__(self):
		self.map_version_pub = rospy.Publisher('/settings/map_version', Int8, queue_size=3)
		self.start_pub = rospy.Publisher('/settings/start_command', Bool, queue_size=3)
		self.scan_pub = rospy.Publisher('/robot/navigation/lidar/scan_fused', LaserScan, queue_size=3)
		#self.scan_pc_pub = rospy.Publisher('/robot/navigation/lidar/scan_fused_pc', PointCloud, queue_size=3)
		rospy.init_node('laser_fuser')

		# Start a TF listener and make sure it gets data before preceding
		self.tf_listener = tf.TransformListener()
		print ""
		print " > Waiting for TF..."
		self.tf_listener.waitForTransform("base_link","laser_left", rospy.Time(0), rospy.Duration(5.0))
		self.tf_listener.waitForTransform("base_link","laser_front", rospy.Time(0), rospy.Duration(5.0))
		self.tf_listener.waitForTransform("base_link","laser_right", rospy.Time(0), rospy.Duration(5.0))
		self.tf_listener.waitForTransform("base_link","laser_back", rospy.Time(0), rospy.Duration(5.0))
		self.frames = ["laser_left","laser_front","laser_right","laser_back"]
		self.FOVs = [80,80,80,80] #degrees
		print " > TF Found! Starting Fuser."

		# # Define our message filters - these will make sure the laserscans come in at the same time
		# left_sub = message_filters.Subscriber('/robot/navigation/lidar/scan_left',LaserScan, queue_size=3)
		# front_sub = message_filters.Subscriber('/robot/navigation/lidar/scan_front',LaserScan, queue_size=3)
		# right_sub = message_filters.Subscriber('/robot/navigation/lidar/scan_right',LaserScan, queue_size=3)
		# back_sub = message_filters.Subscriber('/robot/navigation/lidar/scan_back',LaserScan, queue_size=3)
		# mf_ts = message_filters.ApproximateTimeSynchronizer([left_sub,front_sub,right_sub,back_sub], 50, 5)
		# mf_ts.registerCallback(self.got_scans)

		self.scans = [[],[],[],[]]

		# Define new laserscan
		self.fused_scan = LaserScan()
		self.fused_scan.header.frame_id = "laser_fused"
		self.fused_scan.angle_increment = .007 #rads/index
		self.fused_scan.angle_min = -3.14159274101 #rads
		self.fused_scan.angle_max = 3.14159274101 #rads
		self.fused_scan.range_max = 5.0 #m
		self.fused_scan.range_min = 0.0002 #m

		rospy.Subscriber('/robot/navigation/lidar/scan_left',LaserScan,self.got_scan, queue_size=3)
		rospy.Subscriber('/robot/navigation/lidar/scan_front',LaserScan,self.got_scan, queue_size=3)
		rospy.Subscriber('/robot/navigation/lidar/scan_right',LaserScan,self.got_scan, queue_size=3)
		rospy.Subscriber('/robot/navigation/lidar/scan_back',LaserScan,self.got_scan, queue_size=3)

		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Since message filters don't work sometimes this is required.
			if self.left and self.front and self.right and self.back:
				self.got_scans()
			r.sleep()

		# For checking for gestures
		self.left,self.front,self.right,self.back = False, False, False, False
		self.started = False
		self.time_checker = time.time()
		self.listen_for_start()

		rospy.spin()

	def got_scan(self, msg):
		info = msg.header.frame_id,msg.header.stamp
		#rospy.loginfo(info)
		if msg.header.frame_id == 'laser_left':
			self.left = True
			self.scans[0] = msg
		elif msg.header.frame_id == 'laser_front':
			self.front = True
			self.scans[1] = msg
		elif msg.header.frame_id == 'laser_right':
			self.right = True
			self.scans[2] = msg
		elif msg.header.frame_id == 'laser_back':
			self.back = True
			self.scans[3] = msg

	def got_scans(self):
		# Take all four scans, trim to a new FOV, convert them to cartesian, apply transformations, pub pointcloud, tranform back to cart, and pub laserscan
		#rospy.loginfo("got")
		fused_cartesian = np.array([-1,-1])
		for i,s in enumerate(self.scans):
			# Define parameters based on scan data
			angle_increment = s.angle_increment
			ranges = np.array(s.ranges)
			ranges_size = ranges.size

			# Trim to FOV
			new_FOV = math.radians(self.FOVs[i])
			angle_min = -(new_FOV/2.0)
			angle_max = (new_FOV/2.0)
			ranges = ranges[int(ranges_size/2 + angle_min/angle_increment): int(ranges_size/2 + angle_max/angle_increment)]
			ranges_size = ranges.size
			#self.pubish_scan(s,ranges,self.frames[i],angle_min,angle_max,angle_increment)

			# Get TF data about the LIDAR position
			(trans,q_rot) = self.tf_listener.lookupTransform('base_link',self.frames[i], rospy.Time(0))
			rot = tf.transformations.euler_from_quaternion(q_rot)

			# Generate list that maps indices of the ranges to radian angle measures
			thetas = np.linspace(angle_min, angle_max, ranges.size, endpoint=True) + rot[2]
			cartesian_scan = np.apply_along_axis(self.polar_to_cart,0,np.vstack((ranges,thetas)),trans,self.frames[i]).T# + np.array(trans[:2])

			# Add this scan's points to the list of x,y points
			fused_cartesian = np.vstack((fused_cartesian,cartesian_scan))

		#print ranges_size*3
		self.fused_cartesian = fused_cartesian[1:]
		#self.publish_pc(self.fused_cartesian)

		# Now we convert back to polar and publish
		# Steal a timestamp from one of the real laserscans so we are in the right timeframe
		self.fused_scan.header.stamp = self.scans[1].header.stamp
		self.fused_scan.ranges = self.cart_to_polar()
		self.scan_pub.publish(self.fused_scan)

	def listen_for_start(self):
		'''
		Check for the start gesture (covering front amd back lidars), then use the left and right lidars to determine the map layout.
		'''
		# Publish start command.
		r = rospy.Rate(10)
		while not self.started and not rospy.is_shutdown():
			self.check_start_command()
			r.sleep()

		time.sleep(.5)
		# Publish map version
		self.check_map_version()

	def check_map_version(self):
		# Take the average of the left and right sides, removing any NaN's or Inf's
		l_scan = np.array(self.scans[0].ranges)
		r_scan = np.array(self.scans[2].ranges)

		if np.nanmean(l_scan[np.isfinite(l_scan)]) < np.nanmean(r_scan[np.isfinite(r_scan)]):
			self.map_version_pub.publish(Int8(data=1))
		else:
			self.map_version_pub.publish(Int8(data=2))

	def check_start_command(self):
		f_scan = np.array(self.scans[1].ranges)
		b_scan = np.array(self.scans[3].ranges)

		if np.nanmean(f_scan[np.isfinite(f_scan)]) < .2 and np.nanmean(b_scan[np.isfinite(b_scan)]) < .2:
			print time.time() - self.time_checker
			if time.time() - self.time_checker > 2:
				self.start_pub.publish(Bool(data=True))
				self.started = True
		else:
			self.time_checker = time.time()

	def polar_to_cart(self,r_theta,translation,frame):
		# Convert (r,theta) into (x,y)
		# Since the LIDAR are flipped and rotated in different directions, you have to change the signs of x and y based on the scan
		if frame == "laser_front" or frame == "laser_back":
			x = (r_theta[0] * np.cos(r_theta[1]) + translation[0])
			y = -(r_theta[0] * np.sin(r_theta[1]) + translation[1])
		else:
			x = -(r_theta[0] * np.cos(r_theta[1])) + translation[0]
			y = (r_theta[0] * np.sin(r_theta[1])) + translation[1]

		return x,y

	def cart_to_polar(self):
		x_y = self.fused_cartesian
		# Generate the angles associated with each indices.
		indices = int((self.fused_scan.angle_max-self.fused_scan.angle_min)/self.fused_scan.angle_increment)+1

		polar_points_angles = np.linspace(self.fused_scan.angle_min,self.fused_scan.angle_max,indices)
		polar_points_bins = np.zeros(indices) + np.inf

		# Convert (x,y) to (r,theta). It's faster to use all numpy functions.
		r_theta = np.vstack((np.sqrt(x_y[:,0]**2 + x_y[:,1]**2),np.arctan2(x_y[:,1], x_y[:,0]))).T

		# Find which indices each (r,theta) point belongs in and put it there.
		# Note: if there is more then one r associated with the same bin, the value will get overwritten NOT averaged
		sorted_indices = np.digitize(r_theta[:,1],polar_points_angles)
		polar_points_bins[sorted_indices-1] = r_theta[:,0]

		return polar_points_bins

	def publish_pc(self,points,frame_id="laser_fused"):
		pc_points = []
		for p in points:
			if np.isfinite(p.any()): pc_points.append(Point32(x=p[0],y=p[1],z=0))

		self.scan_pc_pub.publish(PointCloud(
			header = Header(
				stamp=rospy.Time.now(),
				frame_id=frame_id
				),
			points=pc_points
			)
		)

l = LaserFuser()
