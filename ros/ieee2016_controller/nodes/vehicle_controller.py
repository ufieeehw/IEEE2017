#!/usr/bin/env python
from __future__ import division
## Math
import numpy as np
import math
## Ros/tools
import tf
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3
from ieee2016_msgs.msg import Mecanum
from ieee2016_msgs.srv import StopController, NavWaypoint
from nav_msgs.msg import Odometry

# max_linear_vel = 1 # m/s
max_linear_vel = 0.420
max_linear_acc = max_linear_vel # m/s^2

# max_angular_vel = 2 # rad/s
max_angular_vel = 0.69 # rad/s
max_angular_acc = max_angular_vel # rad/s^2 

# (Jason says this is just called angular acceleration, # I call it angcelleration)


def print_in(f):
    '''Shitty decorator for printing function business'''
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name)
        result = f(*args, **kwargs)
    
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)


class Controller(object):
    '''Controller object 
    See Jacob Panikulam or Aaron Marquez for questions

    Function:
        - Determine the ideal velocity for the current position

    Notes:
        Special glory to Lord Forrest Voight, creator of the universe
        Greater thanks to the incredible, brilliant and good-looking Khaled Hassan, for his unyielding mentorship
        and love for Burrito Bros
    '''
    def __init__(self):
        '''Initialize Controller Object'''
        rospy.init_node('vehicle_controller')

        # Twist pub
        self.twist_pub = rospy.Publisher('/robot/twist', TwistStamped, queue_size=1) 
        
        self.tf_listener = tf.TransformListener()

        # Initializations to avoid weird desynchronizations
        self.des_position = None
        self.des_yaw = None
        # Don't want to have an a-priori position
        self.position = None
        self.yaw = None
        # This will be used for the velocity profile
        self.starting_move_error = None

        # Current pose sub
        self.pose_sub = rospy.Subscriber('/robot/pf_pose_est', PoseStamped, self.got_pose)
        #self.odom_sub = rospy.Subscriber('/robot/odom', Odometry, self.got_odom)
        #self.desired_pose_sub = rospy.Subscriber('/robot/waypoint', PoseStamped, self.got_desired_pose)

        rospy.Service('controller/stop', StopController, self.stop)
        rospy.Service('/robot/nav_waypoint', NavWaypoint, self.got_desired_pose)

        self.on = True
        print "Initialization Finished."

        rospy.spin()

    def stop(self, req):
        self.on = not req.stop
        if not self.on:
            rospy.logwarn("DISABLING MECANUM CONTROLLER")
            self.des_position = None
            self.des_yaw = None
        else:
            rospy.logwarn("ENABLING MECANUM CONTROLLER")
            self.des_position = None
            self.des_yaw = None

    def send_twist(self, (xvel, yvel), angvel):
        '''Generate twist message'''
        #rospy.loginfo("Send")
        self.twist_pub.publish(
            TwistStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='base_link',
                ),
                twist=Twist(
                    linear=Vector3(xvel, yvel, 0),
                    angular=Vector3(0, 0, angvel),  # Radians
                )
            )
        )

    def norm_angle_diff(self, ang_1, ang_2):
        '''norm_angle_diff(ang_1, ang_2)
        -> Normalized angle difference, constrained to range [-pi, pi]
        '''
        return ((ang_1 - ang_2 + np.pi) % (2 * np.pi)) - (np.pi)

    def unit_vec(self, v):
        '''unit_vec(v)'''
        norm = np.linalg.norm(v)
        if norm == 0:
            return(v)
        return np.array(v) / np.linalg.norm(v)

    def vec_diff(self, v1, v2):
        '''norm(v1 - v2)'''
        assert isinstance(v1, np.array)
        assert isinstance(v2, np.array)
        diff = np.linalg.norm(v1 - v2)
        return(diff)

    def sign(self, x):
        '''return sign of x -> -1, 1 or 0'''
        if x > 0: return 1
        elif x < 0: return -1
        else: return 0

    def got_odom(self, msg):

        pose = msg.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(pose.orientation))[2]

    def got_pose(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

    def control(self):
        '''recieve current pose of robot

        Function:
            Attempts to construct a velocity solution using a k*sqrt(error) controller
            This tries to guarantee a constant acceleration

        Note:
            Right now, this does not require velocity feedback, only pose
            This SHOULD include velocity feedback, once we have it

        Todo:
            Add speed-drop for the case where position feedback is lost
            This will probably have to be done in a separate thread

        Changes from last year:
            Enabled backwards and side to side motion. Added a different velocity profile.

        Velocity calcluation should be done in a separate thread
         this thread should have an independent information "watchdog" timing method
        '''
        if (self.position is None) or (self.yaw is None) or (self.on is False): return
        
        r = rospy.Rate(25) #hz
        # Loop until there is no command being sent out.
        command = ['_']
        while command or not rospy.is_shutdown():
            # World frame position
            position_error = self.des_position - self.position
            yaw_error = self.norm_angle_diff(self.des_yaw, self.yaw)
            print "RAW ERR:",position_error

            rot_mat = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                                [math.sin(self.yaw),  math.cos(self.yaw)]])
            
            position_error = np.dot(position_error,rot_mat)
            print "ERR:",position_error,yaw_error

            nav_tolerance = (.001,.001) #m, rads
            command = [] # 'X' means move in x, 'Y' move in y, 'R' means rotate

            # Determine which commands to send based on how close we are to target
            if abs(position_error[0]) > nav_tolerance[0]:
                command.append('X')
            if abs(position_error[1]) > nav_tolerance[0]:
                command.append('Y')
            if abs(yaw_error) > nav_tolerance[1]:
                command.append('R')

            if self.starting_move_error is None: 
                self.starting_move_error = np.linalg.norm(position_error) * max_linear_acc + .01
                print "MV_ERR",self.starting_move_error

            linear_speed_raw = math.sqrt(np.linalg.norm(position_error) * max_linear_acc) * \
                               math.pow(self.starting_move_error - (np.linalg.norm(position_error) * max_linear_acc),(1/3.0))
            # Determines the linear speed necessary to maintain a consant backward acceleration
            linear_speed = min(
                                .6*linear_speed_raw, max_linear_vel
                            )
            # Determines the angular speed necessary to maintain a constant angular acceleration 
            #  opposite the direction of motion
            angular_speed = min(
                                .5*math.sqrt(abs(yaw_error) * max_angular_acc), 
                                max_angular_vel
                            )

            # Provide direction for both linear and angular velocity
            desired_vel = linear_speed * self.unit_vec(position_error)
            desired_angvel = angular_speed * self.sign(yaw_error)

            print command
            target_vel = [0,0]
            target_angvel = 0
            if 'X' in command:
                target_vel[0] = desired_vel[0]
            if 'Y' in command:
                target_vel[1] = desired_vel[1]
            if 'R' in command:
                target_angvel = desired_angvel
            if not command:
                # Break when we are finished moving
                break

            print "VEL:",target_vel,target_angvel

            self.send_twist(target_vel, target_angvel)
            r.sleep()


    def got_desired_pose(self, srv):
        '''Recieved desired pose message
        Figure out how to do this in a separate thread
        So we're not depending on a message to act
        #LearnToThreading
        (That's a hashtag)
        '''
        self.starting_move_error = None

        msg = srv.des_pose
        # Make sure the pose is in the map frame
        try:
            self.tf_listener.waitForTransform(msg.header.frame_id,"/map", rospy.Time.now(), rospy.Duration(1.0))
            msg = self.tf_listener.transformPose("/map",msg)
        except:
            return False

        self.des_position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.des_yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

        # Only do this when we have a desired pose
        self.control()
        print "Movement Complete!"
        return True

if __name__ == '__main__':
    rospy.logwarn("Starting")
    controller = Controller()
    rospy.spin()
    
