#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
import time
import sys
import tf.transformations as tftr
from numpy import *

from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from bac_task.msg import CartesianTrajectory
from bac_task.msg import CameraFeatures


"""""""""""""""""""""""""""""""""""""""
    WRITE YOUR DATA BELOW, PLEASE
"""""""""""""""""""""""""""""""""""""""

__author__ = u"Пупкин Василий Бенедиктович"
__id__ = "123456789"
__university__ = "ITMO University"

"""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""""""""""""""""
"""""""""""""""""""""""""""""""""""""""

lock = threading.Lock()


class RobotinoController:

    def __init__(self):
        self.cart_trajectory = None
        self.odometry = None

        "ROS stuff"
        self.trajectory_sub = rospy.Subscriber("/robotino/trajectory", CartesianTrajectory, self.cart_trajectory_callback)
        self.odom_sub = rospy.Subscriber("/robotino/odom", Odometry, self.odometry_callback)
        self.robotino_cmd_vel_pub = rospy.Publisher('/robotino/cmd_vel', Twist, queue_size=1)

        self.ts_d = []
        self.i = 0

    def cart_trajectory_callback(self, msg):
        """
        Trajectory for Robotino.
        Gets trajectory from robotino_trajectory_generator_node and saves to self variable.
        """
        lock.acquire()
        self.cart_trajectory = msg
        lock.release()

    def odometry_callback(self, msg):
        """
        Odometry (state) for Robotino.
        Can be used for feedback for trajectory controller
        """
        lock.acquire()
        self.odometry = msg
        lock.release()

    def stop(self):
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = 0
        msg.linear.y = 0
        self.robotino_cmd_vel_pub.publish(msg)

        # self.robotino_cmd_vel_pub.unregister()
        self.odom_sub.unregister()
        self.trajectory_sub.unregister()

    def exists_odometry(self):
        if self.odometry is not None:
            return True
        else:
            return False

    def exists_cart_trajectory(self):
        if self.cart_trajectory is not None:
            return True
        else:
            return False

    def get_q_des(self, t, i):
        """
        SOLVE interpolation between several points of trajectory.
        Quantity of points selects by wide (wide of windows)
        """
        wide = 10
        N = len(self.cart_trajectory.poses)
        ps = []
        ts = []
        if i <= wide:
            ps = self.cart_trajectory.poses[0:2 * wide + 1]
            ts = self.cart_trajectory.time_from_start[0:2 * wide + 1]
        elif N - wide > i > wide:
            ps = self.cart_trajectory.poses[i - wide - 1:i + wide]
            ts = self.cart_trajectory.time_from_start[i - wide - 1:i + wide]
        elif N > i >= N - wide:
            ps = self.cart_trajectory.poses[N - 2 * wide - 1:N]
            ts = self.cart_trajectory.time_from_start[i - wide - 1:i + wide]

        ts = [ts[0].to_sec() for i in range(len(ps))]
        px = interp(t, [ps[i].x for i in range(len(ps))], ts)
        py = interp(t, [ps[i].y for i in range(len(ps))], ts)
        ptheta = interp(t, [ps[i].theta for i in range(len(ps))], ts)
        return matrix([ptheta, px, py]).T

    def translate_time(self):
        self.ts_d = []
        for i, t in enumerate(self.cart_trajectory.time_from_start):
            sys.stdout.write('\r')
            sys.stdout.write("{}\t{}".format(i, len(self.cart_trajectory.time_from_start)))
            sys.stdout.flush()

            self.ts_d.append(t.to_sec())
        print()

    def get_translated_time(self):
        return self.ts_d

    def get_quantity_points(self, traj):
        return len(traj)

    def update(self, dt, t):
        """ Update control signal for robotino """
        
        ####################################################
        #### TODO WRITE YOUR SOLUTION BELOW! ###############
        #### THIS IS ROBOTINO CONTROLLER ###################
        #### THIS CODE JUST FOR EXAMPLE ####################
        ####################################################

        # DESIRED POSE [x, y, theta]
	# i = ...
        # goal_pose = self.cart_trajectory.poses[i]

        # CURRENT POSE  
        xyz = self.odometry.pose.pose.position
        q = self.odometry.pose.pose.orientation     # quaternion
        rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))  # roll pitch yaw


        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.angular.z = 0.0
        self.robotino_cmd_vel_pub.publish(velocity)



class DroneController:

    def __init__(self):
        self.Z0 = rospy.get_param('/drone_altitude', 2.0)   # Initial altitude of the drone.
                                                            # Could be changed by judge while testing
        "Camera parameters"
        self.width = 640
        self.height = 480
        self.f = 0.002  # focal distance
        self.px = 600  # represent focal length in terms of pixels by x/y dims
        self.py = 600
        self.u0 = 320  # represent the principal point
        self.v0 = 240

        "ROS stuff"
        self.marker_features_sub = rospy.Subscriber('/marker/features', CameraFeatures, self.marker_features_callback)
        self.drone_odom_sub = rospy.Subscriber('/drone/odom', Odometry, self.drone_odom_callback)
        self.drone_target_pose_pub = rospy.Publisher('/drone/target_pose', Pose, queue_size=1, latch=True)

        "Drone state"
        self.drone_state_position = Point()
        self.drone_state_orientation = Vector3()
        self.drone_state_lin_vel = Vector3()
        self.drone_state_ang_vel = Vector3()

        "Camera features for feedback"
        self.camera_features = CameraFeatures()
        

    def drone_odom_callback(self, msg):
        """
        Odometry callback.
        Gets drone state from CoppeliaSim. Like a current position, orientation and velocities
        """
        lock.acquire()

        self.drone_state_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        roll, pitch, yaw = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.drone_state_orientation = Vector3(roll, pitch, yaw)

        self.drone_state_lin_vel = msg.twist.twist.linear
        self.drone_state_ang_vel = msg.twist.twist.angular

        lock.release()

    def marker_features_callback(self, msg):
        """
        Features extraction callback.
        Gets features ()
        """
        lock.acquire()
        self.camera_features = msg
        lock.release()

    def stop(self):
        """ Reset the robot """
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = self.Z0
        self.drone_target_pose_pub.publish(pose)

        self.marker_features_sub.unregister()
        self.drone_odom_sub.unregister()
        self.drone_target_pose_pub.unregister()

    def update(self, dt, t):
        """ Update control signal for drone """
        ####################################################
        #### TODO WRITE YOUR SOLUTION BELOW! ###############
        #### THIS IS DRONE CONTROLLER ######################
        #### THIS CODE JUST FOR EXAMPLE ####################
        ####################################################

        self.pos_x = 0
        self.pos_y = 0

                
        pose = Pose()
        pose.position.x = self.pos_x
        pose.position.y = self.pos_y
        pose.position.z = self.Z0   # don't change this. It sets from judge node
        self.drone_target_pose_pub.publish(pose)


