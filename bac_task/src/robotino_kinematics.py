#!/usr/bin/env python
# coding: utf-8
#
# The module implements the ability to set the spatial velocity
# for the omni mobile platform FESTO Robotino2
#
# see in 'Chapter 13. Wheeled Mobile Robots' of Modern robotics, Kevin Lynch.
#
import rospy
import threading
from numpy import *
from numpy.linalg import *
from scipy.linalg import expm

import tf.transformations as tftr

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

lock = threading.Lock()


class RobotinoKinematics:

    def __init__(self):
        self.RATE = rospy.get_param('rate', 30)     # loop frequency in Hz

        """ Set robot parameters from .launch file"""
        self.d = rospy.get_param('/robotino/d')
        self.r = rospy.get_param('/robotino/r')
        self.gamma = rospy.get_param('/robotino/gamma')
        self.beta = rospy.get_param('/robotino/beta')

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.motors_velocity_pub = rospy.Publisher('/motors_velocities', Float64MultiArray, queue_size=1, latch=True)

    def get_h(self, i, phi):
        """
        :param i: the index number of the wheel (see Figure 13.5 of Modern robotics by Kevin Lynch)
        :param phi: the angle between global frame {s} and body of robot frame {b}.
        :return h_i(phi): the vector for transform Vb to u_i.
        """
        xi = - self.d * sin(self.beta[i])
        yi = self.d * cos(self.beta[i])
        return 1 / (self.r[i] * cos(self.gamma[i])) * \
               array([
                   xi * sin(self.beta[i] + self.gamma[i]) - yi * cos(self.beta[i] + self.gamma[i]),
                   cos(self.beta[i] + self.gamma[i] + phi),
                   sin(self.beta[i] + self.gamma[i] + phi)
               ])

    def get_H(self, phi):
        """
        :param phi: the angle between global frame {s} and body of robot frame {b}.
        :return H(phi) : the matrix for transform Vb to u_i.
        """
        return array([
            self.get_h(0, phi),
            self.get_h(1, phi),
            self.get_h(2, phi)
        ])

    def get_u(self, phi, vb):
        """
        :param phi: the angle between global frame {s} and body of robot frame {b}.
        :param vb: the robot spatial velocity in the robot body frame {b}
        :return u: the vector of angular velocities of wheels
        """
        return self.get_H(phi).dot(vb.T)

    def cmd_vel_callback(self, msg):
        """
        Note:
            The order of wheels in computations and simulator differ.
                - coputations:  0, 1, 2 (top, right, left)
                - simulator:    1, 0, 2 (right, top, left)
        The callback
            - recives data from /cmd_vel topic (velocity in robot body frame {b})
            - compute angular velocities for robot wheels
            and send it to simulator using /motors_velocities topic
        :param msg: geometry_msgs/Twist from /cmd_vel topic
        """
        lock.acquire()

        wbz, vbx, vby = msg.angular.z, msg.linear.x, msg.linear.y
        vb = array([wbz, vbx, vby])
        u = - self.get_u(0, vb)     # phi is equal to 0, hence the velocity is spatial velocity in robot body frame {b}

        motors_velocity = Float64MultiArray()
        motors_velocity.data.extend([u[1], u[0], u[2]])

        lock.release()

        self.motors_velocity_pub.publish(motors_velocity)

    def spin(self):
        """Main loop ROS cycle"""
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("robotino_kinematics")
        robotino_km = RobotinoKinematics()
        robotino_km.spin()
    except rospy.ROSInterruptException as e:
        print('Problems with RobotinoKinematics: ' + e)
