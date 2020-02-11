#!/usr/bin/env python
# coding: utf-8
"""
The module implements the ability to set the spatial velocity
for the omni-mobile platform FESTO Robotino2.

See in 'Chapter 13. Wheeled Mobile Robots' of Modern robotics, Kevin Lynch.
"""
import rospy
import threading
from numpy import *

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

lock = threading.Lock()


class RobotinoModel:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.RATE = rospy.get_param('/rate', 50)  # loop frequency, Hz

        """ Set robot parameters from .launch file"""
        self.d = rospy.get_param('/' + self.robot_name + '/d', 0.13)
        self.r = rospy.get_param('/' + self.robot_name + '/r', [0.0395, 0.0395, 0.0395])
        self.gamma = rospy.get_param('/' + self.robot_name + '/gamma', [0.0, 0.0, 0.0])
        self.beta = rospy.get_param('/' + self.robot_name + '/beta', [0.0, -2.0943951023931953, 2.0943951023931953])

        self.cmd_vel_sub = rospy.Subscriber('/' + self.robot_name + '/cmd_vel', Twist, self.cmd_vel_callback)
        self.motors_velocity_pub = rospy.Publisher('/' + self.robot_name + '/motors_velocities', Float64MultiArray, queue_size=1, latch=False)

    def __del__(self):
        self.cmd_vel_sub.unregister()
        self.motors_velocity_pub.unregister()

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
                - computations:  0, 1, 2 (top, right, left)
                - simulator:     1, 0, 2 (right, top, left)
        The callback
            - recives data from /cmd_vel topic (velocity in robot body frame {b})
            - compute angular velocities for robot wheels
            and send it to simulator using /motors_velocities topic
        :param msg: geometry_msgs/Twist from /cmd_vel topic
        """
        lock.acquire()

        wbz, vbx, vby = msg.angular.z, msg.linear.x, msg.linear.y
        vb = array([wbz, vbx, vby])
        u = - self.get_u(0, vb)  # phi equals 0, hence the velocity is spatial velocity in robot body frame {b}

        motors_velocity = Float64MultiArray()
        motors_velocity.data.extend([u[1], u[0], u[2]])

        self.motors_velocity_pub.publish(motors_velocity)

        lock.release()

    def spin(self):
        """Main loop of ROS cycle"""
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("robotino_model_node")
    robot_name = rospy.get_param('/ground_robot_name', 'robotino')
    robotino_km = None
    while not rospy.is_shutdown():
        try:
            robotino_km = RobotinoModel(robot_name)
            robotino_km.spin()
        except rospy.ROSTimeMovedBackwardsException as e:
            del robotino_km
            print('Restart robotino_model_node with new ros-sim-time')
