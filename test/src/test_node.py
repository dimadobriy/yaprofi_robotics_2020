#!/usr/bin/env python
import rospy
import threading
import numpy
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


# PD-controller's gains for p-to-p movements
Kp_x, Kd_x = 0.5, 5
Kp_y, Kd_y = 0.5, 5

# Goal points list
points = [Point(1, 1, 0), Point(1, -1, 0), Point(-1, -1, 0), Point(-1, 1, 0)]
time_step = 5.0     # time for each point

lock = threading.Lock()


class TestNode:
    """
    Point-to-point node controller for mobile robot
    """

    def __init__(self):
        rospy.init_node('test_node')
        self.RATE = rospy.get_param('rate', 50)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # initial conditions
        self.goal = Point(0, 0, 0)
        self.err_x_old = 0
        self.err_y_old = 0


    def odom_callback(self, msg):
        lock.acquire()
        err_x = self.goal.x - msg.pose.pose.position.x
        err_y = self.goal.y - msg.pose.pose.position.y
        lock.release()

        derr_x = err_x - self.err_x_old
        derr_y = err_y - self.err_y_old

        self.err_x_old = err_x
        self.err_y_old = err_y

        velocity = Twist()
        velocity.linear.x = 0.5 * err_x + 5 * derr_x
        velocity.linear.y = 0.5 * err_y + 5 * derr_y
        self.cmd_vel_pub.publish(velocity)

    def spin(self):
        rate = rospy.Rate(self.RATE)

        time_start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - time_start
            if t < time_step:
                self.goal = points[0]
            elif t < 2 * time_step:
                self.goal = points[1]
            elif t < 3 * time_step:
                self.goal = points[2]
            elif t < 4 * time_step:
                self.goal = points[3]
            else:
                time_start = time.time()
                self.goal = points[0]
            rate.sleep()


if __name__ == "__main__":
    try:
        test_none = TestNode()
        test_none.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr('RobotsCoppeliaSimInterface: ' + e)

