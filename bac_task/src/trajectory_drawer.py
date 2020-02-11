#!/usr/bin/env python
"""
2D trajectory visualizer for rviz
"""
import rospy
import threading

import tf.transformations as tftr

from std_msgs.msg import Header
from bac_task.msg import CartesianTrajectory
from geometry_msgs.msg import PolygonStamped, Pose, PoseArray, Quaternion, Point

RATE = 2    # Update frequency of trajectory topics is RATE Hz
lock = threading.Lock()


class TrajectoryDrawer:

    def __init__(self):
        self.poses = []
        self.points = []
        self.header = Header()

        self.sub = rospy.Subscriber("/robotino/trajectory", CartesianTrajectory, self.cart_trajectory_callback)
        self.pub_polygon = rospy.Publisher("trajectory_polygon", PolygonStamped, queue_size=1)
        self.pub_pose_array = rospy.Publisher("trajectory_poses", PoseArray, queue_size=1)

        self.loop()

    def __del__(self):
        self.sub.unregister()
        self.pub_polygon.unregister()
        self.pub_pose_array.unregister()

    def cart_trajectory_callback(self, msg):
        lock.acquire()
        self.header = msg.header

        self.points = []
        self.poses = []
        for pose in msg.poses:
            self.points.append(Point(pose.x, pose.y, 0.))
            self.poses.append(pose.theta)

        lock.release()

    def draw_polygon(self):
        msg = PolygonStamped()
        msg.header = self.header
        msg.polygon.points = self.points
        self.pub_polygon.publish(msg)

    def draw_pose(self):
        msg = PoseArray()
        msg.header = self.header
        for i, pose in enumerate(self.poses):
            msg.poses.append(Pose())
            msg.poses[i].position = self.points[i]

            q = tftr.quaternion_about_axis(self.poses[i], (0, 0, 1))
            msg.poses[i].orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.pub_pose_array.publish(msg)

    def loop(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            lock.acquire()
            self.draw_polygon()
            self.draw_pose()
            lock.release()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory_drawer_node')
    dr = None
    while not rospy.is_shutdown():
        try:
            dr = TrajectoryDrawer()
        except rospy.ROSInterruptException as e:
            del dr
            print('Restart trajectory_drawer_node with new ros-sim-time')
