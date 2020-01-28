#!/usr/bin/env python
try:
    from coppelia_sim_interface import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import rospy
import threading
import time
from numpy import *
from numpy.linalg import pinv

import tf
import tf.transformations as tftr

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, TwistWithCovariance

lock = threading.Lock()


class RobotsCoppeliaSimInterface:

    def __init__(self):
        rospy.init_node('robots_coppelia_sim_interface')
        rospy.loginfo('CoppeliaSim + ROS interface started! (RemoteAPI)')
        self.RATE = rospy.get_param('rate', 50)

        self.clientID = -1
        self.robotino_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.robotino_motors_velosities_sub = None

    def __del__(self):
        if self.clientID != -1:
            sim.simxGetPingTime(self.clientID)
            sim.simxFinish(self.clientID)
        if self.robotino_motors_velosities_sub:
            self.robotino_motors_velosities_sub.unregister()
        self.robotino_odom_pub.unregister()
        rospy.loginfo('CoppeliaSim + ROS interface ended! (RemoteAPI)')

    def robotino_motors_velocities_callback(self, msg):
        """ Sets motors velocities for Robotino2 model in simulator. """
        lock.acquire()
        sim.simxSetJointTargetVelocity(self.clientID, self.motor1_handler, msg.data[0], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.motor2_handler, msg.data[1], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.motor3_handler, msg.data[2], sim.simx_opmode_streaming)
        lock.release()

    def spin(self):
        rate = rospy.Rate(self.RATE)
        listener = tf.TransformListener()

        startTime = time.time()
        time_prev = 0
        while not rospy.is_shutdown():
            t = time.time() - startTime
            dt = t - time_prev
            time_prev = t

            if self.clientID != 0:

                # CoppeliaSim connection
                sim.simxFinish(-1)  # just in case, close all opened connections
                self.clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

                if self.clientID != -1:
                    # Now try to retrieve data in a blocking fashion (i.e. a service call):
                    _, self.omni_handler = sim.simxGetObjectHandle(self.clientID, 'robotino', sim.simx_opmode_blocking)
                    _, self.motor1_handler = sim.simxGetObjectHandle(self.clientID, 'wheel0_joint',
                                                                     sim.simx_opmode_blocking)
                    _, self.motor2_handler = sim.simxGetObjectHandle(self.clientID, 'wheel1_joint',
                                                                     sim.simx_opmode_blocking)
                    _, self.motor3_handler = sim.simxGetObjectHandle(self.clientID, 'wheel2_joint',
                                                                     sim.simx_opmode_blocking)
                    time.sleep(1)

                    # ROS initialisation
                    self.robotino_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
                    self.robotino_motors_velosities_sub = rospy.Subscriber('/motors_velocities', Float64MultiArray,
                                                                           self.robotino_motors_velocities_callback)
                    rospy.loginfo('Seems, we connected to CoppeliaSim API')
                else:
                    rospy.logwarn('Connection to coppeliaSim API...')
            else:
                # Main loop
                returnCode, timeSim = sim.simxGetFloatSignal(self.clientID, 'timeSim', sim.simx_opmode_streaming)
                if returnCode == sim.simx_return_ok:
                    # Odometry-like code for robotino
                    try:
                        (trans, rot) = listener.lookupTransform('/world', '/odom', rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    odom = Odometry()
                    odom.header.frame_id = 'robotino'
                    odom.pose.pose.position = Point(trans[0], trans[1], 0)
                    odom.pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
                    self.robotino_odom_pub.publish(odom)

            rate.sleep()


if __name__ == '__main__':
    try:
        csim_interface = RobotsCoppeliaSimInterface()
        csim_interface.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr('RobotsCoppeliaSimInterface: ' + e)
