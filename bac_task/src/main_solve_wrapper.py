#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
import time
from numpy import *

from std_srvs.srv import SetBool

from main_solve import RobotinoController, DroneController

lock = threading.Lock()


class Main:
    """
    Class initialize robot controllers (with participants solutions) and
    start/stop movements can be controlled using
        for start: `rosservice call /start_robots {data: true}`
        for stop: `rosservice call /start_robots {data: false}`
    """

    def __init__(self):
        self.isStarted = False
        self.srv = None
        try:
            self.srv = rospy.Service('/start_robots', SetBool, self.start_robots)
        except rospy.service.ServiceException as se:
            # it's ok. Service already registered
            pass

        self.drone_controller = None
        self.robotino_controller = None

    def close(self):
        self.srv.shutdown()

    def task_loop(self):
        RATE = rospy.get_param('/rate', 50)
        dt = rospy.get_param('/dt', 0.02)
        rate = rospy.Rate(RATE)

        while not rospy.is_shutdown():

            if self.isStarted:

                # initialize robot controllers
                if self.drone_controller is None and self.robotino_controller is None:
                    self.drone_controller = DroneController()
                    self.robotino_controller = RobotinoController()

                # wait while all processes start successfully
                rospy.loginfo('Wait for Robotino\' odometry and trajectory... (Is simulation start?)')
                #print("Time translation... Please wait...")
                while not rospy.is_shutdown() and self.isStarted:

                    if self.robotino_controller.exists_odometry() and self.robotino_controller.exists_cart_trajectory():
                        # some prepares
                        rospy.loginfo('Ready!')
                        #self.robotino_controller.translate_time()   # ROS-timestamp -> seconds
                        break
                    rate.sleep()

                rospy.loginfo('Start movements!')
                time_prev = 0
                time_start = rospy.get_time()
                while not rospy.is_shutdown() and self.isStarted:
                    t = rospy.get_time() - time_start
                    #dt = t - time_prev     # !!! used global variable from simulation parameters
                    time_prev = t

                    self.robotino_controller.update(dt, t)
                    self.drone_controller.update(dt, t)

                    rate.sleep()
            else:
                # Stop robots and exit to top loop
                if self.drone_controller is not None and self.robotino_controller is not None:
                    while not rospy.is_shutdown():
                        rospy.loginfo('Stop movements!')
                        self.robotino_controller.stop()
                        self.drone_controller.stop()

                        self.drone_controller = None
                        self.robotino_controller = None
                        break
            rate.sleep()

    def start_robots(self, req):
        """
        Called by Service /start_robots and
        and turn on or tun off robots
        """
        self.isStarted = req.data
        return [1, ""]


if __name__ == '__main__':
    rospy.init_node("main_solve_node")
    rospy.loginfo("Call service /start_robots")

    main = None
    while not rospy.is_shutdown():
        try:
            main = Main()
            main.task_loop()
        except rospy.ROSInterruptException as e:
            main.close()
            del main
            print('Restart main_solve_node with new ros-sim-time')


