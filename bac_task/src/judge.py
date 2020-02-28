#!/usr/bin/env python
# coding: utf-8
import sys
import os
import threading
import hashlib

import third_party_libs.sim as sim
import rospy
import time
import numpy

from std_srvs.srv import SetBool
from bac_task.srv import SetTrajectoryParameters
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from bac_task.msg import CartesianTrajectory

import main_solve


lock = threading.Lock()

RATE = 50  # [Hz]
debug = False
TRANSIENT_TIME = 5.0    # wait while drone stabilised on trajectory

ROBOTINO_NAME = 'robotino'
DRONE_NAME = 'drone'

"TASK CRITERIA"
T_MAX = 10.0 * 60.0  # [sec]
ERROR_ROBOTINO_XY_MAX = 0.2  # [m]
ERROR_DRONE_XY_MAX = 0.5  # [m]
ERROR_DRONE_Z_MAX = 0.3  # [m]

"ROUNDS PARAMETERS"
tests_trajectories = [
    {'laps': 10, 'velocity': 0.1, 'A_x': 1.0, 'A_y': 1.0, 'phi': -1.57, "Z0": 1.5}

]
#    {'laps': 10, 'velocity': 0.05, 'A_x': 1.0, 'A_y': 1.0, 'phi': -1.57, "Z0": 2.0}


class Judge:

    def __init__(self, _author, _id, _university, _md5sum, _tests_trajectories):
        print("Judge started")
	
        velocity = _tests_trajectories[0]['velocity']
        A_x = _tests_trajectories[0]['A_x']
        A_y = _tests_trajectories[0]['A_y']
        Z0 = _tests_trajectories[0]['Z0']

        self.authors_info = u"Name:\t{}\nID:\t{}\nPlace:\t{}\nmd5:\t{}\n\n\nVelocity:\t{}\nA_x:\t{}\nA_y:\t{}\nZ0:\t{}\n\n\n".format(_author, _id, _university, _md5sum, velocity, A_x, A_y, Z0)

        sim.simxFinish(-1)
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        self.robotino_base_handle = None
        self.drone_base_handle = None

        self.start_robots = rospy.ServiceProxy('start_robots', SetBool)
        self.set_trajectory_parameters = rospy.ServiceProxy('set_trajectory_parameters', SetTrajectoryParameters)

        self.trajectory_sub = rospy.Subscriber("/robotino/trajectory", CartesianTrajectory, self.cart_trajectory_callback)
        self.robotino_odom_sub = rospy.Subscriber("/robotino/odom", Odometry, self.odometry_callback)

        self.err_sub = rospy.Subscriber("/err", Float32, self.err_callback)

        self.cart_trajectory = None
        self.odometry = None
        self.err = Float32(0)

        self.total_distance = 0.0
        self.laps = 1  # quantity of laps
        self.fails = 0  # quantity of fails for one lap
	print("Judge ok!")

    def __del__(self):
        if debug: print('Deleting judge...')
        sim.simxFinish(self.clientID)

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

    def err_callback(self, msg):
        lock.acquire()
        self.err = msg
        lock.release()

    def start_simulation(self):
        if debug: print('Simulation is starting...')
        resp = -1
        while resp != 0:#simx_opmode_blocking
            resp = sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot)
            time.sleep(0.1)
        if debug: print('Simulation started')

    def stop_simulation(self):
        if debug: print('Simulation is stoping...')
        resp = -1
        while resp != 0:
            resp = sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)
            time.sleep(0.1)
        if debug: print('Simulation stoped!')

    def initialize_scene(self):
        if debug: print("Getting objects...")
        err1 = -1
        err2 = -1
        while (err1 != 0) and (err2 != 0):
            err1, self.robotino_base_handle = sim.simxGetObjectHandle(self.clientID, ROBOTINO_NAME, sim.simx_opmode_blocking)
            err2, self.drone_base_handle = sim.simxGetObjectHandle(self.clientID, DRONE_NAME, sim.simx_opmode_blocking)
        if debug: print("Objects got!")

    def get_robotino_position(self):
        robotino_position = None
        returnCode = -1
        while returnCode != 0:
            returnCode, robotino_position = sim.simxGetObjectPosition(self.clientID, self.robotino_base_handle, -1,
                                                                      sim.simx_opmode_streaming)
            time.sleep(0.1)
        return robotino_position

    def get_drone_position(self):
        drone_position = None
        returnCode = -1
        while returnCode != 0:
            returnCode, drone_position = sim.simxGetObjectPosition(self.clientID, self.drone_base_handle, -1,
                                                                   sim.simx_opmode_streaming)
            time.sleep(0.1)
        return drone_position

    def run(self):
        """ Run solve testing """
        rate = rospy.Rate(RATE)

        log_string_buffer = self.authors_info
        log_string = ''

        "Loop for all test trajectories"
        for j, test in enumerate(tests_trajectories):

            self.total_distance = 0.0
            self.fails = 0
            self.laps = 1

            "Setup CoppeliaSim handles and simulation start"
            self.initialize_scene()
            self.start_simulation()

            "Setup robotino trajectory and drone altitude"
            print("Trajectory setup")
            rospy.wait_for_service('/set_trajectory_parameters')
            trajectory_srv_resp = self.set_trajectory_parameters(test['laps'],
                                                                 test['velocity'],
                                                                 test['A_x'],
                                                                 test['A_y'],
                                                                 test['phi'])
            Z0 = test['Z0']
            rospy.set_param('/drone_altitude', Z0)

            print("Time translation... Please wait...")
            ts_d = []
            while not rospy.is_shutdown():
                if self.cart_trajectory is not None:
                    for i, t in enumerate(self.cart_trajectory.time_from_start):
                        sys.stdout.write('\r')
                        sys.stdout.write("{}\t{}".format(i, len(self.cart_trajectory.time_from_start)))
                        sys.stdout.flush()

                        ts_d.append(t.to_sec())
                    break
                rospy.sleep(0.02)

            print("Runing your solution")
            "Start participant's solve"
            rospy.wait_for_service('/start_robots')
            self.start_robots(True)
            if debug: print('Robots started!')
            
            flag_2fail = True

            if trajectory_srv_resp.status:  # if trajectory set

                i = 0
                t = 0.0
                d_prev = [0.0, 0.0, 0.0]    # drone position in previous time
                N = len(ts_d)
                time_start = rospy.get_time()
                while not rospy.is_shutdown() and t < T_MAX and j < len(tests_trajectories):  # one attempt start
                    t = rospy.get_time() - time_start
                    
                        
		
                    "Get position of robots in CoppeliaSim scene"
                    r, d = self.get_robotino_position(), self.get_drone_position()

                    "Compute errors according task specification"
                    error_robotino_xy = self.err.data
                    error_drone_xy = numpy.sqrt((d[0] - r[0]) ** 2 + (d[1] - r[1]) ** 2)
                    error_drone_z = abs(Z0 - d[2])

                    "Compute accuracy flags"
                    IS_ROBOTINO_XY_ACCURATE = error_robotino_xy <= ERROR_ROBOTINO_XY_MAX
                    IS_DRONE_XY_ACCURATE = error_drone_xy <= ERROR_DRONE_XY_MAX
                    IS_DRONE_Z_ACCURATE = error_drone_z <= ERROR_DRONE_Z_MAX

                    if t > TRANSIENT_TIME:
                        if IS_ROBOTINO_XY_ACCURATE and IS_DRONE_XY_ACCURATE and IS_DRONE_Z_ACCURATE:
                            self.total_distance += numpy.sqrt((d[0] - d_prev[0]) ** 2 + (d[1] - d_prev[1]) ** 2)
                            if self.total_distance > self.laps * trajectory_srv_resp.length and self.laps > 0:
                                self.laps += 1
                                self.fails = 0
                                flag_2fail = True
                        else:
                            self.fails += 1
                        d_prev = d

                    log_string = log_string_buffer + "Test number: {}\n" \
                                                     "Total time: {:4.2f}\n" \
                                                     "Lap number: {}\n" \
                                                     "Total distance: {:4.2f}\n" \
                                                     "Robotino error (xy-plane): {:4.2f}\n" \
                                                     "Drone error (xy-plane): {:4.2f}\n" \
                                                     "Drone error (z): {:4.2f}\n" \
                                                     "Fails: {}\n"\
                        .format(j+1, t, self.laps, self.total_distance, error_robotino_xy, error_drone_xy, error_drone_z, self.fails)

                    os.system('clear')
                    sys.stdout.write('\r')
                    sys.stdout.write(log_string)
                    sys.stdout.flush()

                    if self.fails == 2 and flag_2fail:
                        self.laps -= 1
                        flag_2fail = False
                        rospy.sleep(1.0)    # wait a few seconds until the robots catch the trajectory again
                    if self.fails >= 3:
                        break

                    i = i + 1
                    
                    rate.sleep()

            else:
                print('Trajectory doesn\'t received. Check if everything is running correctly?')
            self.start_robots(False)
            if debug: print('Robots stoped!')
            self.stop_simulation()

            if self.fails >= 3:
                log_string_buffer = log_string_buffer + log_string + '\nTest failed!\n\n'
            else:
                log_string_buffer = log_string_buffer + log_string + '\nTest succeeded!\n\n'

        print("\nJudge ended!")


if __name__ == '__main__':
    rospy.init_node("judge_node")

    readable_hash = ''
    filename = '/home/root/catkin_ws/src/bac_task/src/main_solve.py'
    with open(filename,"rb") as f:
        bytes = f.read() # read file as bytes
        readable_hash = hashlib.md5(bytes).hexdigest();

    judge = None
    while not rospy.is_shutdown():
        try:
            judge = Judge(main_solve.__author__, main_solve.__id__, main_solve.__university__, str(readable_hash), tests_trajectories)
            judge.run()
        except rospy.ROSInterruptException as e:
            print('Restart judge_node with new ros-sim-time')
        finally:
            judge.stop_simulation()
            del judge
            break
