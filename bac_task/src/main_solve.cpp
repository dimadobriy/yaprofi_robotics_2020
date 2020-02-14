/*
__author__ = "Пупкин Василий Бенедиктович"
__id__ = "123456789"
__university = "ITMO University"
*/

#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <bac_task/CartesianTrajectory.h>
#include <bac_task/CameraFeatures.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

using namespace Eigen;


class RobotinoController {

		bac_task::CartesianTrajectory cart_trajectory;
		nav_msgs::Odometry odometry;

		ros::NodeHandle nh;
		ros::Subscriber trajectory_sub;
		ros::Subscriber odom_sub;
		ros::Publisher robotino_cmd_vel_pub;

public:

		RobotinoController() {
			trajectory_sub = nh.subscribe("/robotino/trajectory", 10, &RobotinoController::cart_trajectory_callback, this);
			odom_sub = nh.subscribe("/robotino/odom", 10, &RobotinoController::odometry_callback, this);
			robotino_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robotino/cmd_vel", 1);
		}

		~RobotinoController() {}

		void cart_trajectory_callback(const bac_task::CartesianTrajectory &msg) {
			cart_trajectory = msg;
		}

		void odometry_callback(const nav_msgs::Odometry &msg) {
			odometry = msg;
		}

		void update(double dt, double t) {
			/*
			 * TODO WRITE YOUR CODE HERE
			 * */
			geometry_msgs::Twist msg;
			msg.angular.z = 0;
			msg.linear.x = 0;
			msg.linear.y = 0;
			robotino_cmd_vel_pub.publish(msg);
		}

};

class DroneController {

		double Z0;

		// Camera parameters
		double width = 640;
		double height = 480;
		double f = 0.002;
		double px = 600;
		double py = 600;
		double u0 = 320;
		double v0 = 240;

		ros::NodeHandle nh;
		ros::Subscriber marker_features_sub;
		ros::Subscriber drone_odom_sub;
		ros::Publisher drone_target_pose_pub;

		// drone state
		geometry_msgs::Pose drone_state_pose;

		// Camera features for feedback"
		bac_task::CameraFeatures camera_features;

public:

		DroneController() {
		    Z0 = nh.param<double>('/drone_altitude', 2.0);
		    
			marker_features_sub = nh.subscribe("/marker/features", 10, &DroneController::marker_features_callback, this);
			drone_odom_sub = nh.subscribe("/drone/odom", 10, &DroneController::drone_odom_callback, this);
			drone_target_pose_pub = nh.advertise<geometry_msgs::Pose>("/drone/target_pose", 1);
		}

		~DroneController() {}

		void marker_features_callback(const bac_task::CameraFeatures  &msg) {
			camera_features = msg;
		}

		void drone_odom_callback(const geometry_msgs::Pose &msg) {
			drone_state_pose = msg;
		}

		void update(double dt, double t) {
			/*
			 * TODO WRITE YOUR CODE HERE
			 * */
			geometry_msgs::Pose msg;
			msg.position.x = 0;
			msg.position.y = 0;
			msg.position.z = Z0;
			drone_target_pose_pub.publish(msg);
		}

};


int main(int argc, char **argv) {
	ros::init(argc, argv, "main_solve_node");
	ros::NodeHandle nh;

	int RATE = 50;
	ros::Rate rate(RATE);

	DroneController drone_controller;
	RobotinoController robotino_controller;

	ROS_INFO("Start task!");
	double time_prev = 0.0;
	double time_start = ros::Time::now().toSec();
	while (nh.ok()) {
		double t = ros::Time::now().toSec() - time_start;
		//double dt = t - time_prev;
		double dt = 0.02; //simulation delta t
		time_prev = t;

		drone_controller.update(dt, t);
		robotino_controller.update(dt, t);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
