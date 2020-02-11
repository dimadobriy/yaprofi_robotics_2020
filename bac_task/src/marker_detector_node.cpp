#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <bac_task/CameraFeatures.h>

#include <tf/transform_broadcaster.h>

#include <visp_bridge/image.h>
#include <visp/vpImageTools.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpPolygon.h>


class MarkerDetectorNode {

		int RATE;
		bool debug;   // if true: it shows camera image with features
		bool isImageReceived;

		std::string camera_name;
		std::string camera_config_file;

		ros::NodeHandle nh;
		ros::Subscriber imageSub;
		ros::Publisher markerPosePub;
		ros::Publisher markerFeaturesPub;

		vpImage<unsigned char> I;
		vpDisplayX d;

		vpDetectorAprilTag::vpAprilTagFamily tagFamily;
		vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod;

		vpCameraParameters cam;
		vpHomogeneousMatrix dMc;  // transformation from drone frame to camera frame

		// marker parameters
		double tagSize = 0.24;
		bool display_tag = true;
		unsigned int thickness = 2;
		bool align_frame = false;

public:

		/* Empty constructor */
		MarkerDetectorNode() : nh("~") {

			isImageReceived = false;

			/* Read parameters from ROS param-server */
			nh.param<int>("/rate", RATE, 50);
			nh.param<bool>("/debug", debug, true);
			nh.param<std::string>("/camera_name", camera_name, "floor_camera");
			nh.param<std::string>("/camera_config_file", camera_config_file, "/home/kika/catkin_ws/src/bac_task/config/floor_camera.xml");

			/* Initialize AprilTag parameters (HARD CODED) */
			tagFamily = vpDetectorAprilTag::TAG_36h11;  // !!!
			poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

			if (!camera_config_file.empty() && !camera_name.empty()) {
				vpXmlParserCamera parser;
				parser.parse(cam, camera_config_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
			} else {
				std::cout << "Using hard coded parameters for your camera" << std::endl;
				double px = 600;
				double py = 600;
				double u0 = 320;
				double v0 = 240;
				cam.initPersProjWithoutDistortion(px, py, u0, v0);
			}

			std::cout << cam << std::endl;
			std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
			std::cout << "tagFamily: " << tagFamily << std::endl;

			// Build drone -> camera transform. Hard coded
			dMc.buildFrom(vpTranslationVector(0, 0, 0), vpRotationMatrix(vpRxyzVector(M_PI, 0, M_PI)));


			/* Initialize ROS entities */
			imageSub = nh.subscribe("/drone/image_raw", 1, &MarkerDetectorNode::image_callback, this);
			markerFeaturesPub = nh.advertise<bac_task::CameraFeatures>("/marker/features", 1, true); // in image frame of the dron's camera
			// markerPosePub = nh.advertise<geometry_msgs::Pose>("/marker_pose", 1, true); // in the drone frame

			ROS_INFO("marker_detector_node started");
		}

		/* Destructor */
		~MarkerDetectorNode() {}

		/* Receives an image from CoppeliaSim and convert it to VISP image type */
		void image_callback(const sensor_msgs::Image::ConstPtr &msg) {
			I = visp_bridge::toVispImage(*msg);
			vpImageTools::flip(I);  // flip vertical, because in CoppeliaSim little-endian image format doesn't work in my case
			isImageReceived = true; // start main loop
		}

		/* Extracts translation and rotation (roll, pitch, yaw) vectors from homogeneous matrix */
		void extract(vpHomogeneousMatrix M, vpTranslationVector &tr, vpQuaternionVector &qt) {
			M.extract(qt);
			M.extract(tr);
		}

		/* Waits while all instances will be initialized and simulation started*/
		void wait_for_image() {
			while (ros::ok()) {
				if (isImageReceived)
					return;
				ros::spinOnce();
			}
		}

		/* The main ROS loop with big deals */
		void spin() {

			wait_for_image();

			if (debug) {
				d.init(I);
				vpDisplay::setTitle(I, "Marker_detector debug display");
			}

			/* Create and setup AprilTag detector */
			vpDetectorAprilTag detector(tagFamily);

			detector.setAprilTagQuadDecimate(1.0);  // more value -> faster detection (lower accuracy)
			detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
			detector.setAprilTagNbThreads(1);
			detector.setDisplayTag(display_tag, vpColor::getColor(1), thickness);
			detector.setZAlignedWithCameraAxis(align_frame);

			/* Start node */
			ros::Rate R(RATE);
			while (nh.ok()) {

				/* Marker detection */
				std::vector<vpHomogeneousMatrix> cMo_vec;  // vector of one transformation matrix between camera and marker
				detector.detect(I, tagSize, cam, cMo_vec);

				if (detector.getNbObjects() > 0) {  // if TAG detected!

					/* Drone -> Camera transform */
					vpTranslationVector camera_tr;
					vpQuaternionVector camera_qt;
					extract(dMc, camera_tr, camera_qt);

					/* Camera -> Marker transform */
					vpTranslationVector marker_tr;
					vpQuaternionVector marker_qt;
					extract(cMo_vec[0], marker_tr, marker_qt);

					/* Add tf frames */
					static tf::TransformBroadcaster br;
					tf::Transform transform;

					// Add camera frame
					transform.setOrigin(tf::Vector3(camera_tr[0], camera_tr[1], camera_tr[2]));
					tf::Quaternion qt_camera(camera_qt.x(), camera_qt.y(), camera_qt.z(), camera_qt.w());
					transform.setRotation(qt_camera);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "drone", "camera"));

					// Add marker frame
					transform.setOrigin(tf::Vector3(marker_tr[0], marker_tr[1], marker_tr[2]));
					tf::Quaternion qt_marker(marker_qt.x(), marker_qt.y(), marker_qt.z(), marker_qt.w());
					transform.setRotation(qt_marker);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "marker"));

					/* Compute area of marker and get COG */
					/* Publish features */
					//std::vector<vpImagePoint> marker_corners = detector.getPolygon(0);
					vpPolygon polygon(detector.getPolygon(0));
					vpImagePoint marker_cog = detector.getCog(0);

					// marker position relative of image frame center
					double dx = cam.get_u0() - marker_cog.get_u();
					double dy = cam.get_v0() - marker_cog.get_v();

					bac_task::CameraFeatures camera_features;
					camera_features.rho = sqrt(dx * dx + dy * dy);
					camera_features.theta = atan2(dy, dx);  // !!! check for every quaters
					camera_features.cog_x = marker_cog.get_u();
					camera_features.cog_y = marker_cog.get_v();
					camera_features.area = polygon.getArea();
					camera_features.Z = marker_tr[2];
					markerFeaturesPub.publish(camera_features);

					if (debug) {  // show camera image using visp tools
						vpDisplay::display(I);

						vpDisplay::displayPolygon(I, polygon.getCorners(), vpColor::orange, 2);
						vpDisplay::displayCross(I, marker_cog, 25, vpColor::orange, 5);

						// center of image frame (cross lines)
						vpDisplay::displayLine(I, 0, cam.get_u0(), I.getHeight() - 1, cam.get_u0(), vpColor::red, 1);
						vpDisplay::displayLine(I, cam.get_v0(), 0, cam.get_v0(), I.getWidth() - 1, vpColor::red, 1);

						// draw marker's frame
						vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);

						vpDisplay::flush(I);
					}
				}
				ros::spinOnce();
				R.sleep();
			}
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "marker_detector_node");

    ros::NodeHandle nh;
    while (nh.ok()) {
        try {
            MarkerDetectorNode markerDetectorNode;
            markerDetectorNode.spin();
        } catch (int e){
            std::cout << "Restarted marker_detector_node with new ros-sim-time" << std::endl;
        }
        ros::spinOnce();
    }
	return 0;
}
