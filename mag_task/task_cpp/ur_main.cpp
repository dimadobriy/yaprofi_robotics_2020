// Write your code here

#include "py_bind.h"
#include <opencv2/opencv.hpp>

#define RAD(X) (3.1415926*X/180)

int main(int argc, char** argv) 
{
  /************ Start simulation ******************/ 
  UR ur(19997);          // connection port number
  ur.startSimulation();

  /********* Write your solution *****************/
  
  // point to point motion
  double q0[] = {0,RAD(20),RAD(-75),RAD(-35),RAD(90),0};
  ur.ptp(q0);

  double q1[JOINT_NO];
  ur.getJointPosition(q1);
  // check
  std::cout << q1[1] << " " << q1[3] << " " << q1[5] << std::endl;

  // tool 
  ur.gripperOpen(true);

  // cartesian motion
  double p[CART_NO];
  double o[CART_NO];

  ur.getToolPosition(p);
  ur.getToolOrientation(o);
  // check
  std::cout << "Position " << p[0] << " " << p[1] << " " << p[2] << std::endl;
  std::cout << "Orientation " << o[0] << " " << o[1] << " " << o[2] << std::endl;

  p[1] -= 0.1;
  p[0] += 0.1;
  ur.lin(p);

  o[2] -= 0.1;
  ur.lin(p,o);
  
  // read force and torque
  double force[CART_NO];
  double torque[CART_NO];
  ur.getForceTorque(force,torque);
  // check
  std::cout << force[0] << " " << torque[0] << std::endl;
  
  // get image
  ImgData data;
  ur.getImage(&data);
  cv::Mat img = cv::Mat(data.height, data.width, CV_8UC3, data.buffer, cv::Mat::AUTO_STEP);
  cv::imshow("image",img);
  cv::waitKey(0);

  /******************** Finish ***************************/
  double qend[] = {0,0,0,0,0,0};
  ur.ptp(qend);
  ur.check();
  ur.stopSimulation();
  
  return 0;
}
