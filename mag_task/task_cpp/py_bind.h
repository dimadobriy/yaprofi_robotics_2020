#ifndef PY_BIND_H
#define PY_BIND_H

#include <python2.7/Python.h>

#include <iostream>

#define JOINT_NO 6
#define CART_NO 3

// Image structure
struct ImgData {
  char* buffer;
  int width;
  int height;
  int size;
};


class UR {
public:
  UR(int port);
  ~UR();
  // execution control
  bool startSimulation();
  bool stopSimulation();
  // motion control
  bool ptp(double *q);                        // q[6]
  bool lin(double *pos);                      // pos[3] 
  bool lin(double *pos, double *orient);      // pos[3], orient[3]
  // tool control
  bool gripperOpen(bool flag);                // true / false
  // get information
  bool getJointPosition(double *dst);         // dst[6]
  bool getToolPosition(double *dst);          // dst[3]
  bool getToolOrientation(double *dst);       // dst[3]
  bool getForceTorque(double *f, double *t);  // f[3], t[3]
  // read image
  bool getImage(ImgData *img); 
  // testing            
  bool check();
  
private:
  PyObject *pAux, *pUr;
  PyObject *pPtp, *pLin, *pGripper;
};


#endif // PY_BIND_H
