
#include "py_bind.h"

typedef struct {
  int two;              /* contains the integer 2 -- simple sanity check */
  int nd;               /* number of dimensions */
  char typekind;        /* kind in array --- character code of typestr */
  int itemsize;         /* size of each element */
  int flags;            /* flags indicating how the data should be interpreted */
                        /*   must set ARR_HAS_DESCR bit to validate descr */
  Py_intptr_t *shape;   /* A length-nd array of shape information */
  Py_intptr_t *strides; /* A length-nd array of stride information */
  void *data;           /* A pointer to the first element of the array */
  PyObject *descr;      /* NULL or data-description (same as descr key
                                of __array_interface__) -- must set ARR_HAS_DESCR
                                flag or this will be ignored. */
} PyArrayInterface;


UR::UR(int port) 
{
  pAux = pUr = pPtp = pLin = pGripper = NULL;
  Py_Initialize(); 
  // add local directory to path
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\".\")");

  //PyObject *pName = PyUnicode_DecodeFSDefault("ur_aux");  // python 3.5
  PyObject *pName = PyString_FromString("ur_aux_cpp");  // python 2.7
  pPtp = PyString_FromString("ptp");
  pLin = PyString_FromString("lin");
  pGripper = PyString_FromString("gripperOpen");

  // import module
  if((pAux = PyImport_Import(pName)) != NULL) {
    // get elements
    PyObject *dict = PyModule_GetDict(pAux);
    if(dict != NULL) {
      // get class reference
      PyObject *obj = PyDict_GetItemString(dict,"VrepModel");
      if(obj != NULL) {
        // create class
        PyObject* arg = PyTuple_New(1);
        PyTuple_SetItem(arg,0,PyInt_FromLong(port));
        if((pUr = PyObject_CallObject(obj,arg)) == NULL) {
          PyErr_Print();
        } else {
          std::cout << "UR object is initialized" << std::endl;
        }
      } else {
        PyErr_Print();
      }
    } else {
      PyErr_Print();
    }
  } else {
    PyErr_Print();
  }

  Py_DECREF(pName);
}

UR::~UR()
{
  Py_DECREF(pAux);
  Py_DECREF(pUr);
  Py_DECREF(pPtp);
  Py_DECREF(pLin);
  Py_DECREF(pGripper);
  Py_Finalize();
}

bool UR::startSimulation()
{
  return PyObject_CallMethod(pUr,"startSimulation",NULL) != NULL;
}

bool UR::stopSimulation()
{
  return PyObject_CallMethod(pUr,"stopSimulation",NULL) != NULL;
}

bool UR::check()
{
  return PyObject_CallMethod(pUr,"check",NULL) != NULL;
}

bool UR::ptp(double *q)
{
  PyObject *args = PyTuple_New(JOINT_NO);
  for(int i = 0; i < JOINT_NO; i++) {
    PyTuple_SetItem(args,i,PyFloat_FromDouble(q[i]));
  }
  PyObject *v = PyObject_CallMethodObjArgs(pUr,pPtp,args,NULL);
  Py_DECREF(args);
  return v != NULL;
}

bool UR::lin(double *pos)
{
  PyObject *p = PyList_New(CART_NO);
  for(int i = 0; i < CART_NO; i++) {
    PyList_SetItem(p,i,PyFloat_FromDouble(pos[i]));
  }
  PyObject *v = PyObject_CallMethodObjArgs(pUr,pLin,p,NULL);
  Py_DECREF(p);
  return v != NULL;
}

bool UR::lin(double *pos, double *orient)
{
  PyObject *p = PyList_New(CART_NO);
  PyObject *o = PyList_New(CART_NO);
  for(int i = 0; i < CART_NO; i++) {
    PyList_SetItem(p,i,PyFloat_FromDouble(pos[i]));
    PyList_SetItem(o,i,PyFloat_FromDouble(orient[i]));
  }
  PyObject *v = PyObject_CallMethodObjArgs(pUr,pLin,p,o,NULL);
  Py_DECREF(p);
  Py_DECREF(o);
  return v != NULL;
}

bool UR::gripperOpen(bool flag)
{
  PyObject *v = PyObject_CallMethodObjArgs(pUr,pGripper,PyBool_FromLong(flag ? 1 : 0),NULL);
  return v != NULL;
}

bool UR::getJointPosition(double *dst)
{
  PyObject* v = PyObject_CallMethod(pUr,"getJointPosition",NULL);
  if(v != NULL) {
    for(int i = 0; i < JOINT_NO; i++) {
      dst[i] = PyFloat_AsDouble(PyList_GetItem(v,i));
    }
    return true;
  }
  return false;
}

bool UR::getToolPosition(double *dst)
{
  PyObject* v = PyObject_CallMethod(pUr,"getToolPosition",NULL);
  if(v != NULL) {
    for(int i = 0; i < CART_NO; i++) {
      dst[i] = PyFloat_AsDouble(PyList_GetItem(v,i));
    }
    return true;
  }
  return false;
}

bool UR::getToolOrientation(double *dst)
{
  PyObject* v = PyObject_CallMethod(pUr,"getToolOrientation",NULL);
  if(v != NULL) {
    for(int i = 0; i < CART_NO; i++) {
      dst[i] = PyFloat_AsDouble(PyList_GetItem(v,i));
    }
    return true;
  }
  return false;
}

bool UR::getForceTorque(double *f, double *t)
{
  PyObject* v = PyObject_CallMethod(pUr,"getForceTorque",NULL);
  if(v != NULL) {
    PyObject *a1 = PyTuple_GetItem(v,0);
    PyObject *a2 = PyTuple_GetItem(v,1);
    for(int i = 0; i < CART_NO; i++) {
      f[i] = PyFloat_AsDouble(PyList_GetItem(a1,i));
      t[i] = PyFloat_AsDouble(PyList_GetItem(a2,i));
    }
    return true;
  }
  return false;
}



bool UR::getImage(ImgData *img)
{
  PyObject* v = PyObject_CallMethod(pUr,"getImage",NULL);
  if(v != NULL) {
    PyObject *ao = PyObject_GetAttrString(v, "__array_struct__");
    PyArrayInterface *pai = (PyArrayInterface*) PyCObject_AsVoidPtr(ao);
    img->buffer = (char*) pai->data;
    img->width = pai->shape[1];
    img->height = pai->shape[0];
    img->size = pai->strides[0] * pai->shape[0];
    
    return true;
  }
  return false;
}
