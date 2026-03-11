// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rover_project/msg/detail/proximity__struct.h"
#include "rover_project/msg/detail/proximity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rover_project__msg__proximity__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rover_project.msg._proximity.Proximity", full_classname_dest, 38) == 0);
  }
  rover_project__msg__Proximity * ros_message = _ros_message;
  {  // proximity_front
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity_front");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->proximity_front = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // proximity_rear
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity_rear");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->proximity_rear = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // proximity_cliff
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity_cliff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->proximity_cliff = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cliff_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "cliff_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cliff_detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // front_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->front_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rear_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rear_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cliff_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "cliff_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cliff_valid = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rover_project__msg__proximity__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Proximity */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rover_project.msg._proximity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Proximity");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rover_project__msg__Proximity * ros_message = (rover_project__msg__Proximity *)raw_ros_message;
  {  // proximity_front
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->proximity_front);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity_front", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proximity_rear
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->proximity_rear);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity_rear", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proximity_cliff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->proximity_cliff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity_cliff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cliff_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cliff_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cliff_detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->front_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rear_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cliff_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cliff_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cliff_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
