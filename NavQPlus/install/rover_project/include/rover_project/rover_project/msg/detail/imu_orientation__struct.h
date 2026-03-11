// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_project:msg/ImuOrientation.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_H_
#define ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ImuOrientation in the package rover_project.
/**
  * IMU orientation message with compass and calibration data
 */
typedef struct rover_project__msg__ImuOrientation
{
  /// Compass heading in degrees (0-360, 0=North)
  float heading;
  /// Pitch angle in degrees (-180 to 180)
  float pitch;
  /// Roll angle in degrees (-180 to 180)
  float roll;
  /// System calibration status (0-3, 3=fully calibrated)
  uint8_t cal_sys;
  /// Magnetometer calibration status (0-3, 3=fully calibrated)
  uint8_t cal_mag;
} rover_project__msg__ImuOrientation;

// Struct for a sequence of rover_project__msg__ImuOrientation.
typedef struct rover_project__msg__ImuOrientation__Sequence
{
  rover_project__msg__ImuOrientation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_project__msg__ImuOrientation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_H_
