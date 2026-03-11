// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_project:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_H_
#define ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GpsFix in the package rover_project.
/**
  * GPS fix message with comprehensive GPS state
 */
typedef struct rover_project__msg__GpsFix
{
  /// Latitude in degrees (WGS84)
  double latitude;
  /// Longitude in degrees (WGS84)
  double longitude;
  /// Speed over ground in m/s
  float speed;
  /// Course over ground in degrees (0-360)
  float course;
  /// Fix type: 0=none, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+RTK, 5=RTK float
  uint8_t fix_type;
  /// Number of satellites used
  uint8_t num_sats;
  /// Horizontal dilution of precision
  float hdop;
  /// Horizontal accuracy estimate in meters
  float h_acc;
  /// Vertical accuracy estimate in meters
  float v_acc;
} rover_project__msg__GpsFix;

// Struct for a sequence of rover_project__msg__GpsFix.
typedef struct rover_project__msg__GpsFix__Sequence
{
  rover_project__msg__GpsFix * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_project__msg__GpsFix__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_H_
