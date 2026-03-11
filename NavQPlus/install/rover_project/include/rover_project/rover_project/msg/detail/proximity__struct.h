// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_H_
#define ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Proximity in the package rover_project.
typedef struct rover_project__msg__Proximity
{
  uint16_t proximity_front;
  uint16_t proximity_rear;
  uint16_t proximity_cliff;
  bool cliff_detected;
  bool front_valid;
  bool rear_valid;
  bool cliff_valid;
} rover_project__msg__Proximity;

// Struct for a sequence of rover_project__msg__Proximity.
typedef struct rover_project__msg__Proximity__Sequence
{
  rover_project__msg__Proximity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_project__msg__Proximity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_H_
