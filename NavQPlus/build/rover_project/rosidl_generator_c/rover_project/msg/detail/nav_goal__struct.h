// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_project:msg/NavGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_H_
#define ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NavGoal in the package rover_project.
/**
  * Navigation goal coordinates
 */
typedef struct rover_project__msg__NavGoal
{
  /// Target latitude in degrees (WGS84)
  double latitude;
  /// Target longitude in degrees (WGS84)
  double longitude;
} rover_project__msg__NavGoal;

// Struct for a sequence of rover_project__msg__NavGoal.
typedef struct rover_project__msg__NavGoal__Sequence
{
  rover_project__msg__NavGoal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_project__msg__NavGoal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_H_
