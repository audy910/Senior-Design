// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_
#define ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'latitudes'
// Member 'longitudes'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/WaypointList in the package rover_project.
/**
  * List of waypoints forming a navigation path
 */
typedef struct rover_project__msg__WaypointList
{
  /// Array of waypoint latitudes
  rosidl_runtime_c__double__Sequence latitudes;
  /// Array of waypoint longitudes
  rosidl_runtime_c__double__Sequence longitudes;
  /// Total number of waypoints in the path
  uint32_t total_waypoints;
  /// Index of the current target waypoint
  uint32_t current_index;
} rover_project__msg__WaypointList;

// Struct for a sequence of rover_project__msg__WaypointList.
typedef struct rover_project__msg__WaypointList__Sequence
{
  rover_project__msg__WaypointList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_project__msg__WaypointList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_H_
