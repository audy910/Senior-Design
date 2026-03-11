// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_project/msg/detail/waypoint_list__rosidl_typesupport_introspection_c.h"
#include "rover_project/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_project/msg/detail/waypoint_list__functions.h"
#include "rover_project/msg/detail/waypoint_list__struct.h"


// Include directives for member types
// Member `latitudes`
// Member `longitudes`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_project__msg__WaypointList__init(message_memory);
}

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_fini_function(void * message_memory)
{
  rover_project__msg__WaypointList__fini(message_memory);
}

size_t rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__size_function__WaypointList__latitudes(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__latitudes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__latitudes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__fetch_function__WaypointList__latitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__latitudes(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__assign_function__WaypointList__latitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__latitudes(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__resize_function__WaypointList__latitudes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__size_function__WaypointList__longitudes(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__longitudes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__longitudes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__fetch_function__WaypointList__longitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__longitudes(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__assign_function__WaypointList__longitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__longitudes(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__resize_function__WaypointList__longitudes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_member_array[4] = {
  {
    "latitudes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__WaypointList, latitudes),  // bytes offset in struct
    NULL,  // default value
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__size_function__WaypointList__latitudes,  // size() function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__latitudes,  // get_const(index) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__latitudes,  // get(index) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__fetch_function__WaypointList__latitudes,  // fetch(index, &value) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__assign_function__WaypointList__latitudes,  // assign(index, value) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__resize_function__WaypointList__latitudes  // resize(index) function pointer
  },
  {
    "longitudes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__WaypointList, longitudes),  // bytes offset in struct
    NULL,  // default value
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__size_function__WaypointList__longitudes,  // size() function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_const_function__WaypointList__longitudes,  // get_const(index) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__get_function__WaypointList__longitudes,  // get(index) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__fetch_function__WaypointList__longitudes,  // fetch(index, &value) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__assign_function__WaypointList__longitudes,  // assign(index, value) function pointer
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__resize_function__WaypointList__longitudes  // resize(index) function pointer
  },
  {
    "total_waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__WaypointList, total_waypoints),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__WaypointList, current_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_members = {
  "rover_project__msg",  // message namespace
  "WaypointList",  // message name
  4,  // number of fields
  sizeof(rover_project__msg__WaypointList),
  rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_member_array,  // message members
  rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_type_support_handle = {
  0,
  &rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_project
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_project, msg, WaypointList)() {
  if (!rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_type_support_handle.typesupport_identifier) {
    rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_project__msg__WaypointList__rosidl_typesupport_introspection_c__WaypointList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
