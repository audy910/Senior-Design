// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_project/msg/detail/proximity__rosidl_typesupport_introspection_c.h"
#include "rover_project/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_project/msg/detail/proximity__functions.h"
#include "rover_project/msg/detail/proximity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_project__msg__Proximity__init(message_memory);
}

void rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_fini_function(void * message_memory)
{
  rover_project__msg__Proximity__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_member_array[7] = {
  {
    "proximity_front",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, proximity_front),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "proximity_rear",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, proximity_rear),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "proximity_cliff",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, proximity_cliff),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cliff_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, cliff_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "front_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, front_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rear_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, rear_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cliff_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_project__msg__Proximity, cliff_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_members = {
  "rover_project__msg",  // message namespace
  "Proximity",  // message name
  7,  // number of fields
  sizeof(rover_project__msg__Proximity),
  rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_member_array,  // message members
  rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_type_support_handle = {
  0,
  &rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_project
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_project, msg, Proximity)() {
  if (!rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_type_support_handle.typesupport_identifier) {
    rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_project__msg__Proximity__rosidl_typesupport_introspection_c__Proximity_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
