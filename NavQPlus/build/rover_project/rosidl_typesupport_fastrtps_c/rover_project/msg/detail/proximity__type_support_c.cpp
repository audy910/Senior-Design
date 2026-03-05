// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice
#include "rover_project/msg/detail/proximity__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_project/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_project/msg/detail/proximity__struct.h"
#include "rover_project/msg/detail/proximity__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Proximity__ros_msg_type = rover_project__msg__Proximity;

static bool _Proximity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Proximity__ros_msg_type * ros_message = static_cast<const _Proximity__ros_msg_type *>(untyped_ros_message);
  // Field name: proximity_front
  {
    cdr << ros_message->proximity_front;
  }

  // Field name: proximity_rear
  {
    cdr << ros_message->proximity_rear;
  }

  // Field name: proximity_cliff
  {
    cdr << ros_message->proximity_cliff;
  }

  // Field name: cliff_detected
  {
    cdr << (ros_message->cliff_detected ? true : false);
  }

  // Field name: front_valid
  {
    cdr << (ros_message->front_valid ? true : false);
  }

  // Field name: rear_valid
  {
    cdr << (ros_message->rear_valid ? true : false);
  }

  // Field name: cliff_valid
  {
    cdr << (ros_message->cliff_valid ? true : false);
  }

  return true;
}

static bool _Proximity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Proximity__ros_msg_type * ros_message = static_cast<_Proximity__ros_msg_type *>(untyped_ros_message);
  // Field name: proximity_front
  {
    cdr >> ros_message->proximity_front;
  }

  // Field name: proximity_rear
  {
    cdr >> ros_message->proximity_rear;
  }

  // Field name: proximity_cliff
  {
    cdr >> ros_message->proximity_cliff;
  }

  // Field name: cliff_detected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->cliff_detected = tmp ? true : false;
  }

  // Field name: front_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->front_valid = tmp ? true : false;
  }

  // Field name: rear_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rear_valid = tmp ? true : false;
  }

  // Field name: cliff_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->cliff_valid = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_project
size_t get_serialized_size_rover_project__msg__Proximity(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Proximity__ros_msg_type * ros_message = static_cast<const _Proximity__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name proximity_front
  {
    size_t item_size = sizeof(ros_message->proximity_front);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name proximity_rear
  {
    size_t item_size = sizeof(ros_message->proximity_rear);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name proximity_cliff
  {
    size_t item_size = sizeof(ros_message->proximity_cliff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cliff_detected
  {
    size_t item_size = sizeof(ros_message->cliff_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name front_valid
  {
    size_t item_size = sizeof(ros_message->front_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rear_valid
  {
    size_t item_size = sizeof(ros_message->rear_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cliff_valid
  {
    size_t item_size = sizeof(ros_message->cliff_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Proximity__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_project__msg__Proximity(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_project
size_t max_serialized_size_rover_project__msg__Proximity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: proximity_front
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: proximity_rear
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: proximity_cliff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: cliff_detected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: front_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rear_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cliff_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_project__msg__Proximity;
    is_plain =
      (
      offsetof(DataType, cliff_valid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Proximity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_project__msg__Proximity(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Proximity = {
  "rover_project::msg",
  "Proximity",
  _Proximity__cdr_serialize,
  _Proximity__cdr_deserialize,
  _Proximity__get_serialized_size,
  _Proximity__max_serialized_size
};

static rosidl_message_type_support_t _Proximity__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Proximity,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_project, msg, Proximity)() {
  return &_Proximity__type_support;
}

#if defined(__cplusplus)
}
#endif
