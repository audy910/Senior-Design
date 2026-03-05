// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice
#include "rover_project/msg/detail/proximity__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rover_project/msg/detail/proximity__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rover_project
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_project
cdr_serialize(
  const rover_project::msg::Proximity & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: proximity_front
  cdr << ros_message.proximity_front;
  // Member: proximity_rear
  cdr << ros_message.proximity_rear;
  // Member: proximity_cliff
  cdr << ros_message.proximity_cliff;
  // Member: cliff_detected
  cdr << (ros_message.cliff_detected ? true : false);
  // Member: front_valid
  cdr << (ros_message.front_valid ? true : false);
  // Member: rear_valid
  cdr << (ros_message.rear_valid ? true : false);
  // Member: cliff_valid
  cdr << (ros_message.cliff_valid ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_project
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rover_project::msg::Proximity & ros_message)
{
  // Member: proximity_front
  cdr >> ros_message.proximity_front;

  // Member: proximity_rear
  cdr >> ros_message.proximity_rear;

  // Member: proximity_cliff
  cdr >> ros_message.proximity_cliff;

  // Member: cliff_detected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.cliff_detected = tmp ? true : false;
  }

  // Member: front_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.front_valid = tmp ? true : false;
  }

  // Member: rear_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rear_valid = tmp ? true : false;
  }

  // Member: cliff_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.cliff_valid = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_project
get_serialized_size(
  const rover_project::msg::Proximity & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: proximity_front
  {
    size_t item_size = sizeof(ros_message.proximity_front);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: proximity_rear
  {
    size_t item_size = sizeof(ros_message.proximity_rear);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: proximity_cliff
  {
    size_t item_size = sizeof(ros_message.proximity_cliff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cliff_detected
  {
    size_t item_size = sizeof(ros_message.cliff_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_valid
  {
    size_t item_size = sizeof(ros_message.front_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_valid
  {
    size_t item_size = sizeof(ros_message.rear_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cliff_valid
  {
    size_t item_size = sizeof(ros_message.cliff_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_project
max_serialized_size_Proximity(
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


  // Member: proximity_front
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: proximity_rear
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: proximity_cliff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: cliff_detected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: front_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rear_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cliff_valid
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
    using DataType = rover_project::msg::Proximity;
    is_plain =
      (
      offsetof(DataType, cliff_valid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Proximity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rover_project::msg::Proximity *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Proximity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rover_project::msg::Proximity *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Proximity__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rover_project::msg::Proximity *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Proximity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Proximity(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Proximity__callbacks = {
  "rover_project::msg",
  "Proximity",
  _Proximity__cdr_serialize,
  _Proximity__cdr_deserialize,
  _Proximity__get_serialized_size,
  _Proximity__max_serialized_size
};

static rosidl_message_type_support_t _Proximity__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Proximity__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rover_project

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rover_project
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_project::msg::Proximity>()
{
  return &rover_project::msg::typesupport_fastrtps_cpp::_Proximity__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_project, msg, Proximity)() {
  return &rover_project::msg::typesupport_fastrtps_cpp::_Proximity__handle;
}

#ifdef __cplusplus
}
#endif
