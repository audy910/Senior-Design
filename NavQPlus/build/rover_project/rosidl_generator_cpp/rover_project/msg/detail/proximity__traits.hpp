// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__PROXIMITY__TRAITS_HPP_
#define ROVER_PROJECT__MSG__DETAIL__PROXIMITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_project/msg/detail/proximity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_project
{

namespace msg
{

inline void to_flow_style_yaml(
  const Proximity & msg,
  std::ostream & out)
{
  out << "{";
  // member: proximity_front
  {
    out << "proximity_front: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_front, out);
    out << ", ";
  }

  // member: proximity_rear
  {
    out << "proximity_rear: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_rear, out);
    out << ", ";
  }

  // member: proximity_cliff
  {
    out << "proximity_cliff: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_cliff, out);
    out << ", ";
  }

  // member: cliff_detected
  {
    out << "cliff_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff_detected, out);
    out << ", ";
  }

  // member: front_valid
  {
    out << "front_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.front_valid, out);
    out << ", ";
  }

  // member: rear_valid
  {
    out << "rear_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_valid, out);
    out << ", ";
  }

  // member: cliff_valid
  {
    out << "cliff_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff_valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Proximity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: proximity_front
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity_front: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_front, out);
    out << "\n";
  }

  // member: proximity_rear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity_rear: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_rear, out);
    out << "\n";
  }

  // member: proximity_cliff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity_cliff: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_cliff, out);
    out << "\n";
  }

  // member: cliff_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cliff_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff_detected, out);
    out << "\n";
  }

  // member: front_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.front_valid, out);
    out << "\n";
  }

  // member: rear_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_valid, out);
    out << "\n";
  }

  // member: cliff_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cliff_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff_valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Proximity & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rover_project

namespace rosidl_generator_traits
{

[[deprecated("use rover_project::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_project::msg::Proximity & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_project::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_project::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_project::msg::Proximity & msg)
{
  return rover_project::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_project::msg::Proximity>()
{
  return "rover_project::msg::Proximity";
}

template<>
inline const char * name<rover_project::msg::Proximity>()
{
  return "rover_project/msg/Proximity";
}

template<>
struct has_fixed_size<rover_project::msg::Proximity>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_project::msg::Proximity>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_project::msg::Proximity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_PROJECT__MSG__DETAIL__PROXIMITY__TRAITS_HPP_
