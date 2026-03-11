// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_project:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__GPS_FIX__TRAITS_HPP_
#define ROVER_PROJECT__MSG__DETAIL__GPS_FIX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_project/msg/detail/gps_fix__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_project
{

namespace msg
{

inline void to_flow_style_yaml(
  const GpsFix & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: course
  {
    out << "course: ";
    rosidl_generator_traits::value_to_yaml(msg.course, out);
    out << ", ";
  }

  // member: fix_type
  {
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << ", ";
  }

  // member: num_sats
  {
    out << "num_sats: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sats, out);
    out << ", ";
  }

  // member: hdop
  {
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << ", ";
  }

  // member: h_acc
  {
    out << "h_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.h_acc, out);
    out << ", ";
  }

  // member: v_acc
  {
    out << "v_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.v_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GpsFix & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: course
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "course: ";
    rosidl_generator_traits::value_to_yaml(msg.course, out);
    out << "\n";
  }

  // member: fix_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << "\n";
  }

  // member: num_sats
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sats: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sats, out);
    out << "\n";
  }

  // member: hdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hdop: ";
    rosidl_generator_traits::value_to_yaml(msg.hdop, out);
    out << "\n";
  }

  // member: h_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.h_acc, out);
    out << "\n";
  }

  // member: v_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.v_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GpsFix & msg, bool use_flow_style = false)
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
  const rover_project::msg::GpsFix & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_project::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_project::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_project::msg::GpsFix & msg)
{
  return rover_project::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_project::msg::GpsFix>()
{
  return "rover_project::msg::GpsFix";
}

template<>
inline const char * name<rover_project::msg::GpsFix>()
{
  return "rover_project/msg/GpsFix";
}

template<>
struct has_fixed_size<rover_project::msg::GpsFix>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_project::msg::GpsFix>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_project::msg::GpsFix>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_PROJECT__MSG__DETAIL__GPS_FIX__TRAITS_HPP_
