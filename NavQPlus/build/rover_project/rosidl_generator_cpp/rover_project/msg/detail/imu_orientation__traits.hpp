// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_project:msg/ImuOrientation.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__TRAITS_HPP_
#define ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_project/msg/detail/imu_orientation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_project
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImuOrientation & msg,
  std::ostream & out)
{
  out << "{";
  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: cal_sys
  {
    out << "cal_sys: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_sys, out);
    out << ", ";
  }

  // member: cal_mag
  {
    out << "cal_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_mag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImuOrientation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: cal_sys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cal_sys: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_sys, out);
    out << "\n";
  }

  // member: cal_mag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cal_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.cal_mag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImuOrientation & msg, bool use_flow_style = false)
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
  const rover_project::msg::ImuOrientation & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_project::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_project::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_project::msg::ImuOrientation & msg)
{
  return rover_project::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_project::msg::ImuOrientation>()
{
  return "rover_project::msg::ImuOrientation";
}

template<>
inline const char * name<rover_project::msg::ImuOrientation>()
{
  return "rover_project/msg/ImuOrientation";
}

template<>
struct has_fixed_size<rover_project::msg::ImuOrientation>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_project::msg::ImuOrientation>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_project::msg::ImuOrientation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__TRAITS_HPP_
