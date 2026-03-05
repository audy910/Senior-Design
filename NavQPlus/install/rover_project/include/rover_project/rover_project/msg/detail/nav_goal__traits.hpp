// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_project:msg/NavGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__TRAITS_HPP_
#define ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_project/msg/detail/nav_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_project
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavGoal & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavGoal & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavGoal & msg, bool use_flow_style = false)
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
  const rover_project::msg::NavGoal & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_project::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_project::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_project::msg::NavGoal & msg)
{
  return rover_project::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_project::msg::NavGoal>()
{
  return "rover_project::msg::NavGoal";
}

template<>
inline const char * name<rover_project::msg::NavGoal>()
{
  return "rover_project/msg/NavGoal";
}

template<>
struct has_fixed_size<rover_project::msg::NavGoal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_project::msg::NavGoal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_project::msg::NavGoal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__TRAITS_HPP_
