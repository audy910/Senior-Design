// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__TRAITS_HPP_
#define ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_project/msg/detail/waypoint_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_project
{

namespace msg
{

inline void to_flow_style_yaml(
  const WaypointList & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitudes
  {
    if (msg.latitudes.size() == 0) {
      out << "latitudes: []";
    } else {
      out << "latitudes: [";
      size_t pending_items = msg.latitudes.size();
      for (auto item : msg.latitudes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: longitudes
  {
    if (msg.longitudes.size() == 0) {
      out << "longitudes: []";
    } else {
      out << "longitudes: [";
      size_t pending_items = msg.longitudes.size();
      for (auto item : msg.longitudes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: total_waypoints
  {
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << ", ";
  }

  // member: current_index
  {
    out << "current_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WaypointList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitudes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.latitudes.size() == 0) {
      out << "latitudes: []\n";
    } else {
      out << "latitudes:\n";
      for (auto item : msg.latitudes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: longitudes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.longitudes.size() == 0) {
      out << "longitudes: []\n";
    } else {
      out << "longitudes:\n";
      for (auto item : msg.longitudes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: total_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << "\n";
  }

  // member: current_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WaypointList & msg, bool use_flow_style = false)
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
  const rover_project::msg::WaypointList & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_project::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_project::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_project::msg::WaypointList & msg)
{
  return rover_project::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_project::msg::WaypointList>()
{
  return "rover_project::msg::WaypointList";
}

template<>
inline const char * name<rover_project::msg::WaypointList>()
{
  return "rover_project/msg/WaypointList";
}

template<>
struct has_fixed_size<rover_project::msg::WaypointList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_project::msg::WaypointList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_project::msg::WaypointList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__TRAITS_HPP_
