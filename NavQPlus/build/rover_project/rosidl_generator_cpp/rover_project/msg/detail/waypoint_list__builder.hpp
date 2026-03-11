// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_
#define ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_project/msg/detail/waypoint_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_project
{

namespace msg
{

namespace builder
{

class Init_WaypointList_current_index
{
public:
  explicit Init_WaypointList_current_index(::rover_project::msg::WaypointList & msg)
  : msg_(msg)
  {}
  ::rover_project::msg::WaypointList current_index(::rover_project::msg::WaypointList::_current_index_type arg)
  {
    msg_.current_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_project::msg::WaypointList msg_;
};

class Init_WaypointList_total_waypoints
{
public:
  explicit Init_WaypointList_total_waypoints(::rover_project::msg::WaypointList & msg)
  : msg_(msg)
  {}
  Init_WaypointList_current_index total_waypoints(::rover_project::msg::WaypointList::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_WaypointList_current_index(msg_);
  }

private:
  ::rover_project::msg::WaypointList msg_;
};

class Init_WaypointList_longitudes
{
public:
  explicit Init_WaypointList_longitudes(::rover_project::msg::WaypointList & msg)
  : msg_(msg)
  {}
  Init_WaypointList_total_waypoints longitudes(::rover_project::msg::WaypointList::_longitudes_type arg)
  {
    msg_.longitudes = std::move(arg);
    return Init_WaypointList_total_waypoints(msg_);
  }

private:
  ::rover_project::msg::WaypointList msg_;
};

class Init_WaypointList_latitudes
{
public:
  Init_WaypointList_latitudes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointList_longitudes latitudes(::rover_project::msg::WaypointList::_latitudes_type arg)
  {
    msg_.latitudes = std::move(arg);
    return Init_WaypointList_longitudes(msg_);
  }

private:
  ::rover_project::msg::WaypointList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_project::msg::WaypointList>()
{
  return rover_project::msg::builder::Init_WaypointList_latitudes();
}

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_
