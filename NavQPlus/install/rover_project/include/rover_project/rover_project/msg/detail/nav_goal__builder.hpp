// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_project:msg/NavGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__BUILDER_HPP_
#define ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_project/msg/detail/nav_goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_project
{

namespace msg
{

namespace builder
{

class Init_NavGoal_longitude
{
public:
  explicit Init_NavGoal_longitude(::rover_project::msg::NavGoal & msg)
  : msg_(msg)
  {}
  ::rover_project::msg::NavGoal longitude(::rover_project::msg::NavGoal::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_project::msg::NavGoal msg_;
};

class Init_NavGoal_latitude
{
public:
  Init_NavGoal_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavGoal_longitude latitude(::rover_project::msg::NavGoal::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_NavGoal_longitude(msg_);
  }

private:
  ::rover_project::msg::NavGoal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_project::msg::NavGoal>()
{
  return rover_project::msg::builder::Init_NavGoal_latitude();
}

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__BUILDER_HPP_
