// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_project:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__GPS_FIX__BUILDER_HPP_
#define ROVER_PROJECT__MSG__DETAIL__GPS_FIX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_project/msg/detail/gps_fix__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_project
{

namespace msg
{

namespace builder
{

class Init_GpsFix_v_acc
{
public:
  explicit Init_GpsFix_v_acc(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  ::rover_project::msg::GpsFix v_acc(::rover_project::msg::GpsFix::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_h_acc
{
public:
  explicit Init_GpsFix_h_acc(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_v_acc h_acc(::rover_project::msg::GpsFix::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_GpsFix_v_acc(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_hdop
{
public:
  explicit Init_GpsFix_hdop(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_h_acc hdop(::rover_project::msg::GpsFix::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_GpsFix_h_acc(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_num_sats
{
public:
  explicit Init_GpsFix_num_sats(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_hdop num_sats(::rover_project::msg::GpsFix::_num_sats_type arg)
  {
    msg_.num_sats = std::move(arg);
    return Init_GpsFix_hdop(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_fix_type
{
public:
  explicit Init_GpsFix_fix_type(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_num_sats fix_type(::rover_project::msg::GpsFix::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_GpsFix_num_sats(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_course
{
public:
  explicit Init_GpsFix_course(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_fix_type course(::rover_project::msg::GpsFix::_course_type arg)
  {
    msg_.course = std::move(arg);
    return Init_GpsFix_fix_type(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_speed
{
public:
  explicit Init_GpsFix_speed(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_course speed(::rover_project::msg::GpsFix::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_GpsFix_course(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_longitude
{
public:
  explicit Init_GpsFix_longitude(::rover_project::msg::GpsFix & msg)
  : msg_(msg)
  {}
  Init_GpsFix_speed longitude(::rover_project::msg::GpsFix::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GpsFix_speed(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

class Init_GpsFix_latitude
{
public:
  Init_GpsFix_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsFix_longitude latitude(::rover_project::msg::GpsFix::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsFix_longitude(msg_);
  }

private:
  ::rover_project::msg::GpsFix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_project::msg::GpsFix>()
{
  return rover_project::msg::builder::Init_GpsFix_latitude();
}

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__GPS_FIX__BUILDER_HPP_
