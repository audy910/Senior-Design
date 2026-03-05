// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_project:msg/ImuOrientation.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__BUILDER_HPP_
#define ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_project/msg/detail/imu_orientation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_project
{

namespace msg
{

namespace builder
{

class Init_ImuOrientation_cal_mag
{
public:
  explicit Init_ImuOrientation_cal_mag(::rover_project::msg::ImuOrientation & msg)
  : msg_(msg)
  {}
  ::rover_project::msg::ImuOrientation cal_mag(::rover_project::msg::ImuOrientation::_cal_mag_type arg)
  {
    msg_.cal_mag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_project::msg::ImuOrientation msg_;
};

class Init_ImuOrientation_cal_sys
{
public:
  explicit Init_ImuOrientation_cal_sys(::rover_project::msg::ImuOrientation & msg)
  : msg_(msg)
  {}
  Init_ImuOrientation_cal_mag cal_sys(::rover_project::msg::ImuOrientation::_cal_sys_type arg)
  {
    msg_.cal_sys = std::move(arg);
    return Init_ImuOrientation_cal_mag(msg_);
  }

private:
  ::rover_project::msg::ImuOrientation msg_;
};

class Init_ImuOrientation_roll
{
public:
  explicit Init_ImuOrientation_roll(::rover_project::msg::ImuOrientation & msg)
  : msg_(msg)
  {}
  Init_ImuOrientation_cal_sys roll(::rover_project::msg::ImuOrientation::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_ImuOrientation_cal_sys(msg_);
  }

private:
  ::rover_project::msg::ImuOrientation msg_;
};

class Init_ImuOrientation_pitch
{
public:
  explicit Init_ImuOrientation_pitch(::rover_project::msg::ImuOrientation & msg)
  : msg_(msg)
  {}
  Init_ImuOrientation_roll pitch(::rover_project::msg::ImuOrientation::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ImuOrientation_roll(msg_);
  }

private:
  ::rover_project::msg::ImuOrientation msg_;
};

class Init_ImuOrientation_heading
{
public:
  Init_ImuOrientation_heading()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuOrientation_pitch heading(::rover_project::msg::ImuOrientation::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_ImuOrientation_pitch(msg_);
  }

private:
  ::rover_project::msg::ImuOrientation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_project::msg::ImuOrientation>()
{
  return rover_project::msg::builder::Init_ImuOrientation_heading();
}

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__BUILDER_HPP_
