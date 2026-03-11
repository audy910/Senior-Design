// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__PROXIMITY__BUILDER_HPP_
#define ROVER_PROJECT__MSG__DETAIL__PROXIMITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_project/msg/detail/proximity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_project
{

namespace msg
{

namespace builder
{

class Init_Proximity_cliff_valid
{
public:
  explicit Init_Proximity_cliff_valid(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  ::rover_project::msg::Proximity cliff_valid(::rover_project::msg::Proximity::_cliff_valid_type arg)
  {
    msg_.cliff_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_rear_valid
{
public:
  explicit Init_Proximity_rear_valid(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  Init_Proximity_cliff_valid rear_valid(::rover_project::msg::Proximity::_rear_valid_type arg)
  {
    msg_.rear_valid = std::move(arg);
    return Init_Proximity_cliff_valid(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_front_valid
{
public:
  explicit Init_Proximity_front_valid(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  Init_Proximity_rear_valid front_valid(::rover_project::msg::Proximity::_front_valid_type arg)
  {
    msg_.front_valid = std::move(arg);
    return Init_Proximity_rear_valid(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_cliff_detected
{
public:
  explicit Init_Proximity_cliff_detected(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  Init_Proximity_front_valid cliff_detected(::rover_project::msg::Proximity::_cliff_detected_type arg)
  {
    msg_.cliff_detected = std::move(arg);
    return Init_Proximity_front_valid(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_proximity_cliff
{
public:
  explicit Init_Proximity_proximity_cliff(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  Init_Proximity_cliff_detected proximity_cliff(::rover_project::msg::Proximity::_proximity_cliff_type arg)
  {
    msg_.proximity_cliff = std::move(arg);
    return Init_Proximity_cliff_detected(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_proximity_rear
{
public:
  explicit Init_Proximity_proximity_rear(::rover_project::msg::Proximity & msg)
  : msg_(msg)
  {}
  Init_Proximity_proximity_cliff proximity_rear(::rover_project::msg::Proximity::_proximity_rear_type arg)
  {
    msg_.proximity_rear = std::move(arg);
    return Init_Proximity_proximity_cliff(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

class Init_Proximity_proximity_front
{
public:
  Init_Proximity_proximity_front()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Proximity_proximity_rear proximity_front(::rover_project::msg::Proximity::_proximity_front_type arg)
  {
    msg_.proximity_front = std::move(arg);
    return Init_Proximity_proximity_rear(msg_);
  }

private:
  ::rover_project::msg::Proximity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_project::msg::Proximity>()
{
  return rover_project::msg::builder::Init_Proximity_proximity_front();
}

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__PROXIMITY__BUILDER_HPP_
