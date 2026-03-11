// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_project:msg/ImuOrientation.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_HPP_
#define ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_project__msg__ImuOrientation __attribute__((deprecated))
#else
# define DEPRECATED__rover_project__msg__ImuOrientation __declspec(deprecated)
#endif

namespace rover_project
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuOrientation_
{
  using Type = ImuOrientation_<ContainerAllocator>;

  explicit ImuOrientation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->pitch = 0.0f;
      this->roll = 0.0f;
      this->cal_sys = 0;
      this->cal_mag = 0;
    }
  }

  explicit ImuOrientation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->pitch = 0.0f;
      this->roll = 0.0f;
      this->cal_sys = 0;
      this->cal_mag = 0;
    }
  }

  // field types and members
  using _heading_type =
    float;
  _heading_type heading;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _roll_type =
    float;
  _roll_type roll;
  using _cal_sys_type =
    uint8_t;
  _cal_sys_type cal_sys;
  using _cal_mag_type =
    uint8_t;
  _cal_mag_type cal_mag;

  // setters for named parameter idiom
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__cal_sys(
    const uint8_t & _arg)
  {
    this->cal_sys = _arg;
    return *this;
  }
  Type & set__cal_mag(
    const uint8_t & _arg)
  {
    this->cal_mag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_project::msg::ImuOrientation_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_project::msg::ImuOrientation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::ImuOrientation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::ImuOrientation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_project__msg__ImuOrientation
    std::shared_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_project__msg__ImuOrientation
    std::shared_ptr<rover_project::msg::ImuOrientation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuOrientation_ & other) const
  {
    if (this->heading != other.heading) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->cal_sys != other.cal_sys) {
      return false;
    }
    if (this->cal_mag != other.cal_mag) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuOrientation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuOrientation_

// alias to use template instance with default allocator
using ImuOrientation =
  rover_project::msg::ImuOrientation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__IMU_ORIENTATION__STRUCT_HPP_
