// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_project:msg/NavGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_HPP_
#define ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_project__msg__NavGoal __attribute__((deprecated))
#else
# define DEPRECATED__rover_project__msg__NavGoal __declspec(deprecated)
#endif

namespace rover_project
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavGoal_
{
  using Type = NavGoal_<ContainerAllocator>;

  explicit NavGoal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
    }
  }

  explicit NavGoal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_project::msg::NavGoal_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_project::msg::NavGoal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_project::msg::NavGoal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_project::msg::NavGoal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::NavGoal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::NavGoal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::NavGoal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::NavGoal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_project::msg::NavGoal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_project::msg::NavGoal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_project__msg__NavGoal
    std::shared_ptr<rover_project::msg::NavGoal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_project__msg__NavGoal
    std::shared_ptr<rover_project::msg::NavGoal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavGoal_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavGoal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavGoal_

// alias to use template instance with default allocator
using NavGoal =
  rover_project::msg::NavGoal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__NAV_GOAL__STRUCT_HPP_
