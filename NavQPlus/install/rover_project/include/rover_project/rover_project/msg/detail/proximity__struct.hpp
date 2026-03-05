// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_HPP_
#define ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_project__msg__Proximity __attribute__((deprecated))
#else
# define DEPRECATED__rover_project__msg__Proximity __declspec(deprecated)
#endif

namespace rover_project
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Proximity_
{
  using Type = Proximity_<ContainerAllocator>;

  explicit Proximity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->proximity_front = 0;
      this->proximity_rear = 0;
      this->proximity_cliff = 0;
      this->cliff_detected = false;
      this->front_valid = false;
      this->rear_valid = false;
      this->cliff_valid = false;
    }
  }

  explicit Proximity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->proximity_front = 0;
      this->proximity_rear = 0;
      this->proximity_cliff = 0;
      this->cliff_detected = false;
      this->front_valid = false;
      this->rear_valid = false;
      this->cliff_valid = false;
    }
  }

  // field types and members
  using _proximity_front_type =
    uint16_t;
  _proximity_front_type proximity_front;
  using _proximity_rear_type =
    uint16_t;
  _proximity_rear_type proximity_rear;
  using _proximity_cliff_type =
    uint16_t;
  _proximity_cliff_type proximity_cliff;
  using _cliff_detected_type =
    bool;
  _cliff_detected_type cliff_detected;
  using _front_valid_type =
    bool;
  _front_valid_type front_valid;
  using _rear_valid_type =
    bool;
  _rear_valid_type rear_valid;
  using _cliff_valid_type =
    bool;
  _cliff_valid_type cliff_valid;

  // setters for named parameter idiom
  Type & set__proximity_front(
    const uint16_t & _arg)
  {
    this->proximity_front = _arg;
    return *this;
  }
  Type & set__proximity_rear(
    const uint16_t & _arg)
  {
    this->proximity_rear = _arg;
    return *this;
  }
  Type & set__proximity_cliff(
    const uint16_t & _arg)
  {
    this->proximity_cliff = _arg;
    return *this;
  }
  Type & set__cliff_detected(
    const bool & _arg)
  {
    this->cliff_detected = _arg;
    return *this;
  }
  Type & set__front_valid(
    const bool & _arg)
  {
    this->front_valid = _arg;
    return *this;
  }
  Type & set__rear_valid(
    const bool & _arg)
  {
    this->rear_valid = _arg;
    return *this;
  }
  Type & set__cliff_valid(
    const bool & _arg)
  {
    this->cliff_valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_project::msg::Proximity_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_project::msg::Proximity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_project::msg::Proximity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_project::msg::Proximity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::Proximity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::Proximity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::Proximity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::Proximity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_project::msg::Proximity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_project::msg::Proximity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_project__msg__Proximity
    std::shared_ptr<rover_project::msg::Proximity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_project__msg__Proximity
    std::shared_ptr<rover_project::msg::Proximity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Proximity_ & other) const
  {
    if (this->proximity_front != other.proximity_front) {
      return false;
    }
    if (this->proximity_rear != other.proximity_rear) {
      return false;
    }
    if (this->proximity_cliff != other.proximity_cliff) {
      return false;
    }
    if (this->cliff_detected != other.cliff_detected) {
      return false;
    }
    if (this->front_valid != other.front_valid) {
      return false;
    }
    if (this->rear_valid != other.rear_valid) {
      return false;
    }
    if (this->cliff_valid != other.cliff_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const Proximity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Proximity_

// alias to use template instance with default allocator
using Proximity =
  rover_project::msg::Proximity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__PROXIMITY__STRUCT_HPP_
