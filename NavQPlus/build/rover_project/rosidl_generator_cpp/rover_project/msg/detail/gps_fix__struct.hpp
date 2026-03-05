// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_project:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_HPP_
#define ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_project__msg__GpsFix __attribute__((deprecated))
#else
# define DEPRECATED__rover_project__msg__GpsFix __declspec(deprecated)
#endif

namespace rover_project
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsFix_
{
  using Type = GpsFix_<ContainerAllocator>;

  explicit GpsFix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->speed = 0.0f;
      this->course = 0.0f;
      this->fix_type = 0;
      this->num_sats = 0;
      this->hdop = 0.0f;
      this->h_acc = 0.0f;
      this->v_acc = 0.0f;
    }
  }

  explicit GpsFix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->speed = 0.0f;
      this->course = 0.0f;
      this->fix_type = 0;
      this->num_sats = 0;
      this->hdop = 0.0f;
      this->h_acc = 0.0f;
      this->v_acc = 0.0f;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _speed_type =
    float;
  _speed_type speed;
  using _course_type =
    float;
  _course_type course;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _num_sats_type =
    uint8_t;
  _num_sats_type num_sats;
  using _hdop_type =
    float;
  _hdop_type hdop;
  using _h_acc_type =
    float;
  _h_acc_type h_acc;
  using _v_acc_type =
    float;
  _v_acc_type v_acc;

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
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__course(
    const float & _arg)
  {
    this->course = _arg;
    return *this;
  }
  Type & set__fix_type(
    const uint8_t & _arg)
  {
    this->fix_type = _arg;
    return *this;
  }
  Type & set__num_sats(
    const uint8_t & _arg)
  {
    this->num_sats = _arg;
    return *this;
  }
  Type & set__hdop(
    const float & _arg)
  {
    this->hdop = _arg;
    return *this;
  }
  Type & set__h_acc(
    const float & _arg)
  {
    this->h_acc = _arg;
    return *this;
  }
  Type & set__v_acc(
    const float & _arg)
  {
    this->v_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_project::msg::GpsFix_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_project::msg::GpsFix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_project::msg::GpsFix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_project::msg::GpsFix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::GpsFix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::GpsFix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::GpsFix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::GpsFix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_project::msg::GpsFix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_project::msg::GpsFix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_project__msg__GpsFix
    std::shared_ptr<rover_project::msg::GpsFix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_project__msg__GpsFix
    std::shared_ptr<rover_project::msg::GpsFix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsFix_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->course != other.course) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
      return false;
    }
    if (this->num_sats != other.num_sats) {
      return false;
    }
    if (this->hdop != other.hdop) {
      return false;
    }
    if (this->h_acc != other.h_acc) {
      return false;
    }
    if (this->v_acc != other.v_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsFix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsFix_

// alias to use template instance with default allocator
using GpsFix =
  rover_project::msg::GpsFix_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__GPS_FIX__STRUCT_HPP_
