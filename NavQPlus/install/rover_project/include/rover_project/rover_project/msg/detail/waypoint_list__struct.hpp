// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_HPP_
#define ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_project__msg__WaypointList __attribute__((deprecated))
#else
# define DEPRECATED__rover_project__msg__WaypointList __declspec(deprecated)
#endif

namespace rover_project
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WaypointList_
{
  using Type = WaypointList_<ContainerAllocator>;

  explicit WaypointList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_waypoints = 0ul;
      this->current_index = 0ul;
    }
  }

  explicit WaypointList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_waypoints = 0ul;
      this->current_index = 0ul;
    }
  }

  // field types and members
  using _latitudes_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _latitudes_type latitudes;
  using _longitudes_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _longitudes_type longitudes;
  using _total_waypoints_type =
    uint32_t;
  _total_waypoints_type total_waypoints;
  using _current_index_type =
    uint32_t;
  _current_index_type current_index;

  // setters for named parameter idiom
  Type & set__latitudes(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->latitudes = _arg;
    return *this;
  }
  Type & set__longitudes(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->longitudes = _arg;
    return *this;
  }
  Type & set__total_waypoints(
    const uint32_t & _arg)
  {
    this->total_waypoints = _arg;
    return *this;
  }
  Type & set__current_index(
    const uint32_t & _arg)
  {
    this->current_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_project::msg::WaypointList_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_project::msg::WaypointList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_project::msg::WaypointList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_project::msg::WaypointList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::WaypointList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::WaypointList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_project::msg::WaypointList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_project::msg::WaypointList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_project::msg::WaypointList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_project::msg::WaypointList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_project__msg__WaypointList
    std::shared_ptr<rover_project::msg::WaypointList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_project__msg__WaypointList
    std::shared_ptr<rover_project::msg::WaypointList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointList_ & other) const
  {
    if (this->latitudes != other.latitudes) {
      return false;
    }
    if (this->longitudes != other.longitudes) {
      return false;
    }
    if (this->total_waypoints != other.total_waypoints) {
      return false;
    }
    if (this->current_index != other.current_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointList_

// alias to use template instance with default allocator
using WaypointList =
  rover_project::msg::WaypointList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_project

#endif  // ROVER_PROJECT__MSG__DETAIL__WAYPOINT_LIST__STRUCT_HPP_
