// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'place'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_execute_interface__srv__Place_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_execute_interface__srv__Place_Request __declspec(deprecated)
#endif

namespace plan_execute_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Place_Request_
{
  using Type = Place_Request_<ContainerAllocator>;

  explicit Place_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : place(_init)
  {
    (void)_init;
  }

  explicit Place_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : place(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _place_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _place_type place;

  // setters for named parameter idiom
  Type & set__place(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->place = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_execute_interface::srv::Place_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_execute_interface::srv::Place_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::Place_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::Place_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_execute_interface__srv__Place_Request
    std::shared_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_execute_interface__srv__Place_Request
    std::shared_ptr<plan_execute_interface::srv::Place_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Place_Request_ & other) const
  {
    if (this->place != other.place) {
      return false;
    }
    return true;
  }
  bool operator!=(const Place_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Place_Request_

// alias to use template instance with default allocator
using Place_Request =
  plan_execute_interface::srv::Place_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace plan_execute_interface


#ifndef _WIN32
# define DEPRECATED__plan_execute_interface__srv__Place_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_execute_interface__srv__Place_Response __declspec(deprecated)
#endif

namespace plan_execute_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Place_Response_
{
  using Type = Place_Response_<ContainerAllocator>;

  explicit Place_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Place_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    plan_execute_interface::srv::Place_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_execute_interface::srv::Place_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::Place_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::Place_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_execute_interface__srv__Place_Response
    std::shared_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_execute_interface__srv__Place_Response
    std::shared_ptr<plan_execute_interface::srv::Place_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Place_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Place_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Place_Response_

// alias to use template instance with default allocator
using Place_Response =
  plan_execute_interface::srv::Place_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace plan_execute_interface

namespace plan_execute_interface
{

namespace srv
{

struct Place
{
  using Request = plan_execute_interface::srv::Place_Request;
  using Response = plan_execute_interface::srv::Place_Response;
};

}  // namespace srv

}  // namespace plan_execute_interface

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_HPP_
