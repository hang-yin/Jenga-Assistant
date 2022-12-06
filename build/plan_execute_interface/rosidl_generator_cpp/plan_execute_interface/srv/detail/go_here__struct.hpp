// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_execute_interface:srv/GoHere.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_execute_interface__srv__GoHere_Request __attribute__((deprecated))
#else
# define DEPRECATED__plan_execute_interface__srv__GoHere_Request __declspec(deprecated)
#endif

namespace plan_execute_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoHere_Request_
{
  using Type = GoHere_Request_<ContainerAllocator>;

  explicit GoHere_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->execute = false;
    }
  }

  explicit GoHere_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->execute = false;
    }
  }

  // field types and members
  using _start_pose_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _start_pose_type start_pose;
  using _goal_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _goal_pose_type goal_pose;
  using _execute_type =
    bool;
  _execute_type execute;

  // setters for named parameter idiom
  Type & set__start_pose(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->start_pose = _arg;
    return *this;
  }
  Type & set__goal_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->goal_pose = _arg;
    return *this;
  }
  Type & set__execute(
    const bool & _arg)
  {
    this->execute = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_execute_interface__srv__GoHere_Request
    std::shared_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_execute_interface__srv__GoHere_Request
    std::shared_ptr<plan_execute_interface::srv::GoHere_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoHere_Request_ & other) const
  {
    if (this->start_pose != other.start_pose) {
      return false;
    }
    if (this->goal_pose != other.goal_pose) {
      return false;
    }
    if (this->execute != other.execute) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoHere_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoHere_Request_

// alias to use template instance with default allocator
using GoHere_Request =
  plan_execute_interface::srv::GoHere_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace plan_execute_interface


#ifndef _WIN32
# define DEPRECATED__plan_execute_interface__srv__GoHere_Response __attribute__((deprecated))
#else
# define DEPRECATED__plan_execute_interface__srv__GoHere_Response __declspec(deprecated)
#endif

namespace plan_execute_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoHere_Response_
{
  using Type = GoHere_Response_<ContainerAllocator>;

  explicit GoHere_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GoHere_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_execute_interface__srv__GoHere_Response
    std::shared_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_execute_interface__srv__GoHere_Response
    std::shared_ptr<plan_execute_interface::srv::GoHere_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoHere_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoHere_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoHere_Response_

// alias to use template instance with default allocator
using GoHere_Response =
  plan_execute_interface::srv::GoHere_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace plan_execute_interface

namespace plan_execute_interface
{

namespace srv
{

struct GoHere
{
  using Request = plan_execute_interface::srv::GoHere_Request;
  using Response = plan_execute_interface::srv::GoHere_Response;
};

}  // namespace srv

}  // namespace plan_execute_interface

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_HPP_
