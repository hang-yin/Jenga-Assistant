// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_execute_interface:srv/GoHere.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__BUILDER_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_execute_interface/srv/detail/go_here__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_execute_interface
{

namespace srv
{

namespace builder
{

class Init_GoHere_Request_execute
{
public:
  explicit Init_GoHere_Request_execute(::plan_execute_interface::srv::GoHere_Request & msg)
  : msg_(msg)
  {}
  ::plan_execute_interface::srv::GoHere_Request execute(::plan_execute_interface::srv::GoHere_Request::_execute_type arg)
  {
    msg_.execute = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_execute_interface::srv::GoHere_Request msg_;
};

class Init_GoHere_Request_goal_pose
{
public:
  explicit Init_GoHere_Request_goal_pose(::plan_execute_interface::srv::GoHere_Request & msg)
  : msg_(msg)
  {}
  Init_GoHere_Request_execute goal_pose(::plan_execute_interface::srv::GoHere_Request::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return Init_GoHere_Request_execute(msg_);
  }

private:
  ::plan_execute_interface::srv::GoHere_Request msg_;
};

class Init_GoHere_Request_start_pose
{
public:
  Init_GoHere_Request_start_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoHere_Request_goal_pose start_pose(::plan_execute_interface::srv::GoHere_Request::_start_pose_type arg)
  {
    msg_.start_pose = std::move(arg);
    return Init_GoHere_Request_goal_pose(msg_);
  }

private:
  ::plan_execute_interface::srv::GoHere_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_execute_interface::srv::GoHere_Request>()
{
  return plan_execute_interface::srv::builder::Init_GoHere_Request_start_pose();
}

}  // namespace plan_execute_interface


namespace plan_execute_interface
{

namespace srv
{

namespace builder
{

class Init_GoHere_Response_success
{
public:
  Init_GoHere_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_execute_interface::srv::GoHere_Response success(::plan_execute_interface::srv::GoHere_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_execute_interface::srv::GoHere_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_execute_interface::srv::GoHere_Response>()
{
  return plan_execute_interface::srv::builder::Init_GoHere_Response_success();
}

}  // namespace plan_execute_interface

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__BUILDER_HPP_
