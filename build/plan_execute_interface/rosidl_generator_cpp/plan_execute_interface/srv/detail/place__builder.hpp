// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__BUILDER_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_execute_interface/srv/detail/place__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_execute_interface
{

namespace srv
{

namespace builder
{

class Init_Place_Request_place
{
public:
  Init_Place_Request_place()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::plan_execute_interface::srv::Place_Request place(::plan_execute_interface::srv::Place_Request::_place_type arg)
  {
    msg_.place = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_execute_interface::srv::Place_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_execute_interface::srv::Place_Request>()
{
  return plan_execute_interface::srv::builder::Init_Place_Request_place();
}

}  // namespace plan_execute_interface


namespace plan_execute_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_execute_interface::srv::Place_Response>()
{
  return ::plan_execute_interface::srv::Place_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace plan_execute_interface

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__BUILDER_HPP_
