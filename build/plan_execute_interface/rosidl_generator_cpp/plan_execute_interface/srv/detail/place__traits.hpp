// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__TRAITS_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_execute_interface/srv/detail/place__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'place'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace plan_execute_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Place_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: place
  {
    out << "place: ";
    to_flow_style_yaml(msg.place, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Place_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: place
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place:\n";
    to_block_style_yaml(msg.place, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Place_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace plan_execute_interface

namespace rosidl_generator_traits
{

[[deprecated("use plan_execute_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_execute_interface::srv::Place_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_execute_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_execute_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_execute_interface::srv::Place_Request & msg)
{
  return plan_execute_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_execute_interface::srv::Place_Request>()
{
  return "plan_execute_interface::srv::Place_Request";
}

template<>
inline const char * name<plan_execute_interface::srv::Place_Request>()
{
  return "plan_execute_interface/srv/Place_Request";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::Place_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<plan_execute_interface::srv::Place_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<plan_execute_interface::srv::Place_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace plan_execute_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Place_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Place_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Place_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace plan_execute_interface

namespace rosidl_generator_traits
{

[[deprecated("use plan_execute_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_execute_interface::srv::Place_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_execute_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_execute_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_execute_interface::srv::Place_Response & msg)
{
  return plan_execute_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_execute_interface::srv::Place_Response>()
{
  return "plan_execute_interface::srv::Place_Response";
}

template<>
inline const char * name<plan_execute_interface::srv::Place_Response>()
{
  return "plan_execute_interface/srv/Place_Response";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::Place_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<plan_execute_interface::srv::Place_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<plan_execute_interface::srv::Place_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_execute_interface::srv::Place>()
{
  return "plan_execute_interface::srv::Place";
}

template<>
inline const char * name<plan_execute_interface::srv::Place>()
{
  return "plan_execute_interface/srv/Place";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::Place>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_execute_interface::srv::Place_Request>::value &&
    has_fixed_size<plan_execute_interface::srv::Place_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_execute_interface::srv::Place>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_execute_interface::srv::Place_Request>::value &&
    has_bounded_size<plan_execute_interface::srv::Place_Response>::value
  >
{
};

template<>
struct is_service<plan_execute_interface::srv::Place>
  : std::true_type
{
};

template<>
struct is_service_request<plan_execute_interface::srv::Place_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_execute_interface::srv::Place_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__TRAITS_HPP_
