// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_execute_interface:srv/GoHere.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__TRAITS_HPP_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_execute_interface/srv/detail/go_here__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace plan_execute_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const GoHere_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_pose
  {
    if (msg.start_pose.size() == 0) {
      out << "start_pose: []";
    } else {
      out << "start_pose: [";
      size_t pending_items = msg.start_pose.size();
      for (auto item : msg.start_pose) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: goal_pose
  {
    out << "goal_pose: ";
    to_flow_style_yaml(msg.goal_pose, out);
    out << ", ";
  }

  // member: execute
  {
    out << "execute: ";
    rosidl_generator_traits::value_to_yaml(msg.execute, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoHere_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.start_pose.size() == 0) {
      out << "start_pose: []\n";
    } else {
      out << "start_pose:\n";
      for (auto item : msg.start_pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: goal_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_pose:\n";
    to_block_style_yaml(msg.goal_pose, out, indentation + 2);
  }

  // member: execute
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execute: ";
    rosidl_generator_traits::value_to_yaml(msg.execute, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoHere_Request & msg, bool use_flow_style = false)
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
  const plan_execute_interface::srv::GoHere_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_execute_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_execute_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_execute_interface::srv::GoHere_Request & msg)
{
  return plan_execute_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_execute_interface::srv::GoHere_Request>()
{
  return "plan_execute_interface::srv::GoHere_Request";
}

template<>
inline const char * name<plan_execute_interface::srv::GoHere_Request>()
{
  return "plan_execute_interface/srv/GoHere_Request";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::GoHere_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_execute_interface::srv::GoHere_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_execute_interface::srv::GoHere_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace plan_execute_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const GoHere_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoHere_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoHere_Response & msg, bool use_flow_style = false)
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
  const plan_execute_interface::srv::GoHere_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_execute_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_execute_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const plan_execute_interface::srv::GoHere_Response & msg)
{
  return plan_execute_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<plan_execute_interface::srv::GoHere_Response>()
{
  return "plan_execute_interface::srv::GoHere_Response";
}

template<>
inline const char * name<plan_execute_interface::srv::GoHere_Response>()
{
  return "plan_execute_interface/srv/GoHere_Response";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::GoHere_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<plan_execute_interface::srv::GoHere_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<plan_execute_interface::srv::GoHere_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<plan_execute_interface::srv::GoHere>()
{
  return "plan_execute_interface::srv::GoHere";
}

template<>
inline const char * name<plan_execute_interface::srv::GoHere>()
{
  return "plan_execute_interface/srv/GoHere";
}

template<>
struct has_fixed_size<plan_execute_interface::srv::GoHere>
  : std::integral_constant<
    bool,
    has_fixed_size<plan_execute_interface::srv::GoHere_Request>::value &&
    has_fixed_size<plan_execute_interface::srv::GoHere_Response>::value
  >
{
};

template<>
struct has_bounded_size<plan_execute_interface::srv::GoHere>
  : std::integral_constant<
    bool,
    has_bounded_size<plan_execute_interface::srv::GoHere_Request>::value &&
    has_bounded_size<plan_execute_interface::srv::GoHere_Response>::value
  >
{
};

template<>
struct is_service<plan_execute_interface::srv::GoHere>
  : std::true_type
{
};

template<>
struct is_service_request<plan_execute_interface::srv::GoHere_Request>
  : std::true_type
{
};

template<>
struct is_service_response<plan_execute_interface::srv::GoHere_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__TRAITS_HPP_
