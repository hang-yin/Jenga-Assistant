// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_execute_interface:srv/GoHere.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_H_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/GoHere in the package plan_execute_interface.
typedef struct plan_execute_interface__srv__GoHere_Request
{
  geometry_msgs__msg__Pose__Sequence start_pose;
  geometry_msgs__msg__Pose goal_pose;
  bool execute;
} plan_execute_interface__srv__GoHere_Request;

// Struct for a sequence of plan_execute_interface__srv__GoHere_Request.
typedef struct plan_execute_interface__srv__GoHere_Request__Sequence
{
  plan_execute_interface__srv__GoHere_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_execute_interface__srv__GoHere_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GoHere in the package plan_execute_interface.
typedef struct plan_execute_interface__srv__GoHere_Response
{
  bool success;
} plan_execute_interface__srv__GoHere_Response;

// Struct for a sequence of plan_execute_interface__srv__GoHere_Response.
typedef struct plan_execute_interface__srv__GoHere_Response__Sequence
{
  plan_execute_interface__srv__GoHere_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_execute_interface__srv__GoHere_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__GO_HERE__STRUCT_H_
