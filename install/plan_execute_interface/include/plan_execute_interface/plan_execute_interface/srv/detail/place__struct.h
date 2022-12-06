// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice

#ifndef PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_H_
#define PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'place'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/Place in the package plan_execute_interface.
typedef struct plan_execute_interface__srv__Place_Request
{
  geometry_msgs__msg__Pose place;
} plan_execute_interface__srv__Place_Request;

// Struct for a sequence of plan_execute_interface__srv__Place_Request.
typedef struct plan_execute_interface__srv__Place_Request__Sequence
{
  plan_execute_interface__srv__Place_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_execute_interface__srv__Place_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Place in the package plan_execute_interface.
typedef struct plan_execute_interface__srv__Place_Response
{
  uint8_t structure_needs_at_least_one_member;
} plan_execute_interface__srv__Place_Response;

// Struct for a sequence of plan_execute_interface__srv__Place_Response.
typedef struct plan_execute_interface__srv__Place_Response__Sequence
{
  plan_execute_interface__srv__Place_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_execute_interface__srv__Place_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_EXECUTE_INTERFACE__SRV__DETAIL__PLACE__STRUCT_H_
