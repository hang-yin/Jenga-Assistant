// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "plan_execute_interface/srv/detail/place__rosidl_typesupport_introspection_c.h"
#include "plan_execute_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "plan_execute_interface/srv/detail/place__functions.h"
#include "plan_execute_interface/srv/detail/place__struct.h"


// Include directives for member types
// Member `place`
#include "geometry_msgs/msg/pose.h"
// Member `place`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_execute_interface__srv__Place_Request__init(message_memory);
}

void plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_fini_function(void * message_memory)
{
  plan_execute_interface__srv__Place_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_member_array[1] = {
  {
    "place",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_execute_interface__srv__Place_Request, place),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_members = {
  "plan_execute_interface__srv",  // message namespace
  "Place_Request",  // message name
  1,  // number of fields
  sizeof(plan_execute_interface__srv__Place_Request),
  plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_member_array,  // message members
  plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_type_support_handle = {
  0,
  &plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_execute_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Request)() {
  plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_type_support_handle.typesupport_identifier) {
    plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_execute_interface__srv__Place_Request__rosidl_typesupport_introspection_c__Place_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "plan_execute_interface/srv/detail/place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "plan_execute_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "plan_execute_interface/srv/detail/place__functions.h"
// already included above
// #include "plan_execute_interface/srv/detail/place__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_execute_interface__srv__Place_Response__init(message_memory);
}

void plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_fini_function(void * message_memory)
{
  plan_execute_interface__srv__Place_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_execute_interface__srv__Place_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_members = {
  "plan_execute_interface__srv",  // message namespace
  "Place_Response",  // message name
  1,  // number of fields
  sizeof(plan_execute_interface__srv__Place_Response),
  plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_member_array,  // message members
  plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_type_support_handle = {
  0,
  &plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_execute_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Response)() {
  if (!plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_type_support_handle.typesupport_identifier) {
    plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_execute_interface__srv__Place_Response__rosidl_typesupport_introspection_c__Place_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "plan_execute_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "plan_execute_interface/srv/detail/place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_members = {
  "plan_execute_interface__srv",  // service namespace
  "Place",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_Request_message_type_support_handle,
  NULL  // response message
  // plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_Response_message_type_support_handle
};

static rosidl_service_type_support_t plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_type_support_handle = {
  0,
  &plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_execute_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place)() {
  if (!plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_type_support_handle.typesupport_identifier) {
    plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_execute_interface, srv, Place_Response)()->data;
  }

  return &plan_execute_interface__srv__detail__place__rosidl_typesupport_introspection_c__Place_service_type_support_handle;
}
