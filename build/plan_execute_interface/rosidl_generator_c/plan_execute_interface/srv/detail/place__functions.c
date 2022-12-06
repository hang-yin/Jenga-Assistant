// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_execute_interface:srv/Place.idl
// generated code does not contain a copyright notice
#include "plan_execute_interface/srv/detail/place__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `place`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
plan_execute_interface__srv__Place_Request__init(plan_execute_interface__srv__Place_Request * msg)
{
  if (!msg) {
    return false;
  }
  // place
  if (!geometry_msgs__msg__Pose__init(&msg->place)) {
    plan_execute_interface__srv__Place_Request__fini(msg);
    return false;
  }
  return true;
}

void
plan_execute_interface__srv__Place_Request__fini(plan_execute_interface__srv__Place_Request * msg)
{
  if (!msg) {
    return;
  }
  // place
  geometry_msgs__msg__Pose__fini(&msg->place);
}

bool
plan_execute_interface__srv__Place_Request__are_equal(const plan_execute_interface__srv__Place_Request * lhs, const plan_execute_interface__srv__Place_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // place
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->place), &(rhs->place)))
  {
    return false;
  }
  return true;
}

bool
plan_execute_interface__srv__Place_Request__copy(
  const plan_execute_interface__srv__Place_Request * input,
  plan_execute_interface__srv__Place_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // place
  if (!geometry_msgs__msg__Pose__copy(
      &(input->place), &(output->place)))
  {
    return false;
  }
  return true;
}

plan_execute_interface__srv__Place_Request *
plan_execute_interface__srv__Place_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Request * msg = (plan_execute_interface__srv__Place_Request *)allocator.allocate(sizeof(plan_execute_interface__srv__Place_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_execute_interface__srv__Place_Request));
  bool success = plan_execute_interface__srv__Place_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_execute_interface__srv__Place_Request__destroy(plan_execute_interface__srv__Place_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_execute_interface__srv__Place_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_execute_interface__srv__Place_Request__Sequence__init(plan_execute_interface__srv__Place_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Request * data = NULL;

  if (size) {
    data = (plan_execute_interface__srv__Place_Request *)allocator.zero_allocate(size, sizeof(plan_execute_interface__srv__Place_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_execute_interface__srv__Place_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_execute_interface__srv__Place_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
plan_execute_interface__srv__Place_Request__Sequence__fini(plan_execute_interface__srv__Place_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      plan_execute_interface__srv__Place_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

plan_execute_interface__srv__Place_Request__Sequence *
plan_execute_interface__srv__Place_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Request__Sequence * array = (plan_execute_interface__srv__Place_Request__Sequence *)allocator.allocate(sizeof(plan_execute_interface__srv__Place_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_execute_interface__srv__Place_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_execute_interface__srv__Place_Request__Sequence__destroy(plan_execute_interface__srv__Place_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_execute_interface__srv__Place_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_execute_interface__srv__Place_Request__Sequence__are_equal(const plan_execute_interface__srv__Place_Request__Sequence * lhs, const plan_execute_interface__srv__Place_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_execute_interface__srv__Place_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_execute_interface__srv__Place_Request__Sequence__copy(
  const plan_execute_interface__srv__Place_Request__Sequence * input,
  plan_execute_interface__srv__Place_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_execute_interface__srv__Place_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_execute_interface__srv__Place_Request * data =
      (plan_execute_interface__srv__Place_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_execute_interface__srv__Place_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_execute_interface__srv__Place_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_execute_interface__srv__Place_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
plan_execute_interface__srv__Place_Response__init(plan_execute_interface__srv__Place_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
plan_execute_interface__srv__Place_Response__fini(plan_execute_interface__srv__Place_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
plan_execute_interface__srv__Place_Response__are_equal(const plan_execute_interface__srv__Place_Response * lhs, const plan_execute_interface__srv__Place_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
plan_execute_interface__srv__Place_Response__copy(
  const plan_execute_interface__srv__Place_Response * input,
  plan_execute_interface__srv__Place_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

plan_execute_interface__srv__Place_Response *
plan_execute_interface__srv__Place_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Response * msg = (plan_execute_interface__srv__Place_Response *)allocator.allocate(sizeof(plan_execute_interface__srv__Place_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_execute_interface__srv__Place_Response));
  bool success = plan_execute_interface__srv__Place_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_execute_interface__srv__Place_Response__destroy(plan_execute_interface__srv__Place_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_execute_interface__srv__Place_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_execute_interface__srv__Place_Response__Sequence__init(plan_execute_interface__srv__Place_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Response * data = NULL;

  if (size) {
    data = (plan_execute_interface__srv__Place_Response *)allocator.zero_allocate(size, sizeof(plan_execute_interface__srv__Place_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_execute_interface__srv__Place_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_execute_interface__srv__Place_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
plan_execute_interface__srv__Place_Response__Sequence__fini(plan_execute_interface__srv__Place_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      plan_execute_interface__srv__Place_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

plan_execute_interface__srv__Place_Response__Sequence *
plan_execute_interface__srv__Place_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_execute_interface__srv__Place_Response__Sequence * array = (plan_execute_interface__srv__Place_Response__Sequence *)allocator.allocate(sizeof(plan_execute_interface__srv__Place_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_execute_interface__srv__Place_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_execute_interface__srv__Place_Response__Sequence__destroy(plan_execute_interface__srv__Place_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_execute_interface__srv__Place_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_execute_interface__srv__Place_Response__Sequence__are_equal(const plan_execute_interface__srv__Place_Response__Sequence * lhs, const plan_execute_interface__srv__Place_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_execute_interface__srv__Place_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_execute_interface__srv__Place_Response__Sequence__copy(
  const plan_execute_interface__srv__Place_Response__Sequence * input,
  plan_execute_interface__srv__Place_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_execute_interface__srv__Place_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_execute_interface__srv__Place_Response * data =
      (plan_execute_interface__srv__Place_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_execute_interface__srv__Place_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_execute_interface__srv__Place_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_execute_interface__srv__Place_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
