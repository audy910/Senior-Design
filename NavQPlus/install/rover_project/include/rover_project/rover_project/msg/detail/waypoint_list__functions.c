// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_project:msg/WaypointList.idl
// generated code does not contain a copyright notice
#include "rover_project/msg/detail/waypoint_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `latitudes`
// Member `longitudes`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rover_project__msg__WaypointList__init(rover_project__msg__WaypointList * msg)
{
  if (!msg) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->latitudes, 0)) {
    rover_project__msg__WaypointList__fini(msg);
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->longitudes, 0)) {
    rover_project__msg__WaypointList__fini(msg);
    return false;
  }
  // total_waypoints
  // current_index
  return true;
}

void
rover_project__msg__WaypointList__fini(rover_project__msg__WaypointList * msg)
{
  if (!msg) {
    return;
  }
  // latitudes
  rosidl_runtime_c__double__Sequence__fini(&msg->latitudes);
  // longitudes
  rosidl_runtime_c__double__Sequence__fini(&msg->longitudes);
  // total_waypoints
  // current_index
}

bool
rover_project__msg__WaypointList__are_equal(const rover_project__msg__WaypointList * lhs, const rover_project__msg__WaypointList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->latitudes), &(rhs->latitudes)))
  {
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->longitudes), &(rhs->longitudes)))
  {
    return false;
  }
  // total_waypoints
  if (lhs->total_waypoints != rhs->total_waypoints) {
    return false;
  }
  // current_index
  if (lhs->current_index != rhs->current_index) {
    return false;
  }
  return true;
}

bool
rover_project__msg__WaypointList__copy(
  const rover_project__msg__WaypointList * input,
  rover_project__msg__WaypointList * output)
{
  if (!input || !output) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->latitudes), &(output->latitudes)))
  {
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->longitudes), &(output->longitudes)))
  {
    return false;
  }
  // total_waypoints
  output->total_waypoints = input->total_waypoints;
  // current_index
  output->current_index = input->current_index;
  return true;
}

rover_project__msg__WaypointList *
rover_project__msg__WaypointList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__WaypointList * msg = (rover_project__msg__WaypointList *)allocator.allocate(sizeof(rover_project__msg__WaypointList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_project__msg__WaypointList));
  bool success = rover_project__msg__WaypointList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_project__msg__WaypointList__destroy(rover_project__msg__WaypointList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_project__msg__WaypointList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_project__msg__WaypointList__Sequence__init(rover_project__msg__WaypointList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__WaypointList * data = NULL;

  if (size) {
    data = (rover_project__msg__WaypointList *)allocator.zero_allocate(size, sizeof(rover_project__msg__WaypointList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_project__msg__WaypointList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_project__msg__WaypointList__fini(&data[i - 1]);
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
rover_project__msg__WaypointList__Sequence__fini(rover_project__msg__WaypointList__Sequence * array)
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
      rover_project__msg__WaypointList__fini(&array->data[i]);
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

rover_project__msg__WaypointList__Sequence *
rover_project__msg__WaypointList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__WaypointList__Sequence * array = (rover_project__msg__WaypointList__Sequence *)allocator.allocate(sizeof(rover_project__msg__WaypointList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_project__msg__WaypointList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_project__msg__WaypointList__Sequence__destroy(rover_project__msg__WaypointList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_project__msg__WaypointList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_project__msg__WaypointList__Sequence__are_equal(const rover_project__msg__WaypointList__Sequence * lhs, const rover_project__msg__WaypointList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_project__msg__WaypointList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_project__msg__WaypointList__Sequence__copy(
  const rover_project__msg__WaypointList__Sequence * input,
  rover_project__msg__WaypointList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_project__msg__WaypointList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_project__msg__WaypointList * data =
      (rover_project__msg__WaypointList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_project__msg__WaypointList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_project__msg__WaypointList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_project__msg__WaypointList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
