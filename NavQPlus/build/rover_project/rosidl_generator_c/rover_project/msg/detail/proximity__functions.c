// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_project:msg/Proximity.idl
// generated code does not contain a copyright notice
#include "rover_project/msg/detail/proximity__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_project__msg__Proximity__init(rover_project__msg__Proximity * msg)
{
  if (!msg) {
    return false;
  }
  // proximity_front
  // proximity_rear
  // proximity_cliff
  // cliff_detected
  // front_valid
  // rear_valid
  // cliff_valid
  return true;
}

void
rover_project__msg__Proximity__fini(rover_project__msg__Proximity * msg)
{
  if (!msg) {
    return;
  }
  // proximity_front
  // proximity_rear
  // proximity_cliff
  // cliff_detected
  // front_valid
  // rear_valid
  // cliff_valid
}

bool
rover_project__msg__Proximity__are_equal(const rover_project__msg__Proximity * lhs, const rover_project__msg__Proximity * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // proximity_front
  if (lhs->proximity_front != rhs->proximity_front) {
    return false;
  }
  // proximity_rear
  if (lhs->proximity_rear != rhs->proximity_rear) {
    return false;
  }
  // proximity_cliff
  if (lhs->proximity_cliff != rhs->proximity_cliff) {
    return false;
  }
  // cliff_detected
  if (lhs->cliff_detected != rhs->cliff_detected) {
    return false;
  }
  // front_valid
  if (lhs->front_valid != rhs->front_valid) {
    return false;
  }
  // rear_valid
  if (lhs->rear_valid != rhs->rear_valid) {
    return false;
  }
  // cliff_valid
  if (lhs->cliff_valid != rhs->cliff_valid) {
    return false;
  }
  return true;
}

bool
rover_project__msg__Proximity__copy(
  const rover_project__msg__Proximity * input,
  rover_project__msg__Proximity * output)
{
  if (!input || !output) {
    return false;
  }
  // proximity_front
  output->proximity_front = input->proximity_front;
  // proximity_rear
  output->proximity_rear = input->proximity_rear;
  // proximity_cliff
  output->proximity_cliff = input->proximity_cliff;
  // cliff_detected
  output->cliff_detected = input->cliff_detected;
  // front_valid
  output->front_valid = input->front_valid;
  // rear_valid
  output->rear_valid = input->rear_valid;
  // cliff_valid
  output->cliff_valid = input->cliff_valid;
  return true;
}

rover_project__msg__Proximity *
rover_project__msg__Proximity__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__Proximity * msg = (rover_project__msg__Proximity *)allocator.allocate(sizeof(rover_project__msg__Proximity), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_project__msg__Proximity));
  bool success = rover_project__msg__Proximity__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_project__msg__Proximity__destroy(rover_project__msg__Proximity * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_project__msg__Proximity__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_project__msg__Proximity__Sequence__init(rover_project__msg__Proximity__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__Proximity * data = NULL;

  if (size) {
    data = (rover_project__msg__Proximity *)allocator.zero_allocate(size, sizeof(rover_project__msg__Proximity), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_project__msg__Proximity__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_project__msg__Proximity__fini(&data[i - 1]);
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
rover_project__msg__Proximity__Sequence__fini(rover_project__msg__Proximity__Sequence * array)
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
      rover_project__msg__Proximity__fini(&array->data[i]);
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

rover_project__msg__Proximity__Sequence *
rover_project__msg__Proximity__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__Proximity__Sequence * array = (rover_project__msg__Proximity__Sequence *)allocator.allocate(sizeof(rover_project__msg__Proximity__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_project__msg__Proximity__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_project__msg__Proximity__Sequence__destroy(rover_project__msg__Proximity__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_project__msg__Proximity__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_project__msg__Proximity__Sequence__are_equal(const rover_project__msg__Proximity__Sequence * lhs, const rover_project__msg__Proximity__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_project__msg__Proximity__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_project__msg__Proximity__Sequence__copy(
  const rover_project__msg__Proximity__Sequence * input,
  rover_project__msg__Proximity__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_project__msg__Proximity);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_project__msg__Proximity * data =
      (rover_project__msg__Proximity *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_project__msg__Proximity__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_project__msg__Proximity__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_project__msg__Proximity__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
