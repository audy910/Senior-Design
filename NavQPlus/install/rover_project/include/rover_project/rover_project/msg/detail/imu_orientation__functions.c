// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_project:msg/ImuOrientation.idl
// generated code does not contain a copyright notice
#include "rover_project/msg/detail/imu_orientation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_project__msg__ImuOrientation__init(rover_project__msg__ImuOrientation * msg)
{
  if (!msg) {
    return false;
  }
  // heading
  // pitch
  // roll
  // cal_sys
  // cal_mag
  return true;
}

void
rover_project__msg__ImuOrientation__fini(rover_project__msg__ImuOrientation * msg)
{
  if (!msg) {
    return;
  }
  // heading
  // pitch
  // roll
  // cal_sys
  // cal_mag
}

bool
rover_project__msg__ImuOrientation__are_equal(const rover_project__msg__ImuOrientation * lhs, const rover_project__msg__ImuOrientation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // cal_sys
  if (lhs->cal_sys != rhs->cal_sys) {
    return false;
  }
  // cal_mag
  if (lhs->cal_mag != rhs->cal_mag) {
    return false;
  }
  return true;
}

bool
rover_project__msg__ImuOrientation__copy(
  const rover_project__msg__ImuOrientation * input,
  rover_project__msg__ImuOrientation * output)
{
  if (!input || !output) {
    return false;
  }
  // heading
  output->heading = input->heading;
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // cal_sys
  output->cal_sys = input->cal_sys;
  // cal_mag
  output->cal_mag = input->cal_mag;
  return true;
}

rover_project__msg__ImuOrientation *
rover_project__msg__ImuOrientation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__ImuOrientation * msg = (rover_project__msg__ImuOrientation *)allocator.allocate(sizeof(rover_project__msg__ImuOrientation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_project__msg__ImuOrientation));
  bool success = rover_project__msg__ImuOrientation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_project__msg__ImuOrientation__destroy(rover_project__msg__ImuOrientation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_project__msg__ImuOrientation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_project__msg__ImuOrientation__Sequence__init(rover_project__msg__ImuOrientation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__ImuOrientation * data = NULL;

  if (size) {
    data = (rover_project__msg__ImuOrientation *)allocator.zero_allocate(size, sizeof(rover_project__msg__ImuOrientation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_project__msg__ImuOrientation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_project__msg__ImuOrientation__fini(&data[i - 1]);
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
rover_project__msg__ImuOrientation__Sequence__fini(rover_project__msg__ImuOrientation__Sequence * array)
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
      rover_project__msg__ImuOrientation__fini(&array->data[i]);
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

rover_project__msg__ImuOrientation__Sequence *
rover_project__msg__ImuOrientation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_project__msg__ImuOrientation__Sequence * array = (rover_project__msg__ImuOrientation__Sequence *)allocator.allocate(sizeof(rover_project__msg__ImuOrientation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_project__msg__ImuOrientation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_project__msg__ImuOrientation__Sequence__destroy(rover_project__msg__ImuOrientation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_project__msg__ImuOrientation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_project__msg__ImuOrientation__Sequence__are_equal(const rover_project__msg__ImuOrientation__Sequence * lhs, const rover_project__msg__ImuOrientation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_project__msg__ImuOrientation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_project__msg__ImuOrientation__Sequence__copy(
  const rover_project__msg__ImuOrientation__Sequence * input,
  rover_project__msg__ImuOrientation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_project__msg__ImuOrientation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_project__msg__ImuOrientation * data =
      (rover_project__msg__ImuOrientation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_project__msg__ImuOrientation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_project__msg__ImuOrientation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_project__msg__ImuOrientation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
