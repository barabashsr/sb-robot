// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression11.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__FUNCTIONS_H_
#define ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_typesupport_microxrcedds_test_msg/msg/rosidl_generator_c__visibility_control.h"

#include "rosidl_typesupport_microxrcedds_test_msg/msg/detail/regression11__struct.h"

/// Initialize msg/Regression11 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11
 * )) before or use
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__init(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * msg);

/// Finalize msg/Regression11 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * msg);

/// Create msg/Regression11 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 *
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__create();

/// Destroy msg/Regression11 message.
/**
 * It calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * msg);

/// Check for msg/Regression11 message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * rhs);

/// Copy a msg/Regression11 message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11 * output);

/// Initialize array of msg/Regression11 messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__init(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * array, size_t size);

/// Finalize array of msg/Regression11 messages.
/**
 * It calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__fini(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * array);

/// Create array of msg/Regression11 messages.
/**
 * It allocates the memory for the array and calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence *
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__create(size_t size);

/// Destroy array of msg/Regression11 messages.
/**
 * It calls
 * rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
void
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__destroy(rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * array);

/// Check for msg/Regression11 message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__are_equal(const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * lhs, const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * rhs);

/// Copy an array of msg/Regression11 messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_typesupport_microxrcedds_test_msg
bool
rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence__copy(
  const rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * input,
  rosidl_typesupport_microxrcedds_test_msg__msg__Regression11__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_TYPESUPPORT_MICROXRCEDDS_TEST_MSG__MSG__DETAIL__REGRESSION11__FUNCTIONS_H_
