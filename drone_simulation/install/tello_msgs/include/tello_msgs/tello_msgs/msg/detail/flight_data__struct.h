// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_msgs:msg/FlightData.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__STRUCT_H_
#define TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SDK_UNKNOWN'.
/**
  * SDK version
 */
enum
{
  tello_msgs__msg__FlightData__SDK_UNKNOWN = 0
};

/// Constant 'SDK_1_3'.
enum
{
  tello_msgs__msg__FlightData__SDK_1_3 = 1
};

/// Constant 'SDK_2_0'.
enum
{
  tello_msgs__msg__FlightData__SDK_2_0 = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'raw'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FlightData in the package tello_msgs.
typedef struct tello_msgs__msg__FlightData
{
  std_msgs__msg__Header header;
  /// Raw string
  rosidl_runtime_c__String raw;
  uint8_t sdk;
  /// SDK 1.3+ fields
  ///
  /// pitch in degrees
  int32_t pitch;
  /// roll in degrees
  int32_t roll;
  /// yaw in degrees relative to startup orientation
  int32_t yaw;
  /// x velocity in m/s, +forward
  int32_t vgx;
  /// y velocity in m/s, +right
  int32_t vgy;
  /// z velocity in m/s, +down
  int32_t vgz;
  /// lowest temp in C
  int32_t templ;
  /// highest temp in C
  int32_t temph;
  /// time of flight distance in cm
  int32_t tof;
  /// height in cm
  int32_t h;
  /// battery %
  int32_t bat;
  /// barometric altitude in m
  float baro;
  /// time used by motor in seconds
  int32_t time;
  /// Acceleration includes gravity
  /// x acceleration in mm/s^2, +forward
  float agx;
  /// y acceleration in mm/s^2, +right
  float agy;
  /// z acceleration in mm/s^2, +down
  float agz;
  /// SDK 2.0+ fields
  ///
  /// mission pad id, or -1 if not found
  int32_t mid;
  /// x coord relative to mission pad in cm, or 0 if not found
  int32_t x;
  /// y coord relative to mission pad in cm, or 0 if not found
  int32_t y;
  /// z coord relative to mission pad in cm, or 0 if not found
  int32_t z;
} tello_msgs__msg__FlightData;

// Struct for a sequence of tello_msgs__msg__FlightData.
typedef struct tello_msgs__msg__FlightData__Sequence
{
  tello_msgs__msg__FlightData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_msgs__msg__FlightData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__STRUCT_H_
