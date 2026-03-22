// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fiducial_vlam_msgs:msg/Observations.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fiducial_vlam_msgs/msg/detail/observations__rosidl_typesupport_introspection_c.h"
#include "fiducial_vlam_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fiducial_vlam_msgs/msg/detail/observations__functions.h"
#include "fiducial_vlam_msgs/msg/detail/observations__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `camera_info`
#include "sensor_msgs/msg/camera_info.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__rosidl_typesupport_introspection_c.h"
// Member `observations`
#include "fiducial_vlam_msgs/msg/observation.h"
// Member `observations`
#include "fiducial_vlam_msgs/msg/detail/observation__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fiducial_vlam_msgs__msg__Observations__init(message_memory);
}

void fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_fini_function(void * message_memory)
{
  fiducial_vlam_msgs__msg__Observations__fini(message_memory);
}

size_t fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__size_function__Observations__observations(
  const void * untyped_member)
{
  const fiducial_vlam_msgs__msg__Observation__Sequence * member =
    (const fiducial_vlam_msgs__msg__Observation__Sequence *)(untyped_member);
  return member->size;
}

const void * fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_const_function__Observations__observations(
  const void * untyped_member, size_t index)
{
  const fiducial_vlam_msgs__msg__Observation__Sequence * member =
    (const fiducial_vlam_msgs__msg__Observation__Sequence *)(untyped_member);
  return &member->data[index];
}

void * fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_function__Observations__observations(
  void * untyped_member, size_t index)
{
  fiducial_vlam_msgs__msg__Observation__Sequence * member =
    (fiducial_vlam_msgs__msg__Observation__Sequence *)(untyped_member);
  return &member->data[index];
}

void fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__fetch_function__Observations__observations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const fiducial_vlam_msgs__msg__Observation * item =
    ((const fiducial_vlam_msgs__msg__Observation *)
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_const_function__Observations__observations(untyped_member, index));
  fiducial_vlam_msgs__msg__Observation * value =
    (fiducial_vlam_msgs__msg__Observation *)(untyped_value);
  *value = *item;
}

void fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__assign_function__Observations__observations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  fiducial_vlam_msgs__msg__Observation * item =
    ((fiducial_vlam_msgs__msg__Observation *)
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_function__Observations__observations(untyped_member, index));
  const fiducial_vlam_msgs__msg__Observation * value =
    (const fiducial_vlam_msgs__msg__Observation *)(untyped_value);
  *item = *value;
}

bool fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__resize_function__Observations__observations(
  void * untyped_member, size_t size)
{
  fiducial_vlam_msgs__msg__Observation__Sequence * member =
    (fiducial_vlam_msgs__msg__Observation__Sequence *)(untyped_member);
  fiducial_vlam_msgs__msg__Observation__Sequence__fini(member);
  return fiducial_vlam_msgs__msg__Observation__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fiducial_vlam_msgs__msg__Observations, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fiducial_vlam_msgs__msg__Observations, camera_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "observations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fiducial_vlam_msgs__msg__Observations, observations),  // bytes offset in struct
    NULL,  // default value
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__size_function__Observations__observations,  // size() function pointer
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_const_function__Observations__observations,  // get_const(index) function pointer
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__get_function__Observations__observations,  // get(index) function pointer
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__fetch_function__Observations__observations,  // fetch(index, &value) function pointer
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__assign_function__Observations__observations,  // assign(index, value) function pointer
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__resize_function__Observations__observations  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_members = {
  "fiducial_vlam_msgs__msg",  // message namespace
  "Observations",  // message name
  3,  // number of fields
  sizeof(fiducial_vlam_msgs__msg__Observations),
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_member_array,  // message members
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_init_function,  // function to initialize message memory (memory has to be allocated)
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_type_support_handle = {
  0,
  &fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fiducial_vlam_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fiducial_vlam_msgs, msg, Observations)() {
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CameraInfo)();
  fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fiducial_vlam_msgs, msg, Observation)();
  if (!fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_type_support_handle.typesupport_identifier) {
    fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fiducial_vlam_msgs__msg__Observations__rosidl_typesupport_introspection_c__Observations_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
