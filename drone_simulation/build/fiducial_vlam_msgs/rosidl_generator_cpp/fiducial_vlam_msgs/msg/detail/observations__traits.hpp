// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fiducial_vlam_msgs:msg/Observations.idl
// generated code does not contain a copyright notice

#ifndef FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATIONS__TRAITS_HPP_
#define FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fiducial_vlam_msgs/msg/detail/observations__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"
// Member 'observations'
#include "fiducial_vlam_msgs/msg/detail/observation__traits.hpp"

namespace fiducial_vlam_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Observations & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: camera_info
  {
    out << "camera_info: ";
    to_flow_style_yaml(msg.camera_info, out);
    out << ", ";
  }

  // member: observations
  {
    if (msg.observations.size() == 0) {
      out << "observations: []";
    } else {
      out << "observations: [";
      size_t pending_items = msg.observations.size();
      for (auto item : msg.observations) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Observations & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: camera_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_info:\n";
    to_block_style_yaml(msg.camera_info, out, indentation + 2);
  }

  // member: observations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.observations.size() == 0) {
      out << "observations: []\n";
    } else {
      out << "observations:\n";
      for (auto item : msg.observations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Observations & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace fiducial_vlam_msgs

namespace rosidl_generator_traits
{

[[deprecated("use fiducial_vlam_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fiducial_vlam_msgs::msg::Observations & msg,
  std::ostream & out, size_t indentation = 0)
{
  fiducial_vlam_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fiducial_vlam_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const fiducial_vlam_msgs::msg::Observations & msg)
{
  return fiducial_vlam_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<fiducial_vlam_msgs::msg::Observations>()
{
  return "fiducial_vlam_msgs::msg::Observations";
}

template<>
inline const char * name<fiducial_vlam_msgs::msg::Observations>()
{
  return "fiducial_vlam_msgs/msg/Observations";
}

template<>
struct has_fixed_size<fiducial_vlam_msgs::msg::Observations>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fiducial_vlam_msgs::msg::Observations>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fiducial_vlam_msgs::msg::Observations>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATIONS__TRAITS_HPP_
