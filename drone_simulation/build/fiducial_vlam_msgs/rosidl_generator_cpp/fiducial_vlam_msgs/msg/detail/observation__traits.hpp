// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fiducial_vlam_msgs:msg/Observation.idl
// generated code does not contain a copyright notice

#ifndef FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATION__TRAITS_HPP_
#define FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fiducial_vlam_msgs/msg/detail/observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace fiducial_vlam_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Observation & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: x0
  {
    out << "x0: ";
    rosidl_generator_traits::value_to_yaml(msg.x0, out);
    out << ", ";
  }

  // member: y0
  {
    out << "y0: ";
    rosidl_generator_traits::value_to_yaml(msg.y0, out);
    out << ", ";
  }

  // member: x1
  {
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << ", ";
  }

  // member: y1
  {
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << ", ";
  }

  // member: x2
  {
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << ", ";
  }

  // member: y2
  {
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << ", ";
  }

  // member: x3
  {
    out << "x3: ";
    rosidl_generator_traits::value_to_yaml(msg.x3, out);
    out << ", ";
  }

  // member: y3
  {
    out << "y3: ";
    rosidl_generator_traits::value_to_yaml(msg.y3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Observation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: x0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x0: ";
    rosidl_generator_traits::value_to_yaml(msg.x0, out);
    out << "\n";
  }

  // member: y0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y0: ";
    rosidl_generator_traits::value_to_yaml(msg.y0, out);
    out << "\n";
  }

  // member: x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << "\n";
  }

  // member: y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << "\n";
  }

  // member: x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << "\n";
  }

  // member: y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << "\n";
  }

  // member: x3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x3: ";
    rosidl_generator_traits::value_to_yaml(msg.x3, out);
    out << "\n";
  }

  // member: y3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y3: ";
    rosidl_generator_traits::value_to_yaml(msg.y3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Observation & msg, bool use_flow_style = false)
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
  const fiducial_vlam_msgs::msg::Observation & msg,
  std::ostream & out, size_t indentation = 0)
{
  fiducial_vlam_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fiducial_vlam_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const fiducial_vlam_msgs::msg::Observation & msg)
{
  return fiducial_vlam_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<fiducial_vlam_msgs::msg::Observation>()
{
  return "fiducial_vlam_msgs::msg::Observation";
}

template<>
inline const char * name<fiducial_vlam_msgs::msg::Observation>()
{
  return "fiducial_vlam_msgs/msg/Observation";
}

template<>
struct has_fixed_size<fiducial_vlam_msgs::msg::Observation>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<fiducial_vlam_msgs::msg::Observation>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<fiducial_vlam_msgs::msg::Observation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FIDUCIAL_VLAM_MSGS__MSG__DETAIL__OBSERVATION__TRAITS_HPP_
