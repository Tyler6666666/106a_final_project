// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_msgs:msg/TelloResponse.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__TRAITS_HPP_
#define TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_msgs/msg/detail/tello_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tello_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TelloResponse & msg,
  std::ostream & out)
{
  out << "{";
  // member: rc
  {
    out << "rc: ";
    rosidl_generator_traits::value_to_yaml(msg.rc, out);
    out << ", ";
  }

  // member: str
  {
    out << "str: ";
    rosidl_generator_traits::value_to_yaml(msg.str, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TelloResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rc: ";
    rosidl_generator_traits::value_to_yaml(msg.rc, out);
    out << "\n";
  }

  // member: str
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "str: ";
    rosidl_generator_traits::value_to_yaml(msg.str, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TelloResponse & msg, bool use_flow_style = false)
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

}  // namespace tello_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tello_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tello_msgs::msg::TelloResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_msgs::msg::TelloResponse & msg)
{
  return tello_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_msgs::msg::TelloResponse>()
{
  return "tello_msgs::msg::TelloResponse";
}

template<>
inline const char * name<tello_msgs::msg::TelloResponse>()
{
  return "tello_msgs/msg/TelloResponse";
}

template<>
struct has_fixed_size<tello_msgs::msg::TelloResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_msgs::msg::TelloResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_msgs::msg::TelloResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_MSGS__MSG__DETAIL__TELLO_RESPONSE__TRAITS_HPP_
