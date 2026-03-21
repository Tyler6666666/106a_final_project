// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_msgs:msg/FlightData.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__TRAITS_HPP_
#define TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_msgs/msg/detail/flight_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tello_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FlightData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: raw
  {
    out << "raw: ";
    rosidl_generator_traits::value_to_yaml(msg.raw, out);
    out << ", ";
  }

  // member: sdk
  {
    out << "sdk: ";
    rosidl_generator_traits::value_to_yaml(msg.sdk, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: vgx
  {
    out << "vgx: ";
    rosidl_generator_traits::value_to_yaml(msg.vgx, out);
    out << ", ";
  }

  // member: vgy
  {
    out << "vgy: ";
    rosidl_generator_traits::value_to_yaml(msg.vgy, out);
    out << ", ";
  }

  // member: vgz
  {
    out << "vgz: ";
    rosidl_generator_traits::value_to_yaml(msg.vgz, out);
    out << ", ";
  }

  // member: templ
  {
    out << "templ: ";
    rosidl_generator_traits::value_to_yaml(msg.templ, out);
    out << ", ";
  }

  // member: temph
  {
    out << "temph: ";
    rosidl_generator_traits::value_to_yaml(msg.temph, out);
    out << ", ";
  }

  // member: tof
  {
    out << "tof: ";
    rosidl_generator_traits::value_to_yaml(msg.tof, out);
    out << ", ";
  }

  // member: h
  {
    out << "h: ";
    rosidl_generator_traits::value_to_yaml(msg.h, out);
    out << ", ";
  }

  // member: bat
  {
    out << "bat: ";
    rosidl_generator_traits::value_to_yaml(msg.bat, out);
    out << ", ";
  }

  // member: baro
  {
    out << "baro: ";
    rosidl_generator_traits::value_to_yaml(msg.baro, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: agx
  {
    out << "agx: ";
    rosidl_generator_traits::value_to_yaml(msg.agx, out);
    out << ", ";
  }

  // member: agy
  {
    out << "agy: ";
    rosidl_generator_traits::value_to_yaml(msg.agy, out);
    out << ", ";
  }

  // member: agz
  {
    out << "agz: ";
    rosidl_generator_traits::value_to_yaml(msg.agz, out);
    out << ", ";
  }

  // member: mid
  {
    out << "mid: ";
    rosidl_generator_traits::value_to_yaml(msg.mid, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FlightData & msg,
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

  // member: raw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw: ";
    rosidl_generator_traits::value_to_yaml(msg.raw, out);
    out << "\n";
  }

  // member: sdk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sdk: ";
    rosidl_generator_traits::value_to_yaml(msg.sdk, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: vgx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vgx: ";
    rosidl_generator_traits::value_to_yaml(msg.vgx, out);
    out << "\n";
  }

  // member: vgy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vgy: ";
    rosidl_generator_traits::value_to_yaml(msg.vgy, out);
    out << "\n";
  }

  // member: vgz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vgz: ";
    rosidl_generator_traits::value_to_yaml(msg.vgz, out);
    out << "\n";
  }

  // member: templ
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "templ: ";
    rosidl_generator_traits::value_to_yaml(msg.templ, out);
    out << "\n";
  }

  // member: temph
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temph: ";
    rosidl_generator_traits::value_to_yaml(msg.temph, out);
    out << "\n";
  }

  // member: tof
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tof: ";
    rosidl_generator_traits::value_to_yaml(msg.tof, out);
    out << "\n";
  }

  // member: h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h: ";
    rosidl_generator_traits::value_to_yaml(msg.h, out);
    out << "\n";
  }

  // member: bat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bat: ";
    rosidl_generator_traits::value_to_yaml(msg.bat, out);
    out << "\n";
  }

  // member: baro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baro: ";
    rosidl_generator_traits::value_to_yaml(msg.baro, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }

  // member: agx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "agx: ";
    rosidl_generator_traits::value_to_yaml(msg.agx, out);
    out << "\n";
  }

  // member: agy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "agy: ";
    rosidl_generator_traits::value_to_yaml(msg.agy, out);
    out << "\n";
  }

  // member: agz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "agz: ";
    rosidl_generator_traits::value_to_yaml(msg.agz, out);
    out << "\n";
  }

  // member: mid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mid: ";
    rosidl_generator_traits::value_to_yaml(msg.mid, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FlightData & msg, bool use_flow_style = false)
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
  const tello_msgs::msg::FlightData & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_msgs::msg::FlightData & msg)
{
  return tello_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_msgs::msg::FlightData>()
{
  return "tello_msgs::msg::FlightData";
}

template<>
inline const char * name<tello_msgs::msg::FlightData>()
{
  return "tello_msgs/msg/FlightData";
}

template<>
struct has_fixed_size<tello_msgs::msg::FlightData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_msgs::msg::FlightData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_msgs::msg::FlightData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_MSGS__MSG__DETAIL__FLIGHT_DATA__TRAITS_HPP_
