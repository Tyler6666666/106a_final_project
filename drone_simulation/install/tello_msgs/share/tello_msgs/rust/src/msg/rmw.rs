#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "tello_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_msgs__msg__FlightData() -> *const std::ffi::c_void;
}

#[link(name = "tello_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_msgs__msg__FlightData__init(msg: *mut FlightData) -> bool;
    fn tello_msgs__msg__FlightData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FlightData>, size: usize) -> bool;
    fn tello_msgs__msg__FlightData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FlightData>);
    fn tello_msgs__msg__FlightData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FlightData>, out_seq: *mut rosidl_runtime_rs::Sequence<FlightData>) -> bool;
}

// Corresponds to tello_msgs__msg__FlightData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FlightData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Raw string
    pub raw: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdk: u8,

    /// SDK 1.3+ fields
    ///
    /// pitch in degrees
    pub pitch: i32,

    /// roll in degrees
    pub roll: i32,

    /// yaw in degrees relative to startup orientation
    pub yaw: i32,

    /// x velocity in m/s, +forward
    pub vgx: i32,

    /// y velocity in m/s, +right
    pub vgy: i32,

    /// z velocity in m/s, +down
    pub vgz: i32,

    /// lowest temp in C
    pub templ: i32,

    /// highest temp in C
    pub temph: i32,

    /// time of flight distance in cm
    pub tof: i32,

    /// height in cm
    pub h: i32,

    /// battery %
    pub bat: i32,

    /// barometric altitude in m
    pub baro: f32,

    /// time used by motor in seconds
    pub time: i32,

    /// Acceleration includes gravity
    /// x acceleration in mm/s^2, +forward
    pub agx: f32,

    /// y acceleration in mm/s^2, +right
    pub agy: f32,

    /// z acceleration in mm/s^2, +down
    pub agz: f32,

    /// SDK 2.0+ fields
    ///
    /// mission pad id, or -1 if not found
    pub mid: i32,

    /// x coord relative to mission pad in cm, or 0 if not found
    pub x: i32,

    /// y coord relative to mission pad in cm, or 0 if not found
    pub y: i32,

    /// z coord relative to mission pad in cm, or 0 if not found
    pub z: i32,

}

impl FlightData {
    /// SDK version
    pub const SDK_UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SDK_1_3: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SDK_2_0: u8 = 2;

}


impl Default for FlightData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_msgs__msg__FlightData__init(&mut msg as *mut _) {
        panic!("Call to tello_msgs__msg__FlightData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FlightData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__FlightData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__FlightData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__FlightData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FlightData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FlightData where Self: Sized {
  const TYPE_NAME: &'static str = "tello_msgs/msg/FlightData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_msgs__msg__FlightData() }
  }
}


#[link(name = "tello_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_msgs__msg__TelloResponse() -> *const std::ffi::c_void;
}

#[link(name = "tello_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_msgs__msg__TelloResponse__init(msg: *mut TelloResponse) -> bool;
    fn tello_msgs__msg__TelloResponse__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TelloResponse>, size: usize) -> bool;
    fn tello_msgs__msg__TelloResponse__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TelloResponse>);
    fn tello_msgs__msg__TelloResponse__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TelloResponse>, out_seq: *mut rosidl_runtime_rs::Sequence<TelloResponse>) -> bool;
}

// Corresponds to tello_msgs__msg__TelloResponse
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Final response code:

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TelloResponse {

    // This member is not documented.
    #[allow(missing_docs)]
    pub rc: u8,

    /// Full text of the response:
    pub str: rosidl_runtime_rs::String,

}

impl TelloResponse {
    /// Response was anything except 'error'
    pub const OK: u8 = 1;

    /// Response was 'error'
    pub const ERROR: u8 = 2;

    /// No response
    pub const TIMEOUT: u8 = 3;

}


impl Default for TelloResponse {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_msgs__msg__TelloResponse__init(&mut msg as *mut _) {
        panic!("Call to tello_msgs__msg__TelloResponse__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TelloResponse {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__TelloResponse__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__TelloResponse__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_msgs__msg__TelloResponse__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TelloResponse {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TelloResponse where Self: Sized {
  const TYPE_NAME: &'static str = "tello_msgs/msg/TelloResponse";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_msgs__msg__TelloResponse() }
  }
}


