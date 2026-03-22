#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "fiducial_vlam_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Map() -> *const std::ffi::c_void;
}

#[link(name = "fiducial_vlam_msgs__rosidl_generator_c")]
extern "C" {
    fn fiducial_vlam_msgs__msg__Map__init(msg: *mut Map) -> bool;
    fn fiducial_vlam_msgs__msg__Map__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Map>, size: usize) -> bool;
    fn fiducial_vlam_msgs__msg__Map__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Map>);
    fn fiducial_vlam_msgs__msg__Map__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Map>, out_seq: *mut rosidl_runtime_rs::Sequence<Map>) -> bool;
}

// Corresponds to fiducial_vlam_msgs__msg__Map
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A list of the markers in the environment

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Map {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Length in meters of a side of all markers
    pub marker_length: f64,

    /// id and pose of the markers
    pub fixed_flags: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ids: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseWithCovariance>,

}



impl Default for Map {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fiducial_vlam_msgs__msg__Map__init(&mut msg as *mut _) {
        panic!("Call to fiducial_vlam_msgs__msg__Map__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Map {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Map__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Map__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Map__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Map {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Map where Self: Sized {
  const TYPE_NAME: &'static str = "fiducial_vlam_msgs/msg/Map";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Map() }
  }
}


#[link(name = "fiducial_vlam_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Observation() -> *const std::ffi::c_void;
}

#[link(name = "fiducial_vlam_msgs__rosidl_generator_c")]
extern "C" {
    fn fiducial_vlam_msgs__msg__Observation__init(msg: *mut Observation) -> bool;
    fn fiducial_vlam_msgs__msg__Observation__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Observation>, size: usize) -> bool;
    fn fiducial_vlam_msgs__msg__Observation__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Observation>);
    fn fiducial_vlam_msgs__msg__Observation__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Observation>, out_seq: *mut rosidl_runtime_rs::Sequence<Observation>) -> bool;
}

// Corresponds to fiducial_vlam_msgs__msg__Observation
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// An observation of a marker.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Observation {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i32,

    /// vertices
    pub x0: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y0: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x3: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y3: f64,

}



impl Default for Observation {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fiducial_vlam_msgs__msg__Observation__init(&mut msg as *mut _) {
        panic!("Call to fiducial_vlam_msgs__msg__Observation__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Observation {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observation__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observation__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observation__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Observation {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Observation where Self: Sized {
  const TYPE_NAME: &'static str = "fiducial_vlam_msgs/msg/Observation";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Observation() }
  }
}


#[link(name = "fiducial_vlam_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Observations() -> *const std::ffi::c_void;
}

#[link(name = "fiducial_vlam_msgs__rosidl_generator_c")]
extern "C" {
    fn fiducial_vlam_msgs__msg__Observations__init(msg: *mut Observations) -> bool;
    fn fiducial_vlam_msgs__msg__Observations__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Observations>, size: usize) -> bool;
    fn fiducial_vlam_msgs__msg__Observations__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Observations>);
    fn fiducial_vlam_msgs__msg__Observations__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Observations>, out_seq: *mut rosidl_runtime_rs::Sequence<Observations>) -> bool;
}

// Corresponds to fiducial_vlam_msgs__msg__Observations
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// An observation of several markers

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Observations {
    /// the header from the image
    pub header: std_msgs::msg::rmw::Header,

    /// the CameraInfo for the camera that captured the image
    pub camera_info: sensor_msgs::msg::rmw::CameraInfo,

    /// A list of locations of marker corners in the image
    pub observations: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Observation>,

}



impl Default for Observations {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fiducial_vlam_msgs__msg__Observations__init(&mut msg as *mut _) {
        panic!("Call to fiducial_vlam_msgs__msg__Observations__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Observations {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observations__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observations__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fiducial_vlam_msgs__msg__Observations__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Observations {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Observations where Self: Sized {
  const TYPE_NAME: &'static str = "fiducial_vlam_msgs/msg/Observations";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fiducial_vlam_msgs__msg__Observations() }
  }
}


