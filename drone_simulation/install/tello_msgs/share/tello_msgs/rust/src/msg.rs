#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to tello_msgs__msg__FlightData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FlightData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Raw string
    pub raw: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FlightData::default())
  }
}

impl rosidl_runtime_rs::Message for FlightData {
  type RmwMsg = super::msg::rmw::FlightData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        raw: msg.raw.as_str().into(),
        sdk: msg.sdk,
        pitch: msg.pitch,
        roll: msg.roll,
        yaw: msg.yaw,
        vgx: msg.vgx,
        vgy: msg.vgy,
        vgz: msg.vgz,
        templ: msg.templ,
        temph: msg.temph,
        tof: msg.tof,
        h: msg.h,
        bat: msg.bat,
        baro: msg.baro,
        time: msg.time,
        agx: msg.agx,
        agy: msg.agy,
        agz: msg.agz,
        mid: msg.mid,
        x: msg.x,
        y: msg.y,
        z: msg.z,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        raw: msg.raw.as_str().into(),
      sdk: msg.sdk,
      pitch: msg.pitch,
      roll: msg.roll,
      yaw: msg.yaw,
      vgx: msg.vgx,
      vgy: msg.vgy,
      vgz: msg.vgz,
      templ: msg.templ,
      temph: msg.temph,
      tof: msg.tof,
      h: msg.h,
      bat: msg.bat,
      baro: msg.baro,
      time: msg.time,
      agx: msg.agx,
      agy: msg.agy,
      agz: msg.agz,
      mid: msg.mid,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      raw: msg.raw.to_string(),
      sdk: msg.sdk,
      pitch: msg.pitch,
      roll: msg.roll,
      yaw: msg.yaw,
      vgx: msg.vgx,
      vgy: msg.vgy,
      vgz: msg.vgz,
      templ: msg.templ,
      temph: msg.temph,
      tof: msg.tof,
      h: msg.h,
      bat: msg.bat,
      baro: msg.baro,
      time: msg.time,
      agx: msg.agx,
      agy: msg.agy,
      agz: msg.agz,
      mid: msg.mid,
      x: msg.x,
      y: msg.y,
      z: msg.z,
    }
  }
}


// Corresponds to tello_msgs__msg__TelloResponse
/// Final response code:

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TelloResponse {

    // This member is not documented.
    #[allow(missing_docs)]
    pub rc: u8,

    /// Full text of the response:
    pub str: std::string::String,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TelloResponse::default())
  }
}

impl rosidl_runtime_rs::Message for TelloResponse {
  type RmwMsg = super::msg::rmw::TelloResponse;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        rc: msg.rc,
        str: msg.str.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      rc: msg.rc,
        str: msg.str.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      rc: msg.rc,
      str: msg.str.to_string(),
    }
  }
}


