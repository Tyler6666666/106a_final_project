#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to fiducial_vlam_msgs__msg__Map
/// A list of the markers in the environment

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Map {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Length in meters of a side of all markers
    pub marker_length: f64,

    /// id and pose of the markers
    pub fixed_flags: Vec<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ids: Vec<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: Vec<geometry_msgs::msg::PoseWithCovariance>,

}



impl Default for Map {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Map::default())
  }
}

impl rosidl_runtime_rs::Message for Map {
  type RmwMsg = super::msg::rmw::Map;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        marker_length: msg.marker_length,
        fixed_flags: msg.fixed_flags.into(),
        ids: msg.ids.into(),
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      marker_length: msg.marker_length,
        fixed_flags: msg.fixed_flags.as_slice().into(),
        ids: msg.ids.as_slice().into(),
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      marker_length: msg.marker_length,
      fixed_flags: msg.fixed_flags
          .into_iter()
          .collect(),
      ids: msg.ids
          .into_iter()
          .collect(),
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseWithCovariance::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to fiducial_vlam_msgs__msg__Observation
/// An observation of a marker.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Observation::default())
  }
}

impl rosidl_runtime_rs::Message for Observation {
  type RmwMsg = super::msg::rmw::Observation;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        x0: msg.x0,
        y0: msg.y0,
        x1: msg.x1,
        y1: msg.y1,
        x2: msg.x2,
        y2: msg.y2,
        x3: msg.x3,
        y3: msg.y3,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      x0: msg.x0,
      y0: msg.y0,
      x1: msg.x1,
      y1: msg.y1,
      x2: msg.x2,
      y2: msg.y2,
      x3: msg.x3,
      y3: msg.y3,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      x0: msg.x0,
      y0: msg.y0,
      x1: msg.x1,
      y1: msg.y1,
      x2: msg.x2,
      y2: msg.y2,
      x3: msg.x3,
      y3: msg.y3,
    }
  }
}


// Corresponds to fiducial_vlam_msgs__msg__Observations
/// An observation of several markers

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Observations {
    /// the header from the image
    pub header: std_msgs::msg::Header,

    /// the CameraInfo for the camera that captured the image
    pub camera_info: sensor_msgs::msg::CameraInfo,

    /// A list of locations of marker corners in the image
    pub observations: Vec<super::msg::Observation>,

}



impl Default for Observations {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Observations::default())
  }
}

impl rosidl_runtime_rs::Message for Observations {
  type RmwMsg = super::msg::rmw::Observations;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        camera_info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Owned(msg.camera_info)).into_owned(),
        observations: msg.observations
          .into_iter()
          .map(|elem| super::msg::Observation::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        camera_info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.camera_info)).into_owned(),
        observations: msg.observations
          .iter()
          .map(|elem| super::msg::Observation::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      camera_info: sensor_msgs::msg::CameraInfo::from_rmw_message(msg.camera_info),
      observations: msg.observations
          .into_iter()
          .map(super::msg::Observation::from_rmw_message)
          .collect(),
    }
  }
}


