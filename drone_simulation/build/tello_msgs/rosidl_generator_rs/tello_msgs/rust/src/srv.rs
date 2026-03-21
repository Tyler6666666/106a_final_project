#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to tello_msgs__srv__TelloAction_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TelloAction_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub cmd: std::string::String,

}



impl Default for TelloAction_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TelloAction_Request::default())
  }
}

impl rosidl_runtime_rs::Message for TelloAction_Request {
  type RmwMsg = super::srv::rmw::TelloAction_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd: msg.cmd.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd: msg.cmd.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cmd: msg.cmd.to_string(),
    }
  }
}


// Corresponds to tello_msgs__srv__TelloAction_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TelloAction_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub rc: u8,

}

impl TelloAction_Response {
    /// Command sent
    pub const OK: u8 = 1;

    /// Can't communicate with drone
    pub const ERROR_NOT_CONNECTED: u8 = 2;

    /// There's already an active command
    pub const ERROR_BUSY: u8 = 3;

}


impl Default for TelloAction_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::TelloAction_Response::default())
  }
}

impl rosidl_runtime_rs::Message for TelloAction_Response {
  type RmwMsg = super::srv::rmw::TelloAction_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        rc: msg.rc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      rc: msg.rc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      rc: msg.rc,
    }
  }
}






#[link(name = "tello_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__tello_msgs__srv__TelloAction() -> *const std::ffi::c_void;
}

// Corresponds to tello_msgs__srv__TelloAction
#[allow(missing_docs, non_camel_case_types)]
pub struct TelloAction;

impl rosidl_runtime_rs::Service for TelloAction {
    type Request = TelloAction_Request;
    type Response = TelloAction_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__tello_msgs__srv__TelloAction() }
    }
}


