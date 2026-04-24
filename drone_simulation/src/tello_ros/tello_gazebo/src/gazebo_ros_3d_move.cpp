#include <mutex>
#include <string>
#include <algorithm>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace tello_gazebo
{

class GazeboRos3DMove : public gazebo::ModelPlugin
{
public:
  GazeboRos3DMove() = default;
  ~GazeboRos3DMove() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    GZ_ASSERT(model != nullptr, "Model pointer is null");
    GZ_ASSERT(sdf != nullptr, "SDF pointer is null");

    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    std::string link_name;
    if (sdf->HasElement("link_name")) {
      link_name = sdf->Get<std::string>("link_name");
    } else if (!model_->GetLinks().empty()) {
      link_name = model_->GetLinks().front()->GetName();
    }

    base_link_ = model_->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing base link for GazeboRos3DMove");

    if (sdf->HasElement("use_body_frame")) {
      use_body_frame_ = sdf->Get<bool>("use_body_frame");
    }
    if (sdf->HasElement("min_z")) {
      min_z_ = sdf->Get<double>("min_z");
    }
    if (sdf->HasElement("max_z")) {
      max_z_ = sdf->Get<double>("max_z");
    }
    if (sdf->HasElement("vertical_recovery_gain")) {
      vertical_recovery_gain_ = sdf->Get<double>("vertical_recovery_gain");
    }
    if (sdf->HasElement("max_vertical_speed")) {
      max_vertical_speed_ = sdf->Get<double>("max_vertical_speed");
    }

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(10),
      std::bind(&GazeboRos3DMove::OnCmdVel, this, std::placeholders::_1));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRos3DMove::OnUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "GazeboRos3DMove loaded for model '%s' on link '%s' (use_body_frame=%s, min_z=%.2f, max_z=%.2f).",
      model_->GetName().c_str(),
      base_link_->GetName().c_str(),
      use_body_frame_ ? "true" : "false",
      min_z_,
      max_z_);
  }

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_linear_.X() = msg->linear.x;
    target_linear_.Y() = msg->linear.y;
    target_linear_.Z() = msg->linear.z;
    target_yaw_rate_ = msg->angular.z;
  }

  void OnUpdate()
  {
    ignition::math::Vector3d linear;
    double yaw_rate;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear = target_linear_;
      yaw_rate = target_yaw_rate_;
    }

    if (use_body_frame_) {
      const auto pose = base_link_->WorldPose();
      linear = pose.Rot().RotateVector(linear);
    }

    const double current_z = base_link_->WorldPose().Pos().Z();
    if (current_z < min_z_) {
      const double recovery_vz = std::min(max_vertical_speed_, vertical_recovery_gain_ * (min_z_ - current_z));
      linear.Z() = std::max(linear.Z(), recovery_vz);
    } else if (current_z > max_z_) {
      const double recovery_vz = std::min(max_vertical_speed_, vertical_recovery_gain_ * (current_z - max_z_));
      linear.Z() = std::min(linear.Z(), -recovery_vz);
    }
    linear.Z() = std::clamp(linear.Z(), -max_vertical_speed_, max_vertical_speed_);

    base_link_->SetLinearVel(linear);
    base_link_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, yaw_rate));
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_link_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  gazebo::event::ConnectionPtr update_connection_;

  std::mutex mutex_;
  ignition::math::Vector3d target_linear_{0.0, 0.0, 0.0};
  double target_yaw_rate_{0.0};
  bool use_body_frame_{false};
  double min_z_{0.8};
  double max_z_{1.8};
  double vertical_recovery_gain_{2.0};
  double max_vertical_speed_{0.6};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRos3DMove)

}  // namespace tello_gazebo
