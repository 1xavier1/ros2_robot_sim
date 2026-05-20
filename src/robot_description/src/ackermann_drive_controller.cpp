#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace gazebo
{

class AckermannDriveControllerPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  struct Targets
  {
    double left_steer = 0.0;
    double right_steer = 0.0;
    double rear_left_velocity = 0.0;
    double rear_right_velocity = 0.0;
  };

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void OnUpdate();
  Targets ComputeTargets(double linear_x, double angular_z) const;
  void ApplyTargets(const Targets & targets);
  void PublishOdometry(const common::Time & sim_time);
  double ReadDouble(sdf::ElementPtr sdf, const std::string & name, double fallback) const;
  std::string ReadString(
    sdf::ElementPtr sdf, const std::string & name, const std::string & fallback) const;

  physics::ModelPtr model_;
  physics::JointPtr left_steering_joint_;
  physics::JointPtr right_steering_joint_;
  physics::JointPtr rear_left_wheel_joint_;
  physics::JointPtr rear_right_wheel_joint_;
  event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_footprint";
  double wheelbase_ = 0.45;
  double track_width_ = 0.35;
  double wheel_radius_ = 0.07;
  double max_steering_angle_ = 0.5236;
  double cmd_timeout_ = 0.5;
  double update_period_ = 0.02;
  double max_wheel_torque_ = 50.0;

  double target_linear_ = 0.0;
  double target_angular_ = 0.0;
  common::Time last_cmd_time_;
  common::Time last_update_time_;
};

double AckermannDriveControllerPlugin::ReadDouble(
  sdf::ElementPtr sdf, const std::string & name, double fallback) const
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}

std::string AckermannDriveControllerPlugin::ReadString(
  sdf::ElementPtr sdf, const std::string & name, const std::string & fallback) const
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}

void AckermannDriveControllerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  const auto left_steering_name = ReadString(sdf, "left_steering_joint", "left_steering_joint");
  const auto right_steering_name = ReadString(sdf, "right_steering_joint", "right_steering_joint");
  const auto rear_left_name = ReadString(sdf, "rear_left_wheel_joint", "rear_left_joint");
  const auto rear_right_name = ReadString(sdf, "rear_right_wheel_joint", "rear_right_joint");

  const auto cmd_vel_topic = ReadString(sdf, "cmd_vel_topic", "cmd_vel");
  const auto odom_topic = ReadString(sdf, "odom_topic", "odom");
  const auto ground_truth_odom_topic = ReadString(
    sdf, "ground_truth_odom_topic", "ground_truth/odom");
  odom_frame_ = ReadString(sdf, "odom_frame", "odom");
  base_frame_ = ReadString(sdf, "base_frame", "base_footprint");

  wheelbase_ = ReadDouble(sdf, "wheelbase", wheelbase_);
  track_width_ = ReadDouble(sdf, "track_width", track_width_);
  wheel_radius_ = ReadDouble(sdf, "wheel_radius", wheel_radius_);
  max_steering_angle_ = ReadDouble(sdf, "max_steering_angle", max_steering_angle_);
  cmd_timeout_ = ReadDouble(sdf, "cmd_timeout", cmd_timeout_);
  max_wheel_torque_ = ReadDouble(sdf, "max_wheel_torque", max_wheel_torque_);

  const double update_rate = ReadDouble(sdf, "update_rate", 50.0);
  update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.02;

  left_steering_joint_ = model_->GetJoint(left_steering_name);
  right_steering_joint_ = model_->GetJoint(right_steering_name);
  rear_left_wheel_joint_ = model_->GetJoint(rear_left_name);
  rear_right_wheel_joint_ = model_->GetJoint(rear_right_name);

  if (!left_steering_joint_ || !right_steering_joint_ ||
    !rear_left_wheel_joint_ || !rear_right_wheel_joint_)
  {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "AckermannDriveController missing joints: %s=%d %s=%d %s=%d %s=%d",
      left_steering_name.c_str(), static_cast<int>(left_steering_joint_ != nullptr),
      right_steering_name.c_str(), static_cast<int>(right_steering_joint_ != nullptr),
      rear_left_name.c_str(), static_cast<int>(rear_left_wheel_joint_ != nullptr),
      rear_right_name.c_str(), static_cast<int>(rear_right_wheel_joint_ != nullptr));
    return;
  }

  const double steering_p = ReadDouble(sdf, "steering_p", 50.0);
  const double steering_i = ReadDouble(sdf, "steering_i", 0.0);
  const double steering_d = ReadDouble(sdf, "steering_d", 5.0);

  auto joint_controller = model_->GetJointController();
  joint_controller->AddJoint(left_steering_joint_);
  joint_controller->AddJoint(right_steering_joint_);
  left_steering_joint_->SetParam("fmax", 0, 100.0);
  right_steering_joint_->SetParam("fmax", 0, 100.0);
  joint_controller->SetPositionPID(
    left_steering_joint_->GetScopedName(), common::PID(steering_p, steering_i, steering_d));
  joint_controller->SetPositionPID(
    right_steering_joint_->GetScopedName(), common::PID(steering_p, steering_i, steering_d));

  rear_left_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  rear_right_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);

  cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&AckermannDriveControllerPlugin::OnCmdVel, this, std::placeholders::_1));
  odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  ground_truth_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    ground_truth_odom_topic, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);

  last_cmd_time_ = model_->GetWorld()->SimTime();
  last_update_time_ = last_cmd_time_;
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AckermannDriveControllerPlugin::OnUpdate, this));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "AckermannDriveController ready: cmd=%s odom=%s ground_truth=%s "
    "wb=%.3f track=%.3f radius=%.3f",
    cmd_vel_topic.c_str(), odom_topic.c_str(), ground_truth_odom_topic.c_str(),
    wheelbase_, track_width_, wheel_radius_);
}

void AckermannDriveControllerPlugin::OnCmdVel(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_linear_ = msg->linear.x;
  target_angular_ = msg->angular.z;
  last_cmd_time_ = model_->GetWorld()->SimTime();
}

AckermannDriveControllerPlugin::Targets AckermannDriveControllerPlugin::ComputeTargets(
  double linear_x, double angular_z) const
{
  Targets targets;

  if (std::abs(linear_x) < 0.01) {
    const double steer = std::clamp(
      angular_z * 0.5, -max_steering_angle_, max_steering_angle_);
    targets.left_steer = steer;
    targets.right_steer = steer;
    return targets;
  }

  double steer = std::atan(wheelbase_ * angular_z / linear_x);
  steer = std::clamp(steer, -max_steering_angle_, max_steering_angle_);

  if (std::abs(steer) < 1e-6) {
    targets.rear_left_velocity = linear_x / wheel_radius_;
    targets.rear_right_velocity = linear_x / wheel_radius_;
    return targets;
  }

  const double sign = steer > 0.0 ? 1.0 : -1.0;
  const double radius = wheelbase_ / std::tan(std::abs(steer));
  const double inner_angle = std::atan(wheelbase_ / (radius - track_width_ / 2.0));
  const double outer_angle = std::atan(wheelbase_ / (radius + track_width_ / 2.0));

  double inner_rear_linear = std::abs(linear_x) * (radius - track_width_ / 2.0) / radius;
  double outer_rear_linear = std::abs(linear_x) * (radius + track_width_ / 2.0) / radius;

  if (linear_x < 0.0) {
    inner_rear_linear = -inner_rear_linear;
    outer_rear_linear = -outer_rear_linear;
  }

  if (sign > 0.0) {
    targets.left_steer = inner_angle;
    targets.right_steer = outer_angle;
    targets.rear_left_velocity = inner_rear_linear / wheel_radius_;
    targets.rear_right_velocity = outer_rear_linear / wheel_radius_;
  } else {
    targets.left_steer = -outer_angle;
    targets.right_steer = -inner_angle;
    targets.rear_left_velocity = outer_rear_linear / wheel_radius_;
    targets.rear_right_velocity = inner_rear_linear / wheel_radius_;
  }

  return targets;
}

void AckermannDriveControllerPlugin::ApplyTargets(const Targets & targets)
{
  auto joint_controller = model_->GetJointController();
  left_steering_joint_->SetPosition(0, targets.left_steer);
  right_steering_joint_->SetPosition(0, targets.right_steer);
  joint_controller->SetPositionTarget(left_steering_joint_->GetScopedName(), targets.left_steer);
  joint_controller->SetPositionTarget(right_steering_joint_->GetScopedName(), targets.right_steer);
  rear_left_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  rear_right_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  rear_left_wheel_joint_->SetParam("vel", 0, targets.rear_left_velocity);
  rear_right_wheel_joint_->SetParam("vel", 0, targets.rear_right_velocity);
  joint_controller->Update();
}

void AckermannDriveControllerPlugin::PublishOdometry(const common::Time & sim_time)
{
  const auto pose = model_->WorldPose();
  const auto linear = model_->WorldLinearVel();
  const auto angular = model_->WorldAngularVel();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(sim_time.sec);
  stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x = linear.X();
  odom.twist.twist.linear.y = linear.Y();
  odom.twist.twist.linear.z = linear.Z();
  odom.twist.twist.angular.x = angular.X();
  odom.twist.twist.angular.y = angular.Y();
  odom.twist.twist.angular.z = angular.Z();
  odom_pub_->publish(odom);
  ground_truth_odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = pose.Pos().X();
  tf.transform.translation.y = pose.Pos().Y();
  tf.transform.translation.z = pose.Pos().Z();
  tf.transform.rotation.x = pose.Rot().X();
  tf.transform.rotation.y = pose.Rot().Y();
  tf.transform.rotation.z = pose.Rot().Z();
  tf.transform.rotation.w = pose.Rot().W();
  tf_broadcaster_->sendTransform(tf);
}

void AckermannDriveControllerPlugin::OnUpdate()
{
  const common::Time now = model_->GetWorld()->SimTime();
  if ((now - last_update_time_).Double() < update_period_) {
    return;
  }
  last_update_time_ = now;

  double linear = target_linear_;
  double angular = target_angular_;
  if ((now - last_cmd_time_).Double() > cmd_timeout_) {
    linear = 0.0;
    angular = 0.0;
  }

  ApplyTargets(ComputeTargets(linear, angular));
  PublishOdometry(now);
}

GZ_REGISTER_MODEL_PLUGIN(AckermannDriveControllerPlugin)

}  // namespace gazebo
