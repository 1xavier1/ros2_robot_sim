/**
 * Wheel Encoder Gazebo Plugin
 * Publishes wheel encoder data for robot odometry and localization
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class WheelEncoderPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();

      // Get joint name
      std::string joint_name = "rear_left_wheel_joint";
      if (_sdf->HasElement("joint"))
        joint_name = _sdf->Get<std::string>("joint");

      this->joint = _model->GetJoint(joint_name);
      if (!this->joint)
      {
        ROS_ERROR("Wheel encoder: joint not found: %s", joint_name.c_str());
        return;
      }

      // Get parameters
      if (_sdf->HasElement("update_rate"))
        this->update_rate = _sdf->Get<double>("update_rate");
      if (_sdf->HasElement("encoder_resolution"))
        this->encoder_resolution = _sdf->Get<int>("encoder_resolution");

      // ROS node
      if (!ros::isInitialized())
      {
        ROS_FATAL("ROS not initialized");
        return;
      }

      std::string topic_name = "/robot/wheel_encoder/" + joint_name;
      this->pub = ros::nodeHandle().advertise<std_msgs::Float64>(topic_name, 10);

      // Create transport node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      // Listen to physics update
      this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WheelEncoderPlugin::OnUpdate, this));

      // Initialize encoder
      this->prev_angle = this->joint->Position(0);
      this->cumulative_angle = 0.0;
    }

    void OnUpdate()
    {
      common::Time cur_time = this->world->SimTime();

      double dt = (cur_time - this->last_update_time).Double();
      if (dt < 1.0 / this->update_rate)
        return;

      // Get current joint position
      double current_angle = this->joint->Position(0);

      // Calculate angle difference (handle wrap-around)
      double delta = current_angle - this->prev_angle;
      if (delta > M_PI)
        delta -= 2 * M_PI;
      if (delta < -M_PI)
        delta += 2 * M_PI;

      this->cumulative_angle += delta;
      this->prev_angle = current_angle;

      // Convert to encoder ticks
      double encoder_value = fmod(this->cumulative_angle * this->encoder_resolution / (2 * M_PI), this->encoder_resolution);
      if (encoder_value < 0)
        encoder_value += this->encoder_resolution;

      // Publish
      std_msgs::Float64 msg;
      msg.data = encoder_value;
      this->pub.publish(msg);

      this->last_update_time = cur_time;
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    physics::WorldPtr world;
    transport::NodePtr node;
    event::ConnectionPtr update_connection;

    ros::Publisher pub;
    common::Time last_update_time;
    double prev_angle;
    double cumulative_angle;
    double update_rate = 50.0;
    int encoder_resolution = 360;
  };

  GZ_REGISTER_MODEL_PLUGIN(WheelEncoderPlugin)
}  // namespace gazebo
