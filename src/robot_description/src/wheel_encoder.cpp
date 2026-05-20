#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  class WheelEncoderPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();
      this->ros_node = gazebo_ros::Node::Get(_sdf);

      std::string joint_name = "rear_left_joint";
      if (_sdf->HasElement("joint"))
        joint_name = _sdf->Get<std::string>("joint");

      this->joint = _model->GetJoint(joint_name);
      if (!this->joint)
      {
        RCLCPP_ERROR(this->ros_node->get_logger(),
                     "Wheel encoder joint not found: %s", joint_name.c_str());
        return;
      }

      if (_sdf->HasElement("right_joint"))
      {
        std::string right_joint_name = _sdf->Get<std::string>("right_joint");
        this->right_joint = _model->GetJoint(right_joint_name);
        if (!this->right_joint)
        {
          RCLCPP_ERROR(this->ros_node->get_logger(),
                       "Wheel encoder right_joint not found: %s",
                       right_joint_name.c_str());
          return;
        }
      }

      if (_sdf->HasElement("update_rate"))
        this->update_rate = _sdf->Get<double>("update_rate");
      if (_sdf->HasElement("wheel_radius"))
        this->wheel_radius = _sdf->Get<double>("wheel_radius");
      if (_sdf->HasElement("output_type"))
        this->output_type = _sdf->Get<std::string>("output_type");

      std::string topic_name = "/robot/wheel_encoder/" + joint_name;
      if (_sdf->HasElement("topic"))
        topic_name = _sdf->Get<std::string>("topic");

      this->publisher =
        this->ros_node->create_publisher<std_msgs::msg::Float64>(topic_name, 10);

      if (this->update_rate <= 0.0)
      {
        RCLCPP_ERROR(this->ros_node->get_logger(),
                     "Wheel encoder update_rate must be positive");
        return;
      }

      this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WheelEncoderPlugin::OnUpdate, this));

      RCLCPP_INFO(this->ros_node->get_logger(),
                  "Publishing wheel encoder %s from %s",
                  topic_name.c_str(), joint_name.c_str());
    }

    void OnUpdate()
    {
      common::Time cur_time = this->world->SimTime();

      double dt = (cur_time - this->last_update_time).Double();
      if (dt < 1.0 / this->update_rate)
        return;

      double value = this->joint->GetVelocity(0);
      if (this->right_joint)
        value = 0.5 * (value + this->right_joint->GetVelocity(0));

      if (this->output_type == "linear_velocity")
        value *= this->wheel_radius;

      std_msgs::msg::Float64 msg;
      msg.data = value;
      this->publisher->publish(msg);

      this->last_update_time = cur_time;
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    physics::JointPtr right_joint;
    physics::WorldPtr world;
    event::ConnectionPtr update_connection;
    gazebo_ros::Node::SharedPtr ros_node;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

    common::Time last_update_time;
    double update_rate = 50.0;
    double wheel_radius = 0.07;
    std::string output_type = "angular_velocity";
  };

  GZ_REGISTER_MODEL_PLUGIN(WheelEncoderPlugin)
}  // namespace gazebo
