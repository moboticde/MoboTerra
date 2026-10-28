#include "kinematics_conversions/joint_state_to_twist.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <vector>

JointStateToTwist::JointStateToTwist() : Node("js_to_twist_converter")
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("traction_joint_names");
  this->declare_parameter<std::vector<std::string>>("steering_joint_names");
  this->declare_parameter<double>("wheel_radius");
  this->declare_parameter<std::vector<double>>("wheels_x");
  this->declare_parameter<std::vector<double>>("wheels_y");
  this->declare_parameter<std::vector<double>>("wheel_mount_angles");

  bool params_ok{true};
  // Get parameters
  params_ok &= this->get_parameter("traction_joint_names", traction_joint_names_);
  params_ok &= this->get_parameter("steering_joint_names", steering_joint_names_);
  params_ok &= this->get_parameter("wheel_radius", wheel_radius_);
  params_ok &= this->get_parameter("wheel_mount_angles", wheels_mounting_angles_);

  std::vector<double> wheels_x, wheels_y;
  params_ok &= this->get_parameter("wheels_x", wheels_x);
  params_ok &= this->get_parameter("wheels_y", wheels_y);

  if (!params_ok || traction_joint_names_.empty() || steering_joint_names_.empty() || wheel_radius_ <= 0.0 ||
      wheels_x.empty() || wheels_y.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "[kinematics_conversions] Parameters not found on the ROS parameter server.");
    rclcpp::shutdown();
    return;
  }

  if (wheels_x.size() != wheels_y.size())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[kinematics_conversions] X and Y coordinates of the wheels do not match by size!");
    rclcpp::shutdown();
    return;
  }

  H_ = Eigen::MatrixXd::Zero(wheels_x.size() * 2, 3);
  for (size_t i = 0; i < wheels_x.size(); i++)
  {
    H_.row(2 * i) << 1, 0, -wheels_y[i];
    H_.row(2 * i + 1) << 0, 1, wheels_x[i];
  }

  sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&JointStateToTwist::jointStateCallback, this, std::placeholders::_1));
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("agv_vel", 1);
}

void JointStateToTwist::jointStateCallback(const sensor_msgs::msg::JointState& msg)
{
  std::vector<double> wheel_velocities, steering_angles;

  for (const auto& steering_joint_name : steering_joint_names_)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), steering_joint_name);
    if (it == msg.name.end())
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[kinematics_conversions] Joint "
                                                 << steering_joint_name << " not found in joint_states message!");
      return;
    }
    steering_angles.push_back(msg.position[std::distance(msg.name.begin(), it)]);
  }

  for (const auto& traction_joint_name : traction_joint_names_)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), traction_joint_name);
    if (it == msg.name.end())
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[kinematics_conversions] Joint "
                                                 << traction_joint_name << " not found in joint_states message!");
      return;
    }
    wheel_velocities.push_back(msg.velocity[std::distance(msg.name.begin(), it)]);
  }

  if (wheel_velocities.size() != steering_angles.size())
  {
    RCLCPP_WARN(this->get_logger(),
                "[kinematics_conversions] The number of wheel velocities and wheel steering angles do not match!");
    return;
  }

  Eigen::MatrixXd v(wheel_velocities.size() * 2, 1), V(3, 1);
  for (size_t i = 0; i < wheel_velocities.size(); i++)
  {
    v.row(2 * i) << wheel_velocities[i] * std::cos(steering_angles[i] + wheels_mounting_angles_[i]);
    v.row(2 * i + 1) << wheel_velocities[i] * std::sin(steering_angles[i] + wheels_mounting_angles_[i]);
  }
  //std::cout << "wheel velocities (pre-multiplication): " << v << std::endl;
  v *= wheel_radius_;

  V = H_.completeOrthogonalDecomposition().pseudoInverse() * v;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg.header;
  twist.twist.linear.x = V(0);
  twist.twist.linear.y = V(1);
  twist.twist.angular.z = V(2);
  pub_twist_->publish(twist);
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateToTwist>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
