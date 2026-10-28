#ifndef JOINT_STATE_TO_TWIST_CONTROLLER_JOINTSTATETOTWIST_H
#define JOINT_STATE_TO_TWIST_CONTROLLER_JOINTSTATETOTWIST_H

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

class JointStateToTwist : public rclcpp::Node
{
public:
  explicit JointStateToTwist();
  void jointStateCallback(const sensor_msgs::msg::JointState& msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  std::vector<std::string> traction_joint_names_, steering_joint_names_;
  std::vector<double> wheels_mounting_angles_;
  double wheel_radius_;
  Eigen::MatrixXd H_;
};
#endif // JOINT_STATE_TO_TWIST_CONTROLLER_JOINTSTATETOTWIST_H