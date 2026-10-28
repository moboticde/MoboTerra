#include "kinematics_conversions/twist_to_acker.h"

TwistToAckermann::TwistToAckermann() : Node("twist_to_acker_converter")
{
  // Declare the 'wheelbase' parameter with a default value, making it configurable from launch files.
  this->declare_parameter<double>("wheelbase", 0.35);
  this->get_parameter("wheelbase", wheelbase_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Wheelbase set to: " << wheelbase_);

  acker_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("agv_acker", 10);
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "agv_vel", 10, std::bind(&TwistToAckermann::twistCallback, this, std::placeholders::_1));
}

void TwistToAckermann::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const double v = std::abs(msg->twist.linear.x) < TOLERANCE_LINEAR_VELOCITY ? 0.0 : msg->twist.linear.x;
  const double w = std::abs(msg->twist.angular.z) < TOLERANCE_ANGULAR_VELOCITY ? 0.0 : msg->twist.angular.z;
  const double wL = w * wheelbase_;

  ackermann_msgs::msg::AckermannDriveStamped acker;
  acker.header = msg->header;
  acker.drive.speed = std::copysign(std::sqrt(v * v + wL * wL), v);
  acker.drive.steering_angle = std::abs(acker.drive.speed) > 1e-3 ? std::atan2(wL, v) : last_steering_angle_;
  acker_pub_->publish(acker);

  last_steering_angle_ = acker.drive.steering_angle;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToAckermann>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}