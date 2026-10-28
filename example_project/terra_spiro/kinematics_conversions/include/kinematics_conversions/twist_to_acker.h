#ifndef TWIST_TO_ACKERMANN_H
#define TWIST_TO_ACKERMANN_H

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

class TwistToAckermann : public rclcpp::Node
{
public:
  TwistToAckermann();

private:
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr acker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  double wheelbase_, last_steering_angle_;

  // C++11 constexpr is fine, no change needed here.
  static constexpr double TOLERANCE_LINEAR_VELOCITY{0.001}, TOLERANCE_ANGULAR_VELOCITY{0.0174532925}; // lin: 1 mm/s, ang: 1 deg/s
};

#endif // TWIST_TO_ACKERMANN_H