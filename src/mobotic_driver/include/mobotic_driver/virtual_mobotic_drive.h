#ifndef VIRTUAL_MOBOTIC_DRIVE_H
#define VIRTUAL_MOBOTIC_DRIVE_H
#include "virtual_drive_protocol.hpp"

#include <can_msgs/msg/frame.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class VirtualMoboticDriveCanCommunication final : public rclcpp::Node
{
public:
  explicit VirtualMoboticDriveCanCommunication(const rclcpp::NodeOptions& options);
  void canFrameCallback(const can_msgs::msg::Frame& req_frame);

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  void advanceMotion();
  std::unique_ptr<VirtualDriveProtocol> protocol_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
};
#endif // VIRTUAL_MOBOTIC_DRIVE_H
