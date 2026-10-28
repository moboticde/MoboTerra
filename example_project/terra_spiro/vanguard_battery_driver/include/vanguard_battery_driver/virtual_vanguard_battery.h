#ifndef VIRTUAL_MOBOTIC_DRIVE_H
#define VIRTUAL_MOBOTIC_DRIVE_H
#include "communication.h"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

class VirtualVanguardBattery final : public rclcpp::Node
{
public:
  explicit VirtualVanguardBattery(const rclcpp::NodeOptions& options);
  void canFrameCallback(const can_msgs::msg::Frame& req_frame);

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  OperationalStatus operational_status_;
  HighVoltageStatus high_voltage_status_;
  std::string battery_id_;
  uint8_t source_address_, controller_address_;
  uint32_t status_pgn_;
  bool seen_op_consent_3{false}, seen_op_consent_1{false}, seen_high_voltage_1{false};
  rclcpp::Time time_first_op_consent_3, time_first_op_consent_1, time_first_high_voltage_1;
  rclcpp::TimerBase::SharedPtr timer_feedback_;
};
#endif // VIRTUAL_MOBOTIC_DRIVE_H
