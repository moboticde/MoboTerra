#ifndef VANGUARD_DRIVER_H
#define VANGUARD_DRIVER_H
#include "communication.h"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>

class VanguardBatteryDriver final : public rclcpp::Node
{
public:
  explicit VanguardBatteryDriver(const rclcpp::NodeOptions& options);
  ~VanguardBatteryDriver() override;
  void canFrameCallback(const can_msgs::msg::Frame& frame);
  static can_msgs::msg::Frame stampFrame(const can_msgs::msg::Frame& frame, const std::string& frame_id,
                                         const rclcpp::Time& stamp);

private:
  std::shared_ptr<rclcpp::TimerBase> timer_high_voltage_, timer_mobotic_driver_;
  std::vector<VanguardBatteryCanCommunication> batteries_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_high_voltage_, srv_disable_high_voltage_;
  int high_voltage_msg_counter_{0}, hv_command_{3}, op_consent_{3};
  bool high_voltage_requested_{false}, mobotic_driver_watchdog_timeout_{false};
  uint8_t source_address_{0};
  unsigned mobotic_driver_src_addr_{0};
  rclcpp::Time high_voltage_start_time_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr> pubs_battery_state_;
};
#endif // VANGUARD_DRIVER_H
