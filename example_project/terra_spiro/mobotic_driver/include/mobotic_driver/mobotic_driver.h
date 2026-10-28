#ifndef MOBOTIC_DRIVER_H
#define MOBOTIC_DRIVER_H
#include <mobotic_driver/communication.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

class MoboticDriver : public rclcpp::Node
{
public:
  explicit MoboticDriver(const rclcpp::NodeOptions& options);
  ~MoboticDriver() override;
  void canFrameCallback(const can_msgs::msg::Frame& frame);
  void feedbackTimerCallback();
  void jointSetpointsCallback(const sensor_msgs::msg::JointState& msg);
  void watchdogTimerCallback();
  void sendStopToAllDrives();
  void disableAllDrives();
  void enableAllDrives();
  void initializeAllDrives();
  void tractionPdoTimerCallback();
  void publishTractionPdoCycle();
  static can_msgs::msg::Frame stampFrame(const can_msgs::msg::Frame& frame, const std::string& frame_id, const rclcpp::Time& stamp);

private:
  std::shared_ptr<rclcpp::TimerBase> feedback_timer_, watchdog_timer_setpoints_, timer_can_heartbeat_, traction_pdo_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  // CAN infrastructure
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  std::map<std::string, MoboticDriveCanCommunication> drives_;
  std::map<std::string, rclcpp::Time> last_enable_attempt_, last_initialize_attempt_;
  std::map<std::string, rclcpp::Time> last_error_poll_, last_status_poll_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_setpoints_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_all_drives_, srv_disable_all_drives_, srv_clear_errors_, srv_initialize_all_drives_;
  double watchdog_timeout_{0};
  double diagnostic_sdo_period_{1.0};
  int traction_max_profile_velocity_{185429};
  int traction_profile_accel_{92714}, traction_profile_decel_{92714};
  int telemetry_pdo_sync_divider_{1};
  bool traction_pdo_enabled_{true};
  bool telemetry_pdo_enabled_{true};
  bool feedbacks_requested_{false}, enable_all_drives_requested_{false}, initialize_all_drives_requested_{false};
};
#endif // MOBOTIC_DRIVER_H
