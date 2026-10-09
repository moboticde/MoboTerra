#ifndef MOBOTIC_DRIVER_H
#define MOBOTIC_DRIVER_H
#include <mobotic_driver/communication.hpp>
#include <mobotic_driver/management_lease.hpp>
#include <mobotic_driver/motion_source_gate.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mobotic_interfaces/msg/wheel_module_command.hpp>
#include <mobotic_interfaces/msg/wheel_module_status_array.hpp>
#include <mobotic_interfaces/msg/vehicle_mode_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

class MoboticDriver : public rclcpp::Node
{
public:
  explicit MoboticDriver(const rclcpp::NodeOptions& options);
  ~MoboticDriver() override;
  void canFrameCallback(const can_msgs::msg::Frame& frame);
  void feedbackTimerCallback();
  void jointSetpointsCallback(const sensor_msgs::msg::JointState& msg, MotionSourceGate::Source source);
  void wheelModuleCommandCallback(const mobotic_interfaces::msg::WheelModuleCommand& msg);
  void watchdogTimerCallback();
  void sendStopToAllDrives();
  void disableAllDrives();
  void enableAllDrives();
  void initializeAllDrives();
  void tractionPdoTimerCallback();
  void publishTractionPdoCycle();
  static can_msgs::msg::Frame stampFrame(const can_msgs::msg::Frame& frame, const std::string& frame_id, const rclcpp::Time& stamp);

private:
  struct WheelModuleBinding
  {
    std::string steering_drive;
    std::string traction_drive;
  };

  void applyJointSetpoints(const sensor_msgs::msg::JointState& msg);
  void clearErrorsOnAllDrives();
  void configureWheelModules(const std::vector<std::string>& module_names,
                             const std::vector<std::string>& steering_drives,
                             const std::vector<std::string>& traction_drives);
  void publishWheelModuleStatus(const rclcpp::Time& stamp);
  void enforceManagementLease();
  void modeStateCallback(const mobotic_interfaces::msg::VehicleModeState& msg);
  void enforceModeAuthority();
  bool driveFeedbackFresh(const MoboticDriveCanCommunication& drive, FeedbackFreshness::Clock::time_point now) const;
  std::string feedbackProblem(const MoboticDriveCanCommunication& drive, FeedbackFreshness::Clock::time_point now) const;
  bool allDriveFeedbackFresh(FeedbackFreshness::Clock::time_point now) const;
  void enforceFeedbackFreshness();
  void publishDiagnostics();

  std::shared_ptr<rclcpp::TimerBase> feedback_timer_, watchdog_timer_setpoints_, timer_can_heartbeat_,
      traction_pdo_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<mobotic_interfaces::msg::WheelModuleStatusArray>::SharedPtr pub_wheel_module_status_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::TimerBase::SharedPtr management_watchdog_timer_, diagnostics_timer_;
  ManagementLease management_lease_;
  MotionSourceGate motion_source_gate_;
  bool mode_stop_sent_{false};
  MotionSourceGate::Clock::time_point last_mode_stop_{};
  double supervisor_timeout_{0.3};
  int64_t last_management_stamp_ns_{0};
  int64_t motion_accept_after_ns_{0};
  bool motion_command_received_{false};
  ManagementLease::Clock::time_point last_motion_command_received_{}, last_can_error_{};
  uint64_t can_error_count_{0};
  bool management_disable_sent_{false};
  ManagementLease::Clock::time_point last_management_disable_{};
  std::map<std::string, OrderedFeedbackSample> last_can_feedback_;
  // Original ROS acquisition/reception stamps for velocity and steering position.
  std::map<std::string, std::array<int64_t, 2>> motion_sample_stamp_ns_;
  std::map<std::string, RawRegisterTelemetry> sto_status_, digital_inputs_;
  std::map<std::string, rclcpp::Time> last_safety_poll_;
  // CAN infrastructure
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  std::map<std::string, MoboticDriveCanCommunication> drives_;
  std::map<std::string, WheelModuleBinding> wheel_modules_;
  std::map<std::string, double> last_motor_current_ma_;
  std::map<std::string, rclcpp::Time> last_enable_attempt_, last_initialize_attempt_;
  std::map<std::string, rclcpp::Time> last_error_poll_, last_status_poll_;
  rclcpp::Subscription<mobotic_interfaces::msg::WheelModuleCommand>::SharedPtr sub_wheel_module_command_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_setpoints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_supervisor_joint_setpoints_;
  rclcpp::Subscription<mobotic_interfaces::msg::VehicleModeState>::SharedPtr sub_mode_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_all_drives_, srv_disable_all_drives_, srv_clear_errors_, srv_initialize_all_drives_;
  double watchdog_timeout_{0};
  double diagnostic_sdo_period_{1.0};
  double motion_feedback_timeout_{0.3}, status_feedback_timeout_{0.3}, fault_feedback_timeout_{2.5};
  bool feedback_motion_blocked_{true};
  bool feedback_stop_sent_{false};
  FeedbackFreshness::Clock::time_point last_feedback_stop_{};
  int traction_max_profile_velocity_{0}; // Populated from declared parameters in the constructor.
  int traction_profile_accel_{0}, traction_profile_decel_{0};
  int telemetry_pdo_sync_divider_{1};
  bool traction_pdo_enabled_{true};
  bool telemetry_pdo_enabled_{true};
  bool feedbacks_requested_{false}, enable_all_drives_requested_{false}, initialize_all_drives_requested_{false};
  bool module_management_received_{false};
};
#endif // MOBOTIC_DRIVER_H
