#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <can_msgs/msg/frame.hpp>
#include <mobotic_driver/communication.hpp>
#include <mobotic_driver/mobotic_driver.h>

namespace
{
constexpr int DEFAULT_TRACTION_PROFILE_ACCEL = 32093; // 1.0 m/s^2 at r=0.325 m, 16:1, 4096 inc/rev
constexpr int DEFAULT_TRACTION_PROFILE_DECEL = DEFAULT_TRACTION_PROFILE_ACCEL;
constexpr int DEFAULT_TRACTION_MAX_PROFILE_VELOCITY = 116053; // Four-wheel reference ceiling, motor inc/s

unsigned parseCanNodeId(const std::string& text)
{
  auto parse_with_base = [](const std::string& value, int base) -> unsigned {
    std::size_t pos = 0;
    const auto parsed = std::stoul(value, &pos, base);
    if (pos != value.size())
    {
      throw std::invalid_argument("trailing characters in CAN node id");
    }
    return static_cast<unsigned>(parsed);
  };

  try
  {
    return parse_with_base(text, 0);  // accepts 4, 0x04
  }
  catch (const std::exception&)
  {
    return parse_with_base(text, 10); // fallback for strings like 08
  }
}

DeviceMode effectiveMode(const MoboticDriveCanCommunication& drive)
{
  return drive.currentMode() == UNKNOWN ? drive.targetMode() : drive.currentMode();
}

bool hasIndex(const std::vector<double>& values, const std::size_t index)
{
  return index < values.size();
}

std::string driveErrorText(const MoboticDriveCanCommunication& drive)
{
  std::ostringstream text;
  text << "0x" << std::hex << drive.getError();
  if (drive.isTractionController() && drive.getErrorCode() != 0)
  {
    text << " (DS402 error code 0x" << drive.getErrorCode() << ")";
  }
  return text.str();
}

bool endsWith(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

uint8_t toStatusControlMode(const DeviceMode mode)
{
  using Status = mobotic_interfaces::msg::WheelModuleStatus;
  switch (mode)
  {
  case POSITION:
    return Status::CONTROL_MODE_POSITION;
  case VELOCITY:
  case S_VELOCITY:
    return Status::CONTROL_MODE_VELOCITY;
  case CURRENT:
    return Status::CONTROL_MODE_CURRENT;
  default:
    return Status::CONTROL_MODE_UNKNOWN;
  }
}
} // namespace

MoboticDriver::MoboticDriver(const rclcpp::NodeOptions& options) : Node("mobotic_driver", options)
{
  this->declare_parameter("can_node_id", "");
  this->declare_parameter("feedback_request_period", 0.02);
  this->declare_parameter("can_heartbeat_period", 0.01);
  this->declare_parameter("watchdog_timeout", 0.1);
  this->declare_parameter("diagnostic_sdo_period", 1.0);
  this->declare_parameter("supervisor_timeout", 0.3);
  this->declare_parameter("motion_feedback_timeout", 0.3);
  this->declare_parameter("status_feedback_timeout", 0.3);
  this->declare_parameter("fault_feedback_timeout", 2.5);
  this->declare_parameter("traction_pdo_enabled", true);
  this->declare_parameter("telemetry_pdo_enabled", true);
  this->declare_parameter("telemetry_pdo_sync_divider", 1);
  this->declare_parameter("traction_pdo_period", 0.02);
  this->declare_parameter("traction_max_profile_velocity", DEFAULT_TRACTION_MAX_PROFILE_VELOCITY);
  this->declare_parameter("traction_profile_accel", DEFAULT_TRACTION_PROFILE_ACCEL);
  this->declare_parameter("traction_profile_decel", DEFAULT_TRACTION_PROFILE_DECEL);
  this->declare_parameter("drives.names", std::vector<std::string>());
  this->declare_parameter("drives.modes", std::vector<std::string>());
  this->declare_parameter("drives.can_node_ids", std::vector<std::string>());
  this->declare_parameter("drives.gear_ratios", std::vector<double>());
  this->declare_parameter("drives.resolutions", std::vector<int64_t>());
  this->declare_parameter("drives.min_steering_velocities", std::vector<int64_t>());
  this->declare_parameter("drives.max_steering_velocities", std::vector<int64_t>());
  this->declare_parameter("drives.min_traction_velocities", std::vector<int64_t>());
  this->declare_parameter("drives.max_traction_velocities", std::vector<int64_t>());
  this->declare_parameter("drives.min_currents", std::vector<int64_t>());
  this->declare_parameter("drives.max_currents", std::vector<int64_t>());
  this->declare_parameter("modules.names", std::vector<std::string>());
  this->declare_parameter("modules.steering_drives", std::vector<std::string>());
  this->declare_parameter("modules.traction_drives", std::vector<std::string>());

  std::string driver_can_node_id;
  double feedback_request_period = 0.1;
  double can_heartbeat_period = 0.01;
  double traction_pdo_period = 0.02;
  std::vector<std::string> names;
  std::vector<std::string> modes;
  std::vector<std::string> can_node_ids;
  std::vector<double> gear_ratios;
  std::vector<int64_t> resolutions;
  std::vector<int64_t> min_currents;
  std::vector<int64_t> max_currents;
  std::vector<int64_t> min_traction_velocities;
  std::vector<int64_t> max_traction_velocities;
  std::vector<int64_t> min_steering_velocities;
  std::vector<int64_t> max_steering_velocities;
  std::vector<std::string> module_names;
  std::vector<std::string> module_steering_drives;
  std::vector<std::string> module_traction_drives;

  bool params{true};
  params &= this->get_parameter("can_node_id", driver_can_node_id);
  params &= this->get_parameter("feedback_request_period", feedback_request_period);
  params &= this->get_parameter("can_heartbeat_period", can_heartbeat_period);
  params &= this->get_parameter("watchdog_timeout", watchdog_timeout_);
  params &= this->get_parameter("diagnostic_sdo_period", diagnostic_sdo_period_);
  params &= this->get_parameter("supervisor_timeout", supervisor_timeout_);
  params &= this->get_parameter("motion_feedback_timeout", motion_feedback_timeout_);
  params &= this->get_parameter("status_feedback_timeout", status_feedback_timeout_);
  params &= this->get_parameter("fault_feedback_timeout", fault_feedback_timeout_);
  params &= this->get_parameter("traction_pdo_enabled", traction_pdo_enabled_);
  params &= this->get_parameter("telemetry_pdo_enabled", telemetry_pdo_enabled_);
  params &= this->get_parameter("telemetry_pdo_sync_divider", telemetry_pdo_sync_divider_);
  params &= this->get_parameter("traction_pdo_period", traction_pdo_period);
  params &= this->get_parameter("traction_max_profile_velocity", traction_max_profile_velocity_);
  params &= this->get_parameter("traction_profile_accel", traction_profile_accel_);
  params &= this->get_parameter("traction_profile_decel", traction_profile_decel_);
  params &= this->get_parameter("drives.names", names);
  params &= this->get_parameter("drives.modes", modes);
  params &= this->get_parameter("drives.can_node_ids", can_node_ids);
  params &= this->get_parameter("drives.gear_ratios", gear_ratios);
  params &= this->get_parameter("drives.resolutions", resolutions);
  params &= this->get_parameter("drives.min_steering_velocities", min_steering_velocities);
  params &= this->get_parameter("drives.max_steering_velocities", max_steering_velocities);
  params &= this->get_parameter("drives.min_traction_velocities", min_traction_velocities);
  params &= this->get_parameter("drives.max_traction_velocities", max_traction_velocities);
  params &= this->get_parameter("drives.min_currents", min_currents);
  params &= this->get_parameter("drives.max_currents", max_currents);
  params &= this->get_parameter("modules.names", module_names);
  params &= this->get_parameter("modules.steering_drives", module_steering_drives);
  params &= this->get_parameter("modules.traction_drives", module_traction_drives);

  if (!params)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "[mobotic_driver] Failed to load required parameters. Required: can_node_id, "
                 "feedback_request_period, can_heartbeat_period, watchdog_timeout, drives.names, drives.modes, "
                 "drives.can_node_ids, drives.gear_ratios, drives.resolutions, drives.min_steering_velocities, "
                 "drives.max_steering_velocities, drives.min_traction_velocities, drives.max_traction_velocities, "
                 "drives.min_currents, drives.max_currents.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (traction_max_profile_velocity_ <= 0 || traction_profile_accel_ <= 0 || traction_profile_decel_ <= 0)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] Traction profile velocity, acceleration and deceleration must be positive.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (!std::isfinite(diagnostic_sdo_period_) || diagnostic_sdo_period_ <= 0.0)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] diagnostic_sdo_period must be positive.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (!std::isfinite(watchdog_timeout_) || watchdog_timeout_ <= 0.0 ||
      !std::isfinite(supervisor_timeout_) || supervisor_timeout_ <= 0.0)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] watchdog_timeout must be positive.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (telemetry_pdo_sync_divider_ < 1 || telemetry_pdo_sync_divider_ > 240)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] telemetry_pdo_sync_divider must be in [1, 240].");
    rclcpp::shutdown();
    std::exit(1);
  }
  if (!std::isfinite(motion_feedback_timeout_) || motion_feedback_timeout_ <= 0.0 ||
      !std::isfinite(status_feedback_timeout_) || status_feedback_timeout_ <= 0.0 ||
      !std::isfinite(fault_feedback_timeout_) || fault_feedback_timeout_ <= diagnostic_sdo_period_ ||
      !std::isfinite(feedback_request_period) || feedback_request_period <= 0.0 ||
      feedback_request_period >= status_feedback_timeout_)
  {
    throw std::invalid_argument("Feedback timeouts must be finite/positive; status timeout must exceed feedback period, fault timeout must exceed fault polling period");
  }

  const bool all_sizes_match =
      names.size() == modes.size() && names.size() == can_node_ids.size() && names.size() == gear_ratios.size() &&
      names.size() == resolutions.size() && names.size() == min_currents.size() && names.size() == max_currents.size() &&
      names.size() == min_traction_velocities.size() && names.size() == max_traction_velocities.size() &&
      names.size() == min_steering_velocities.size() && names.size() == max_steering_velocities.size();

  if (!all_sizes_match || names.empty())
  {
    RCLCPP_FATAL(this->get_logger(),
                 "[mobotic_driver] Invalid drive parameter arrays. All drive arrays must have equal non-zero size.");
    rclcpp::shutdown();
    std::exit(1);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Configured drives: " << names.size());

  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const unsigned can_node_id = parseCanNodeId(can_node_ids.at(i));

    std::map<DriveLimits, double> limits;
    limits.emplace(MIN_CURRENT, min_currents.at(i));
    limits.emplace(MAX_CURRENT, max_currents.at(i));
    limits.emplace(MIN_STEERING_VELOCITY, min_steering_velocities.at(i));
    limits.emplace(MAX_STEERING_VELOCITY, max_steering_velocities.at(i));
    limits.emplace(MIN_TRACTION_VELOCITY, min_traction_velocities.at(i));
    limits.emplace(MAX_TRACTION_VELOCITY, max_traction_velocities.at(i));
    limits.emplace(MIN_STEERING_POSITION, -std::numeric_limits<double>::infinity());
    limits.emplace(MAX_STEERING_POSITION, std::numeric_limits<double>::infinity());

    DeviceMode mode = fromString(modes.at(i));
    if (mode == UNKNOWN)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Unknown drive mode '" << modes.at(i) << "' for drive '" << names.at(i)
                                                                << "'. Expected: velocity, position, current, s_velocity.");
      rclcpp::shutdown();
      std::exit(1);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Drive '" << names.at(i) << "': node=" << can_node_ids.at(i)
                                                      << ", mode=" << modes.at(i)
                                                      << ", gear_ratio=" << gear_ratios.at(i)
                                                      << ", resolution=" << resolutions.at(i));
    RCLCPP_INFO_STREAM(this->get_logger(), "  Current limits: " << limits.at(MIN_CURRENT) / 1000.0 << " ... "
                                                                 << limits.at(MAX_CURRENT) / 1000.0 << " A");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Steering velocity limits: "
                                                << rpmToAngVel(limits.at(MIN_STEERING_VELOCITY), gear_ratios.at(i))
                                                << " ... "
                                                << rpmToAngVel(limits.at(MAX_STEERING_VELOCITY), gear_ratios.at(i))
                                                << " rad/s");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Traction velocity limits: "
                                                << ecTractionVelocityToAngVel(limits.at(MIN_TRACTION_VELOCITY),
                                                                             gear_ratios.at(i), resolutions.at(i))
                                                << " ... "
                                                << ecTractionVelocityToAngVel(limits.at(MAX_TRACTION_VELOCITY),
                                                                             gear_ratios.at(i), resolutions.at(i))
                                                << " rad/s");
    if (mode == VELOCITY)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "  Traction max profile velocity: "
                                                << traction_max_profile_velocity_ << " controller increments/s");
      RCLCPP_INFO_STREAM(this->get_logger(), "  Traction profile: accel=" << traction_profile_accel_
                                                                           << ", decel=" << traction_profile_decel_
                                                                           << " controller increments/s^2");
    }

    drives_.emplace(names.at(i), MoboticDriveCanCommunication(names.at(i), can_node_id, mode,
                                                              static_cast<int>(resolutions.at(i)), gear_ratios.at(i),
                                                              limits));
  }

  configureWheelModules(module_names, module_steering_drives, module_traction_drives);

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()),
      [this](const can_msgs::msg::Frame& frame) { this->canFrameCallback(frame); });

  pub_joint_states_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  pub_wheel_module_status_ = this->create_publisher<mobotic_interfaces::msg::WheelModuleStatusArray>(
      "wheel_modules/status", rclcpp::QoS(rclcpp::KeepLast(1)));

  sub_wheel_module_command_ = this->create_subscription<mobotic_interfaces::msg::WheelModuleCommand>(
      "wheel_modules/command", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const mobotic_interfaces::msg::WheelModuleCommand& command) {
        this->wheelModuleCommandCallback(command);
      });

  sub_joint_setpoints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "kinematics/joint_setpoints", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::JointState& command) {
        this->jointSetpointsCallback(command, MotionSourceGate::Source::Kinematics);
      });

  sub_supervisor_joint_setpoints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "supervisor/joint_setpoints", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::JointState& command) {
        this->jointSetpointsCallback(command, MotionSourceGate::Source::SupervisorDirect);
      });
  sub_mode_state_ = this->create_subscription<mobotic_interfaces::msg::VehicleModeState>(
      "vehicle/mode_state", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const mobotic_interfaces::msg::VehicleModeState& state) { this->modeStateCallback(state); });

  feedback_timer_ = this->create_timer(std::chrono::duration<double>{feedback_request_period},
                                       [this]() { this->feedbackTimerCallback(); });

  if (traction_pdo_enabled_ || telemetry_pdo_enabled_)
  {
    traction_pdo_timer_ = this->create_timer(std::chrono::duration<double>{traction_pdo_period},
                                             [this]() { this->tractionPdoTimerCallback(); });
  }

  watchdog_timer_setpoints_ = this->create_wall_timer(std::chrono::duration<double>{watchdog_timeout_},
                                                 [this]() { this->watchdogTimerCallback(); });
  management_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
      [this]() { this->enforceManagementLease(); this->enforceFeedbackFreshness(); this->enforceModeAuthority(); });
  diagnostics_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      [this]() { this->publishDiagnostics(); });

  timer_can_heartbeat_ =
      this->create_timer(std::chrono::duration<double>{can_heartbeat_period}, [this, driver_can_node_id]() {
        if (driver_can_node_id.empty())
        {
          return;
        }
        pub_can_frames_->publish(
            stampFrame(createHeartbeatFrame(parseCanNodeId(driver_can_node_id)), "can", this->now()));
      });

  srv_initialize_all_drives_ = this->create_service<std_srvs::srv::Trigger>(
      "initialize_all_drives", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->initializeAllDrives();
        response->success = true;
        response->message = "Initialization requested.";
      });

  srv_disable_all_drives_ = this->create_service<std_srvs::srv::Trigger>(
      "disable_all_drives", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->disableAllDrives();
        response->success = true;
        response->message = "Disable requested.";
      });

  srv_enable_all_drives_ = this->create_service<std_srvs::srv::Trigger>(
      "enable_all_drives", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = management_lease_.permitsEnable(supervisor_timeout_);
        if (response->success) { this->enableAllDrives(); }
        response->message = response->success ? "Enable requested." : "Fresh supervisor enable required.";
      });

  srv_clear_errors_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_errors", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->clearErrorsOnAllDrives();
        response->success = true;
        response->message = "Clear-error request sent.";
      });
}

MoboticDriver::~MoboticDriver()
{
  sendStopToAllDrives();
  disableAllDrives();
}

void MoboticDriver::configureWheelModules(const std::vector<std::string>& module_names,
                                          const std::vector<std::string>& steering_drives,
                                          const std::vector<std::string>& traction_drives)
{
  const bool explicit_mapping = !module_names.empty() || !steering_drives.empty() || !traction_drives.empty();

  if (explicit_mapping)
  {
    if (module_names.empty() || module_names.size() != steering_drives.size() ||
        module_names.size() != traction_drives.size())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "[mobotic_driver] modules.names, modules.steering_drives, and modules.traction_drives must have "
                   "the same non-zero size.");
      rclcpp::shutdown();
      std::exit(1);
    }

    for (std::size_t i = 0; i < module_names.size(); ++i)
    {
      if (module_names.at(i).empty() || drives_.count(steering_drives.at(i)) == 0 ||
          drives_.count(traction_drives.at(i)) == 0)
      {
        RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] Invalid drive mapping for wheel module '"
                                                   << module_names.at(i) << "'.");
        rclcpp::shutdown();
        std::exit(1);
      }

      if (!wheel_modules_.emplace(module_names.at(i),
                                  WheelModuleBinding{steering_drives.at(i), traction_drives.at(i)})
               .second)
      {
        RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] Duplicate wheel module name '"
                                                   << module_names.at(i) << "'.");
        rclcpp::shutdown();
        std::exit(1);
      }
    }
  }
  else
  {
    constexpr const char* steering_suffix = "_steering";
    constexpr const char* traction_suffix = "_traction";

    for (const auto& [drive_name, drive] : drives_)
    {
      (void)drive;
      if (endsWith(drive_name, steering_suffix))
      {
        const auto module_name = drive_name.substr(0, drive_name.size() - std::char_traits<char>::length(steering_suffix));
        wheel_modules_[module_name].steering_drive = drive_name;
      }
      else if (endsWith(drive_name, traction_suffix))
      {
        const auto module_name = drive_name.substr(0, drive_name.size() - std::char_traits<char>::length(traction_suffix));
        wheel_modules_[module_name].traction_drive = drive_name;
      }
    }
  }

  if (wheel_modules_.empty())
  {
    RCLCPP_FATAL(this->get_logger(),
                 "[mobotic_driver] No wheel modules configured. Set modules.* parameters or use drive names ending "
                 "in _steering and _traction.");
    rclcpp::shutdown();
    std::exit(1);
  }

  std::set<std::string> mapped_drives;
  for (const auto& [module_name, binding] : wheel_modules_)
  {
    if (binding.steering_drive.empty() || binding.traction_drive.empty() ||
        drives_.count(binding.steering_drive) == 0 || drives_.count(binding.traction_drive) == 0 ||
        binding.steering_drive == binding.traction_drive)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] Wheel module '" << module_name
                                                                                  << "' has an invalid steering/traction "
                                                                                     "mapping.");
      rclcpp::shutdown();
      std::exit(1);
    }

    if (!mapped_drives.insert(binding.steering_drive).second ||
        !mapped_drives.insert(binding.traction_drive).second)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] A drive is assigned to more than one wheel module; "
                                                   "conflict at '" << module_name << "'.");
      rclcpp::shutdown();
      std::exit(1);
    }

    auto& steering = drives_.at(binding.steering_drive);
    auto& traction = drives_.at(binding.traction_drive);
    steering.setTractionController(false);
    traction.setTractionController(true);

    if (steering.targetMode() != POSITION ||
        (traction.targetMode() != VELOCITY && traction.targetMode() != CURRENT))
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] Wheel module '" << module_name
                                                                                  << "' must use position steering and "
                                                                                     "velocity/current traction.");
      rclcpp::shutdown();
      std::exit(1);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Wheel module '" << module_name << "': steering='"
                                                              << binding.steering_drive << "', traction='"
                                                              << binding.traction_drive << "'.");
  }

  if (mapped_drives.size() != drives_.size())
  {
    RCLCPP_FATAL_STREAM(this->get_logger(), "[mobotic_driver] Wheel-module mappings cover " << mapped_drives.size()
                                                                                              << " of " << drives_.size()
                                                                                              << " configured drives.");
    rclcpp::shutdown();
    std::exit(1);
  }
}

void MoboticDriver::publishWheelModuleStatus(const rclcpp::Time& stamp)
{
  const auto steady_now = FeedbackFreshness::Clock::now();
  mobotic_interfaces::msg::WheelModuleStatusArray array;
  array.header.stamp = stamp;
  array.header.frame_id = "can";
  array.modules.reserve(wheel_modules_.size());

  for (const auto& [module_name, binding] : wheel_modules_)
  {
    const auto& steering = drives_.at(binding.steering_drive);
    const auto& traction = drives_.at(binding.traction_drive);

    mobotic_interfaces::msg::WheelModuleStatus status;
    status.name = module_name;
    status.enabled = steering.enabled() && traction.enabled();
    status.feedback_fresh = driveFeedbackFresh(steering, steady_now) && driveFeedbackFresh(traction, steady_now);
    const auto steering_problem = feedbackProblem(steering, steady_now);
    const auto traction_problem = feedbackProblem(traction, steady_now);
    status.status_message = !steering_problem.empty() ? binding.steering_drive + ": " + steering_problem :
        !traction_problem.empty() ? binding.traction_drive + ": " + traction_problem : "Feedback fresh";
    const bool steering_status_fresh = steering.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now);
    const bool traction_status_fresh = traction.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now);
    status.enabled = status.enabled && steering_status_fresh && traction_status_fresh;
    status.brake_state_known = false;
    status.brake_released = false;
    status.control_mode = traction_status_fresh ? toStatusControlMode(effectiveMode(traction)) :
        mobotic_interfaces::msg::WheelModuleStatus::CONTROL_MODE_UNKNOWN;
    const auto current = last_motor_current_ma_.find(binding.traction_drive);
    status.motor_current = current != last_motor_current_ma_.end() &&
                           traction.feedbackFreshness().fresh(FeedbackFreshness::CURRENT, motion_feedback_timeout_, steady_now)
                               ? current->second / 1000.0
                               : std::numeric_limits<double>::quiet_NaN();
    status.status_word = traction.hasStatusword() && traction_status_fresh
                             ? traction.statusword()
                             : 0;

    const int traction_error = traction.getErrorCode() != 0 ? traction.getErrorCode() : traction.getError();
    const int steering_error = steering.getErrorCode() != 0 ? steering.getErrorCode() : steering.getError();
    status.error_code = traction_error != 0 ? traction_error : steering_error;
    array.modules.push_back(status);
  }

  pub_wheel_module_status_->publish(array);
}

void MoboticDriver::canFrameCallback(const can_msgs::msg::Frame& frame)
{
  if (frame.is_error)
  {
    ++can_error_count_;
    last_can_error_ = ManagementLease::Clock::now();
    return;
  }
  if (frame.is_rtr || frame.is_extended || frame.dlc > 8) { return; }
  if (sourceStampNanoseconds(frame) < 0) { return; }
  // A short SDO frame must never update cached drive state.
  if (frame.id >= 0x580 && frame.id <= 0x5FF && frame.dlc != 8) { return; }
  sensor_msgs::msg::JointState feedback_msg;
  bool all_feedbacks_available = true;
  bool all_currents_available = true;
  const auto steady_now = FeedbackFreshness::Clock::now();
  const auto ros_now = this->now();
  // Account for queued CAN frames when the bridge supplies acquisition timestamps.
  // Zero stamps from other CAN transports fall back to local reception time.
  const rclcpp::Time acquisition_stamp(frame.header.stamp);
  const double acquisition_age = acquisition_stamp.nanoseconds() == 0 ? 0.0 : (ros_now - acquisition_stamp).seconds();
  if (acquisition_age < -0.1 || acquisition_age > std::max({motion_feedback_timeout_, status_feedback_timeout_, fault_feedback_timeout_})) { return; }
  const auto sample_time = steady_now - std::chrono::duration_cast<FeedbackFreshness::Clock::duration>(
      std::chrono::duration<double>(std::max(0.0, acquisition_age)));
  int64_t oldest_motion_stamp_ns = ros_now.nanoseconds();
  const auto sample_stamp_ns = acquisition_stamp.nanoseconds() == 0 ? ros_now.nanoseconds() :
      std::min(ros_now.nanoseconds(), acquisition_stamp.nanoseconds());

  for (auto& [name, drive] : drives_)
  {
    (void)name;

    const auto node_id = drive.canNodeID() - 0x600;
    const bool is_sdo = frame.id == 0x580 + node_id && frame.dlc == 8;
    const bool is_pdo = (frame.id == 0x180 + node_id || frame.id == 0x280 + node_id ||
                         frame.id == 0x380 + node_id || frame.id == 0x480 + node_id) && frame.dlc > 0;
    if (is_sdo || is_pdo || (frame.id == 0x700 + node_id && frame.dlc == 1))
    {
      last_can_feedback_[name].record(sample_time, acquisition_stamp.nanoseconds());
    }
    if (drive.isTractionController() && is_sdo && frame.data[3] == 0)
    {
      const unsigned index = static_cast<unsigned>(frame.data[1]) |
                             (static_cast<unsigned>(frame.data[2]) << 8);
      if (index == 0x2086 || index == 0x60FD)
      {
        auto& sample = index == 0x2086 ? sto_status_[name] : digital_inputs_[name];
        sample.update(frame, sample_time);
      }
    }

    const auto velocity_generation = drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY);
    const auto position_generation = drive.feedbackFreshness().generation(FeedbackFreshness::POSITION);
    const auto frames_to_send = drive.update(frame, sample_time);
    if (drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY) != velocity_generation)
    {
      motion_sample_stamp_ns_[name][0] = sample_stamp_ns;
    }
    if (drive.feedbackFreshness().generation(FeedbackFreshness::POSITION) != position_generation)
    {
      motion_sample_stamp_ns_[name][1] = sample_stamp_ns;
    }
    for (const auto& frame_to_send : frames_to_send)
    {
      pub_can_frames_->publish(stampFrame(frame_to_send, "can", this->now()));
    }

    if (drive.hasFeedback(CURRENT_FEEDBACK))
    {
      last_motor_current_ma_[drive.id()] = drive.feedback(CURRENT_FEEDBACK);
    }

    const bool needs_position_feedback = !drive.isTractionController();
    const bool drive_feedback_available =
        drive.enabled() && driveFeedbackFresh(drive, steady_now) && drive.hasFeedback(VELOCITY_FEEDBACK) &&
        (!needs_position_feedback || drive.hasFeedback(POSITION_FEEDBACK));

    all_feedbacks_available = all_feedbacks_available && drive_feedback_available;

    if (!drive_feedback_available)
    {
      continue;
    }
    oldest_motion_stamp_ns = std::min(oldest_motion_stamp_ns, motion_sample_stamp_ns_.at(name)[0]);
    if (needs_position_feedback)
    {
      oldest_motion_stamp_ns = std::min(oldest_motion_stamp_ns, motion_sample_stamp_ns_.at(name)[1]);
    }

    feedback_msg.name.emplace_back(drive.id());
    all_currents_available = all_currents_available && drive.hasFeedback(CURRENT_FEEDBACK) &&
        drive.feedbackFreshness().fresh(FeedbackFreshness::CURRENT, motion_feedback_timeout_, steady_now);
    feedback_msg.effort.push_back(drive.hasFeedback(CURRENT_FEEDBACK) ? drive.feedback(CURRENT_FEEDBACK) / 1000.0 : 0.0);
    feedback_msg.velocity.push_back(drive.hasFeedback(VELOCITY_FEEDBACK) ? drive.velocityFeedbackRadPerSec() : 0.0);

    if (needs_position_feedback)
    {
      feedback_msg.position.push_back(ticksToAngle(drive.feedback(POSITION_FEEDBACK), drive.resolution()));
    }
    else
    {
      feedback_msg.position.push_back(0.0); // Elmo traction has no MiControl steering position object 0x3762
    }
  }

  if (all_feedbacks_available && feedbacks_requested_ && feedback_msg.name.size() == drives_.size())
  {
    const rclcpp::Time feedback_stamp(oldest_motion_stamp_ns, ros_now.get_clock_type());
    feedback_msg.header.stamp = feedback_stamp;
    feedback_msg.header.frame_id = "can";
    if (!all_currents_available) { feedback_msg.effort.clear(); }
    pub_joint_states_->publish(feedback_msg);
    publishWheelModuleStatus(ros_now);

    for (auto& [name, drive] : drives_)
    {
      (void)name;
      drive.resetAllFeedbacks();
    }

    feedbacks_requested_ = false;
  }
  enforceFeedbackFreshness();
}

void MoboticDriver::tractionPdoTimerCallback()
{
  enforceManagementLease();
  enforceFeedbackFreshness();
  enforceModeAuthority();
  if (!enable_all_drives_requested_)
  {
    return;
  }

  if (traction_pdo_enabled_)
  {
    publishTractionPdoCycle();
  }
  else if (telemetry_pdo_enabled_)
  {
    pub_can_frames_->publish(stampFrame(MoboticDriveCanCommunication::syncFrame(), "can", this->now()));
  }
}

void MoboticDriver::publishTractionPdoCycle()
{
  bool published_traction_command = false;

  for (auto& [name, drive] : drives_)
  {
    (void)name;
    if (!drive.supportsTractionVelocityPdo())
    {
      continue;
    }
    if (feedback_motion_blocked_ || !allDriveFeedbackFresh(FeedbackFreshness::Clock::now()) ||
        !motion_source_gate_.fresh(this->now().nanoseconds(), supervisor_timeout_))
    {
      drive.setTractionPdoTarget(0);
    }

    pub_can_frames_->publish(stampFrame(drive.tractionPdoCommandFrame(), "can", this->now()));
    published_traction_command = true;
  }

  // Steering telemetry PDOs are synchronous as well. Keep emitting SYNC when
  // telemetry is enabled even when every traction module is current-controlled
  // and therefore has no velocity RPDO command to publish.
  if (published_traction_command || telemetry_pdo_enabled_)
  {
    pub_can_frames_->publish(stampFrame(MoboticDriveCanCommunication::syncFrame(), "can", this->now()));
  }
}

void MoboticDriver::feedbackTimerCallback()
{
  enforceManagementLease();
  enforceFeedbackFreshness();
  const auto now = this->now();
  const auto shouldPoll = [now](std::map<std::string, rclcpp::Time>& attempts,
                                const std::string& name,
                                const double period_seconds) {
    const auto attempt = attempts.find(name);
    if (attempt == attempts.end() || (now - attempt->second).seconds() >= period_seconds)
    {
      attempts[name] = now;
      return true;
    }
    return false;
  };
  const auto shouldRetry = [&shouldPoll](std::map<std::string, rclcpp::Time>& attempts,
                                         const std::string& name) {
    return shouldPoll(attempts, name, 0.2);
  };

  for (auto& [name, drive] : drives_)
  {
    // Status/fault polling must continue in every state, including PDO operation.
    if (shouldPoll(last_status_poll_, name, std::min(0.1, status_feedback_timeout_ / 3.0)))
    {
      pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", now));
    }
    if (shouldPoll(last_error_poll_, name, diagnostic_sdo_period_))
    {
      pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", now));
      if (drive.isTractionController())
      {
        pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", now));
      }
    }
    // Read-only ELMO safety telemetry, also when disabled or faulted.
    if (drive.isTractionController() && shouldPoll(last_safety_poll_, name, diagnostic_sdo_period_))
    {
      for (const unsigned index : {0x2086u, 0x60FDu})
      {
        pub_can_frames_->publish(stampFrame(
            createAndFillMoboticHeader(drive.canNodeID(), READ_FROM_DEVICE, index, 0), "can", now));
      }
    }
    if (initialize_all_drives_requested_ && (drive.hasError() || !drive.initialized()))
    {
      if (shouldRetry(last_initialize_attempt_, name))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Waiting for drive " << name << " to initialize.");
        if (drive.hasError())
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "[driver] Clearing error " << driveErrorText(drive)
                                                                             << " on drive " << name << ".");
          pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
        }
        pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
        if (drive.isTractionController())
        {
          pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", this->now()));
        }
      }
      continue;
    }

    if (enable_all_drives_requested_ && !drive.enabled())
    {
      if (shouldRetry(last_enable_attempt_, name))
      {
        if (drive.hasStatusword())
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Waiting for drive "
                                                    << name << " to enable (statusword=0x" << std::hex
                                                    << drive.statusword() << std::dec << ").");
        }
        else
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Waiting for drive " << name << " to enable.");
        }
        if (drive.hasError())
        {
          pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
        }
        if (drive.currentMode() != drive.targetMode())
        {
          pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now()));
        }
        // Clear old targets before releasing enable/brakes on a retry too.
        const auto idle_input = drive.isSteeringController() ? STEERING_PROFILE_VELOCITY_INPUT : modeToInputId(drive.targetMode());
        if (!(traction_pdo_enabled_ && drive.supportsTractionVelocityPdo()))
        {
          pub_can_frames_->publish(stampFrame(drive.setTargetRequest(idle_input, 0), "can", this->now()));
        }
        if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
        {
          drive.setTractionPdoTarget(0);
          drive.setTractionPdoControlword(0x0006);
          publishTractionPdoCycle();
          drive.setTractionPdoControlword(0x0007);
          publishTractionPdoCycle();
          drive.setTractionPdoControlword(0x000F);
          publishTractionPdoCycle();
        }
        else
        {
          for (const auto& frame : drive.changeDeviceStateRequests(ENABLE))
          {
            pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
          }
        }
        if (drive.targetMode() == VELOCITY)
        {
          pub_can_frames_->publish(stampFrame(
              drive.setTargetRequest(EC_TRACTION_MAX_PROFILE_VELOCITY, traction_max_profile_velocity_), "can",
              this->now()));
          pub_can_frames_->publish(
              stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_ACCEL, traction_profile_accel_), "can", this->now()));
          pub_can_frames_->publish(
              stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_DECEL, traction_profile_decel_), "can", this->now()));
          if (!traction_pdo_enabled_)
          {
            pub_can_frames_->publish(stampFrame(drive.setTargetRequest(VELOCITY_INPUT, 0), "can", this->now()));
          }
        }
        pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
        if (drive.isTractionController())
        {
          pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", this->now()));
        }
        pub_can_frames_->publish(stampFrame(drive.readDeviceModeRequest(), "can", this->now()));
        pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
      }
      continue;
    }

    if (drive.hasError())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Drive " << drive.id() << " has error " << driveErrorText(drive));
      continue;
    }

    if (!drive.enabled())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Drive " << drive.id() << " is not enabled.");
      pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
      continue;
    }

    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
    {
      continue;
    }

    if (telemetry_pdo_enabled_ && drive.isSteeringController())
    {
      continue;
    }

    if (!(telemetry_pdo_enabled_ && drive.isSteeringController()))
    {
      pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(CURRENT_FEEDBACK), "can", this->now()));
    }
    pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(VELOCITY_FEEDBACK), "can", this->now()));

    if (!drive.isTractionController())
    {
      pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(POSITION_FEEDBACK), "can", this->now()));
    }
  }

  feedbacks_requested_ = true;
  // This periodic publication also reports startup, disabled, and faulted
  // states, for which a complete JointState feedback cycle is unavailable.
  publishWheelModuleStatus(now);
}

void MoboticDriver::jointSetpointsCallback(const sensor_msgs::msg::JointState& msg, MotionSourceGate::Source source)
{
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1'000'000'000u) { return; }
  const rclcpp::Time command_stamp(msg.header.stamp);
  enforceModeAuthority();
  if (!motion_source_gate_.permits(source, command_stamp.nanoseconds(), this->now().nanoseconds(),
                                  supervisor_timeout_)) { return; }
  if (command_stamp.nanoseconds() < motion_accept_after_ns_) { return; }
  if (command_stamp.nanoseconds() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints without a timestamp.");
    return;
  }

  const double command_age = (this->now() - command_stamp).seconds();
  if (command_age > watchdog_timeout_ || command_age < -0.1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints with invalid age: "
                                                 << command_age << " s.");
    return;
  }

  if (msg.name.size() != drives_.size())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints containing "
                                                 << msg.name.size() << " joints; expected " << drives_.size() << ".");
    return;
  }

  std::set<std::string> received_joints;
  for (const auto& name : msg.name)
  {
    if (drives_.find(name) == drives_.end() || !received_joints.insert(name).second)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting unknown or duplicate joint '" << name
                                                                                                          << "'.");
      return;
    }
  }

  for (const auto& [name, drive] : drives_)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), name);
    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    const DeviceMode mode = effectiveMode(drive);
    const bool valid_position = hasIndex(msg.position, index) && std::isfinite(msg.position[index]);
    const bool valid_velocity = hasIndex(msg.velocity, index) && std::isfinite(msg.velocity[index]);
    const bool valid_effort = hasIndex(msg.effort, index) && std::isfinite(msg.effort[index]);

    if ((mode == POSITION && (!valid_position || !valid_velocity)) ||
        (mode == VELOCITY && !valid_velocity) || (mode == CURRENT && !valid_effort))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting incomplete joint setpoint for '" << name
                                                                                                             << "'.");
      return;
    }
    if (mode == POSITION && !drive.steeringPositionRepresentable(msg.position[index]))
    {
      // Validate every joint before applyJointSetpoints can send any CAN frame.
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting entire joint command: steering target for '"
          << name << "' cannot be represented as signed 32-bit encoder ticks.");
      return;
    }
  }

  applyJointSetpoints(msg);
}

void MoboticDriver::wheelModuleCommandCallback(const mobotic_interfaces::msg::WheelModuleCommand& msg)
{
  const rclcpp::Time command_stamp(msg.header.stamp);
  if (command_stamp.nanoseconds() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "[mobotic_driver] Rejecting wheel-module management command without a timestamp.");
    return;
  }

  const double command_age = (this->now() - command_stamp).seconds();
  if (command_age > watchdog_timeout_ || command_age < -0.1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "[mobotic_driver] Rejecting wheel-module management command with invalid age: "
                           << command_age << " s.");
    return;
  }

  if (command_stamp.nanoseconds() <= last_management_stamp_ns_)
  {
    return; // Replayed commands cannot keep the supervisor lease alive.
  }
  last_management_stamp_ns_ = command_stamp.nanoseconds();
  // Observe any expired previous lease before a new command can restore it.
  enforceManagementLease();
  management_lease_.refresh(msg.enable);
  management_disable_sent_ = false;

  if (msg.clear_errors)
  {
    clearErrorsOnAllDrives();
  }

  if (msg.enable)
  {
    // The supervisor republishes the desired state every control cycle.
    // Preserve active motion targets and the existing enable retry sequence.
    if (!enable_all_drives_requested_)
    {
      enableAllDrives();
    }
  }
  else if (!module_management_received_ || enable_all_drives_requested_ ||
           std::any_of(drives_.begin(), drives_.end(),
                       [](const auto& entry) { return entry.second.enabled(); }))
  {
    sendStopToAllDrives();
    disableAllDrives();
  }
  module_management_received_ = true;
}

void MoboticDriver::clearErrorsOnAllDrives()
{
  RCLCPP_INFO(this->get_logger(), "[driver] Clearing errors on all drives.");
  for (auto& [name, drive] : drives_)
  {
    (void)name;
    pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
  }
}

void MoboticDriver::applyJointSetpoints(const sensor_msgs::msg::JointState& msg)
{
  enforceManagementLease();
  enforceFeedbackFreshness();
  if (feedback_motion_blocked_ || rclcpp::Time(msg.header.stamp).nanoseconds() < motion_accept_after_ns_) { return; }
  if (!management_lease_.permitsEnable(supervisor_timeout_) || !enable_all_drives_requested_) { return; }
  bool command_sent = false;

  for (auto& [name, drive] : drives_)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), drive.id());

    if (it == msg.name.end())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "No setpoint for drive '" << drive.id() << "' in joint command.");
      continue;
    }

    if (!drive.enabled())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "Ignoring setpoint for disabled drive '" << drive.id() << "'.");
      continue;
    }

    const auto idx = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    const DeviceMode mode = effectiveMode(drive);
    const ParameterIds input_id = modeToInputId(mode);

    if (input_id == ABSOLUTE_POSITION_INPUT)
    {
      if (!hasIndex(msg.position, idx))
      {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                    "Position-mode drive '" << drive.id() << "' has no position setpoint.");
        continue;
      }

      if (!hasIndex(msg.velocity, idx))
      {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                    "Position-mode drive '" << drive.id() << "' has no velocity setpoint.");
        continue;
      }

      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(STEERING_PROFILE_VELOCITY_INPUT,
                                                                 drive.getTarget(STEERING_PROFILE_VELOCITY_INPUT,
                                                                                 msg.velocity.at(idx))),
                                          "can", this->now()));
      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(ABSOLUTE_POSITION_INPUT,
                                                                 drive.getTarget(ABSOLUTE_POSITION_INPUT,
                                                                                 msg.position.at(idx))),
                                          "can", this->now()));
      command_sent = true;
      continue;
    }

    if (input_id == CURRENT_INPUT)
    {
      if (!hasIndex(msg.effort, idx))
      {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                    "Current-mode drive '" << drive.id() << "' has no effort setpoint.");
        continue;
      }

      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(CURRENT_INPUT,
                                                                 drive.getTarget(CURRENT_INPUT, msg.effort.at(idx))),
                                          "can", this->now()));
      command_sent = true;
      continue;
    }

    // Elmo traction velocity mode: msg.velocity is expected in wheel/output rad/s.
    if (!hasIndex(msg.velocity, idx))
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "Velocity-mode drive '" << drive.id() << "' has no velocity setpoint.");
      continue;
    }

    const int target = drive.getTarget(input_id, msg.velocity.at(idx));
    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo() && input_id == EC_TRACTION_VELOCITY_INPUT)
    {
      drive.setTractionPdoTarget(target);
    }
    else
    {
      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(input_id, target), "can", this->now()));
    }
    command_sent = true;
  }

  if (command_sent)
  {
    motion_command_received_ = true;
    last_motion_command_received_ = ManagementLease::Clock::now();
    watchdog_timer_setpoints_->reset();
  }
}

void MoboticDriver::sendStopToAllDrives()
{
  for (auto& [name, drive] : drives_)
  {
    (void)name;
    if (!drive.enabled())
    {
      continue;
    }

    const DeviceMode mode = effectiveMode(drive);
    const ParameterIds input_id = modeToInputId(mode);
    const ParameterIds stop_input_id = input_id == ABSOLUTE_POSITION_INPUT ? STEERING_PROFILE_VELOCITY_INPUT : input_id;

    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo() && stop_input_id == EC_TRACTION_VELOCITY_INPUT)
    {
      drive.setTractionPdoTarget(0);
    }
    else
    {
      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(stop_input_id, 0), "can", this->now()));
    }
  }

  if (traction_pdo_enabled_)
  {
    publishTractionPdoCycle();
  }
}

void MoboticDriver::disableAllDrives()
{
  RCLCPP_INFO(this->get_logger(), "[driver] Disabling all drives.");
  enable_all_drives_requested_ = false;
  last_enable_attempt_.clear();

  for (auto& [name, drive] : drives_)
  {
    (void)name;
    if (drive.isTractionController())
    {
      const ParameterIds stop_input_id = modeToInputId(effectiveMode(drive));
      if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
      {
        drive.setTractionPdoTarget(0);
        drive.setTractionPdoControlword(0x0000);
      }
      else
      {
        pub_can_frames_->publish(stampFrame(drive.setTargetRequest(stop_input_id, 0), "can", this->now()));
      }
    }
    if (!(traction_pdo_enabled_ && drive.supportsTractionVelocityPdo()))
    {
      for (const auto& frame : drive.changeDeviceStateRequests(DISABLE))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
    }
    pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
  }

  if (traction_pdo_enabled_)
  {
    publishTractionPdoCycle();
  }
}

void MoboticDriver::enableAllDrives()
{
  if (!management_lease_.permitsEnable(supervisor_timeout_)) { return; }
  motion_accept_after_ns_ = this->now().nanoseconds();
  motion_command_received_ = false;
  RCLCPP_INFO(this->get_logger(), "[driver] Enabling all drives.");
  initialize_all_drives_requested_ = true;
  enable_all_drives_requested_ = true;
  last_initialize_attempt_.clear();
  last_enable_attempt_.clear();

  for (auto& [name, drive] : drives_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Sending enable sequence to drive " << name << ".");
    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
    {
      for (const auto& frame : drive.tractionPdoConfigurationRequests(
               static_cast<unsigned>(telemetry_pdo_sync_divider_)))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
    }
    else if (telemetry_pdo_enabled_ && drive.isSteeringController())
    {
      for (const auto& frame : drive.steeringTelemetryPdoConfigurationRequests(
               static_cast<unsigned>(telemetry_pdo_sync_divider_)))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
    }
    pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
    pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now()));
    const auto idle_input = drive.isSteeringController() ? STEERING_PROFILE_VELOCITY_INPUT : modeToInputId(drive.targetMode());
    if (!(traction_pdo_enabled_ && drive.supportsTractionVelocityPdo()))
    {
      pub_can_frames_->publish(stampFrame(drive.setTargetRequest(idle_input, 0), "can", this->now()));
    }
    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
    {
      drive.setTractionPdoTarget(0);
      drive.setTractionPdoControlword(0x0006);
      publishTractionPdoCycle();
      drive.setTractionPdoControlword(0x0007);
      publishTractionPdoCycle();
      drive.setTractionPdoControlword(0x000F);
      publishTractionPdoCycle();
    }
    else
    {
      for (const auto& frame : drive.changeDeviceStateRequests(ENABLE))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
    }
    if (drive.targetMode() == VELOCITY)
    {
      pub_can_frames_->publish(stampFrame(
          drive.setTargetRequest(EC_TRACTION_MAX_PROFILE_VELOCITY, traction_max_profile_velocity_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_ACCEL, traction_profile_accel_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_DECEL, traction_profile_decel_), "can", this->now()));
      if (!traction_pdo_enabled_)
      {
        pub_can_frames_->publish(stampFrame(drive.setTargetRequest(VELOCITY_INPUT, 0), "can", this->now()));
      }
    }
    pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
    if (drive.isTractionController())
    {
      pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", this->now()));
    }
    pub_can_frames_->publish(stampFrame(drive.readDeviceModeRequest(), "can", this->now()));
    pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
  }
}

void MoboticDriver::initializeAllDrives()
{
  RCLCPP_INFO(this->get_logger(), "[driver] Initializing all drives.");
  initialize_all_drives_requested_ = true;
  last_initialize_attempt_.clear();

  for (auto& [name, drive] : drives_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing drive " << name << ".");
    if (traction_pdo_enabled_ && drive.supportsTractionVelocityPdo())
    {
      for (const auto& frame : drive.tractionPdoConfigurationRequests(
               static_cast<unsigned>(telemetry_pdo_sync_divider_)))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
      pub_can_frames_->publish(stampFrame(
          drive.setTargetRequest(EC_TRACTION_MAX_PROFILE_VELOCITY, traction_max_profile_velocity_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_MAX_ACCEL, traction_profile_accel_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_MAX_DECEL, traction_profile_decel_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_ACCEL, traction_profile_accel_), "can", this->now()));
      pub_can_frames_->publish(
          stampFrame(drive.setTargetRequest(EC_TRACTION_PROFILE_DECEL, traction_profile_decel_), "can", this->now()));
    }
    else if (telemetry_pdo_enabled_ && drive.isSteeringController())
    {
      for (const auto& frame : drive.steeringTelemetryPdoConfigurationRequests(
               static_cast<unsigned>(telemetry_pdo_sync_divider_)))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
    }
    pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now()));
    pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
    pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
    if (drive.isTractionController())
    {
      pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", this->now()));
    }
  }
}

can_msgs::msg::Frame MoboticDriver::stampFrame(const can_msgs::msg::Frame& frame, const std::string& frame_id,
                                               const rclcpp::Time& stamp)
{
  auto stamped_frame = frame;
  stamped_frame.header.frame_id = frame_id;
  stamped_frame.header.stamp = stamp;
  return stamped_frame;
}

void MoboticDriver::watchdogTimerCallback()
{
  sendStopToAllDrives();
}

void MoboticDriver::enforceManagementLease()
{
  if (management_lease_.permitsEnable(supervisor_timeout_)) { return; }
  initialize_all_drives_requested_ = false;
  const auto steady_now = ManagementLease::Clock::now();
  // Repeat disable while hardware still reports enabled, but do not flood a stopped bus.
  if (enable_all_drives_requested_ || std::any_of(drives_.begin(), drives_.end(),
      [](const auto& entry) { return entry.second.enabled(); }))
  {
    if (management_disable_sent_ &&
        std::chrono::duration<double>(steady_now - last_management_disable_).count() < 0.1) { return; }
    sendStopToAllDrives();
    disableAllDrives();
    management_disable_sent_ = true;
    last_management_disable_ = steady_now;
  }
}

void MoboticDriver::modeStateCallback(const mobotic_interfaces::msg::VehicleModeState& msg)
{
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1'000'000'000u) { return; }
  const auto now_ns = this->now().nanoseconds();
  const bool was_fresh = motion_source_gate_.fresh(now_ns, supervisor_timeout_);
  const auto old_mode = motion_source_gate_.mode();
  const bool was_transitioning = motion_source_gate_.transitioning();
  const auto generation = motion_source_gate_.generation();
  if (!motion_source_gate_.update(msg.current_mode, msg.transition_in_progress,
          rclcpp::Time(msg.header.stamp).nanoseconds(), now_ns, supervisor_timeout_)) { return; }
  if (generation != motion_source_gate_.generation())
  {
    motion_accept_after_ns_ = std::max(motion_accept_after_ns_, motion_source_gate_.boundary());
    motion_command_received_ = false;
    // Do not interrupt the old path's controlled deceleration at transition
    // start. At completion/recovery clear cached PDO/steering targets before
    // permitting any new source, including same-mode reselection.
    if (!was_fresh || old_mode != msg.current_mode ||
        (was_transitioning && !msg.transition_in_progress)) { sendStopToAllDrives(); }
  }
  mode_stop_sent_ = false;
}

void MoboticDriver::enforceModeAuthority()
{
  if (motion_source_gate_.fresh(this->now().nanoseconds(), supervisor_timeout_)) { return; }
  const auto now = MotionSourceGate::Clock::now();
  if (!mode_stop_sent_ || std::chrono::duration<double>(now - last_mode_stop_).count() >= 0.1)
  {
    sendStopToAllDrives();
    motion_command_received_ = false;
    mode_stop_sent_ = true;
    last_mode_stop_ = now;
  }
}

bool MoboticDriver::driveFeedbackFresh(const MoboticDriveCanCommunication& drive,
    FeedbackFreshness::Clock::time_point now) const
{
  return drive.feedbackFreshness().ready(drive.isSteeringController(), motion_feedback_timeout_,
                                        status_feedback_timeout_, fault_feedback_timeout_, now);
}

std::string MoboticDriver::feedbackProblem(const MoboticDriveCanCommunication& drive,
    FeedbackFreshness::Clock::time_point now) const
{
  const auto& feedback = drive.feedbackFreshness();
  if (!feedback.fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, now)) { return "status stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::VELOCITY, motion_feedback_timeout_, now)) { return "velocity stale/missing"; }
  if (drive.isSteeringController() && !feedback.fresh(FeedbackFreshness::POSITION, motion_feedback_timeout_, now)) { return "position stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::ERROR_REGISTER, fault_feedback_timeout_, now)) { return "error register stale/missing"; }
  if (drive.isTractionController() && !feedback.fresh(FeedbackFreshness::ERROR_CODE, fault_feedback_timeout_, now)) { return "error code stale/missing"; }
  return "";
}

bool MoboticDriver::allDriveFeedbackFresh(FeedbackFreshness::Clock::time_point now) const
{
  return !drives_.empty() && std::all_of(drives_.begin(), drives_.end(),
      [this, now](const auto& entry) { return driveFeedbackFresh(entry.second, now); });
}

void MoboticDriver::enforceFeedbackFreshness()
{
  const auto steady_now = FeedbackFreshness::Clock::now();
  const bool blocked = !allDriveFeedbackFresh(steady_now) || std::any_of(drives_.begin(), drives_.end(),
      [](const auto& entry) { return !entry.second.enabled() || entry.second.hasError() || entry.second.getErrorCode() != 0; });
  if (blocked != feedback_motion_blocked_)
  {
    // Recovery never reuses targets or a command queued before feedback returned.
    motion_accept_after_ns_ = this->now().nanoseconds();
    motion_command_received_ = false;
    if (blocked)
    {
      for (auto& entry : drives_) { entry.second.resetAllFeedbacks(); }
      feedback_stop_sent_ = false;
    }
  }
  feedback_motion_blocked_ = blocked;
  if (blocked && (!feedback_stop_sent_ || std::chrono::duration<double>(steady_now - last_feedback_stop_).count() >= 0.1))
  {
    sendStopToAllDrives();
    feedback_stop_sent_ = true;
    last_feedback_stop_ = steady_now;
  }
}

void MoboticDriver::publishDiagnostics()
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  const auto steady_now = ManagementLease::Clock::now();
  const auto add_value = [](Status& status, const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key; entry.value = value; status.values.push_back(entry);
  };
  Status authority;
  authority.name = this->get_fully_qualified_name() + std::string("/supervisor_lease");
  authority.hardware_id = "MoboTerra";
  const bool fresh = management_lease_.fresh(supervisor_timeout_, steady_now);
  authority.level = fresh ? Status::OK : Status::ERROR;
  authority.message = fresh ? "Supervisor management received" : "Supervisor missing/stale: disable requested";
  add_value(authority, "enable_requested", enable_all_drives_requested_ ? "true" : "false");
  add_value(authority, "supervisor_timeout_s", std::to_string(supervisor_timeout_));
  array.status.push_back(authority);
  Status mode_authority;
  mode_authority.name = this->get_fully_qualified_name() + std::string("/mode_authority");
  mode_authority.hardware_id = "MoboTerra";
  const bool mode_fresh = motion_source_gate_.fresh(this->now().nanoseconds(), supervisor_timeout_, steady_now);
  mode_authority.level = mode_fresh ? Status::OK : Status::ERROR;
  mode_authority.message = mode_fresh ? "One selected motion path" : "Mode state missing/stale: motion stopped";
  add_value(mode_authority, "current_mode", std::to_string(motion_source_gate_.mode()));
  add_value(mode_authority, "transition_in_progress", motion_source_gate_.transitioning() ? "true" : "false");
  array.status.push_back(mode_authority);
  Status motion;
  motion.name = this->get_fully_qualified_name() + std::string("/motion_watchdog");
  motion.hardware_id = "MoboTerra";
  const bool motion_fresh = motion_command_received_ &&
      std::chrono::duration<double>(steady_now - last_motion_command_received_).count() <= watchdog_timeout_;
  motion.level = enable_all_drives_requested_ && !motion_fresh ? Status::WARN : Status::OK;
  motion.message = motion_fresh ? "Recent joint setpoints" : "No recent joint setpoints: stop requested/idle";
  array.status.push_back(motion);
  Status can;
  can.name = this->get_fully_qualified_name() + std::string("/can_errors");
  can.hardware_id = "MoboTerra";
  can.level = can_error_count_ > 0 &&
      std::chrono::duration<double>(steady_now - last_can_error_).count() <= 5.0 ? Status::WARN : Status::OK;
  can.message = can.level == Status::WARN ? "Recent CAN error frame" : "No recent CAN error frames";
  add_value(can, "error_frame_count", std::to_string(can_error_count_));
  array.status.push_back(can);
  for (const auto& [name, drive] : drives_)
  {
    Status status;
    status.name = this->get_fully_qualified_name() + std::string("/") + name;
    status.hardware_id = std::to_string(drive.canNodeID() - 0x600);
    const auto received = last_can_feedback_.find(name);
    const bool can_fresh = received != last_can_feedback_.end() && received->second.age(steady_now) >= 0.0 &&
        received->second.age(steady_now) <= std::max(0.5, 2.0 * diagnostic_sdo_period_);
    const auto feedback_problem = feedbackProblem(drive, steady_now);
    const bool faulted = drive.hasError() || drive.getErrorCode() != 0;
    status.level = !can_fresh || !feedback_problem.empty() ? Status::STALE : faulted ? Status::ERROR :
        enable_all_drives_requested_ && !drive.enabled() ? Status::WARN : Status::OK;
    status.message = !can_fresh ? "No recent CAN response" : !feedback_problem.empty() ? feedback_problem : faulted ? "Drive fault" :
        drive.enabled() ? "Enabled" : "Disabled";
    add_value(status, "enabled", drive.enabled() ? "true" : "false");
    add_value(status, "error", driveErrorText(drive));
    add_value(status, "feedback_fresh", feedback_problem.empty() ? "true" : "false");
    add_value(status, "motion_blocked", feedback_motion_blocked_ ? "true" : "false");
    for (const auto& field : {std::make_pair("status_age_s", FeedbackFreshness::STATUS),
                              std::make_pair("velocity_age_s", FeedbackFreshness::VELOCITY),
                              std::make_pair("error_register_age_s", FeedbackFreshness::ERROR_REGISTER)})
    {
      add_value(status, field.first, std::to_string(drive.feedbackFreshness().age(field.second, steady_now)));
    }
    if (drive.isTractionController())
    {
      for (const auto& item : {std::make_pair("sto_0x2086", &sto_status_),
                               std::make_pair("digital_inputs_0x60FD", &digital_inputs_)})
      {
        const auto sample = item.second->find(name);
        const bool valid = sample != item.second->end() && sample->second.available &&
            sample->second.ordering.age(steady_now) >= 0.0 &&
            sample->second.ordering.age(steady_now) <= 2.0 * diagnostic_sdo_period_;
        add_value(status, std::string(item.first) + "_fresh", valid ? "true" : "false");
        add_value(status, std::string(item.first) + "_raw", valid ? std::to_string(sample->second.value) : "unknown");
        if (!valid && status.level == Status::OK) { status.level = Status::WARN; status.message = "Safety register unavailable/stale"; }
      }
      add_value(status, "sto_decoding", "raw only: hardware bit mapping unconfirmed");
    }
    array.status.push_back(status);
  }
  pub_diagnostics_->publish(array);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  MoboticDriver driver{rclcpp::NodeOptions()};
  rclcpp::spin(driver.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
