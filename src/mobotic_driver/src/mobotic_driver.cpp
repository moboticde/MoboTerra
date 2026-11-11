#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>
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
  this->declare_parameter("startup_timeout", 15.0);
  this->declare_parameter("steering_align_on_enable", false);
  this->declare_parameter("steering_alignment_velocity", 0.6);
  this->declare_parameter("steering_alignment_timeout", 12.5);
  this->declare_parameter("diagnostic_sdo_period", 1.0);
  this->declare_parameter("software_enable_diagnostics", false);
  this->declare_parameter("supervisor_timeout", 0.3);
  this->declare_parameter("motion_feedback_timeout", 0.3);
  this->declare_parameter("status_feedback_timeout", 0.3);
  this->declare_parameter("fault_feedback_timeout", 2.5);
  if (this->declare_parameter("traction_pdo_enabled", false) ||
      this->declare_parameter("telemetry_pdo_enabled", false))
  { throw std::invalid_argument("this driver supports miControl SDO transport only"); }
  this->declare_parameter("virtual_position_source", false);
  this->declare_parameter("sample_skew_budget", 0.005);
  this->declare_parameter("physical_sample_skew_budget", 0.1);
  this->declare_parameter("max_steering_velocity", 4.32);
  this->declare_parameter("max_traction_velocity", 3.614);
  this->declare_parameter("wheel_radius", 0.325);
  this->declare_parameter("max_traction_current", 4.0);
  this->declare_parameter("drives.names", std::vector<std::string>());
  this->declare_parameter("drives.modes", std::vector<std::string>());
  this->declare_parameter("drives.controller_types", std::vector<std::string>());
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
  std::vector<std::string> names;
  std::vector<std::string> modes;
  std::vector<std::string> controller_types;
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
  feedback_request_period_ = feedback_request_period;
  params &= this->get_parameter("can_heartbeat_period", can_heartbeat_period);
  params &= this->get_parameter("watchdog_timeout", watchdog_timeout_);
  this->get_parameter("startup_timeout", startup_timeout_);
  params &= this->get_parameter("diagnostic_sdo_period", diagnostic_sdo_period_);
  params &= this->get_parameter("software_enable_diagnostics", software_enable_diagnostics_);
  params &= this->get_parameter("supervisor_timeout", supervisor_timeout_);
  params &= this->get_parameter("motion_feedback_timeout", motion_feedback_timeout_);
  params &= this->get_parameter("status_feedback_timeout", status_feedback_timeout_);
  params &= this->get_parameter("fault_feedback_timeout", fault_feedback_timeout_);
  this->get_parameter("virtual_position_source", virtual_position_source_);
  this->get_parameter("sample_skew_budget", sample_skew_budget_);
  this->get_parameter("physical_sample_skew_budget", physical_sample_skew_budget_);
  this->get_parameter("max_steering_velocity", max_steering_velocity_);
  this->get_parameter("steering_align_on_enable", steering_align_on_enable_);
  this->get_parameter("steering_alignment_velocity", steering_alignment_velocity_);
  this->get_parameter("steering_alignment_timeout", steering_alignment_timeout_);
  if (!std::isfinite(steering_alignment_velocity_) || steering_alignment_velocity_ <= 0 ||
      !std::isfinite(steering_alignment_timeout_) || steering_alignment_timeout_ <= 0 ||
      (steering_align_on_enable_ && steering_alignment_timeout_ >= startup_timeout_))
  { throw std::invalid_argument("steering alignment requires a bounded positive velocity and timeout below startup_timeout"); }
  this->get_parameter("max_traction_velocity", max_traction_velocity_);
  this->get_parameter("wheel_radius", wheel_radius_);
  this->get_parameter("max_traction_current", max_traction_current_);
  if (!std::isfinite(sample_skew_budget_) || sample_skew_budget_ <= 0 ||
      sample_skew_budget_ > motion_feedback_timeout_ ||
      !std::isfinite(max_steering_velocity_) || max_steering_velocity_ <= 0 ||
      !std::isfinite(max_traction_velocity_) || max_traction_velocity_ <= 0 ||
      !std::isfinite(wheel_radius_) || wheel_radius_ <= 0 ||
      !std::isfinite(max_traction_current_) || max_traction_current_ <= 0 ||
      !std::isfinite(physical_sample_skew_budget_) || physical_sample_skew_budget_ <= 0 ||
      physical_sample_skew_budget_ > motion_feedback_timeout_)
  { throw std::invalid_argument("invalid motion limits or feedback sample skew budget"); }
  if (!std::isfinite(startup_timeout_) || startup_timeout_ <= 0)
  { throw std::invalid_argument("startup_timeout must be finite and positive"); }
  RCLCPP_INFO(this->get_logger(), "Drive transport: miControl SDO-only.");
  params &= this->get_parameter("drives.names", names);
  params &= this->get_parameter("drives.modes", modes);
  this->get_parameter("drives.controller_types", controller_types);
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

  if (!std::isfinite(motion_feedback_timeout_) || motion_feedback_timeout_ <= 0.0 ||
      !std::isfinite(status_feedback_timeout_) || status_feedback_timeout_ <= 0.0 ||
      !std::isfinite(fault_feedback_timeout_) || fault_feedback_timeout_ <= diagnostic_sdo_period_ ||
      !std::isfinite(feedback_request_period) || feedback_request_period <= 0.0 ||
      feedback_request_period >= status_feedback_timeout_)
  {
    throw std::invalid_argument("Feedback timeouts must be finite/positive; status timeout must exceed feedback period, fault timeout must exceed fault polling period");
  }

  const bool all_sizes_match =
      (controller_types.empty() || controller_types.size() == names.size()) &&
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
  if (!virtual_position_source_)
  { sdo_bus_scheduler_.useFinalSenderPacing(); }

  // Each motion sample needs separate position and velocity SDO requests.
  // Reject scans that cannot fit even without arbitration or reply delays.
  if (!virtual_position_source_ &&
      std::chrono::duration<double>(can_scheduling::minimum_gap * (2 * names.size() - 1)).count() >
          physical_sample_skew_budget_)
  {
    throw std::invalid_argument("physical_sample_skew_budget cannot fit two SDO motion reads per drive at the configured CAN gap");
  }

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

    if (steering_align_on_enable_ && mode == POSITION &&
        angVelToRpm(steering_alignment_velocity_, gear_ratios.at(i)) > max_steering_velocities.at(i))
    { throw std::invalid_argument("steering alignment exceeds native motor RPM limit"); }

    drives_.emplace(names.at(i), MoboticDriveCanCommunication(names.at(i), can_node_id, mode,
        static_cast<int>(resolutions.at(i)), gear_ratios.at(i), limits,
        controller_types.empty() ? "" : controller_types.at(i)));
    const auto traction_speed = [&](double value) {
      return rpmToAngVel(value, gear_ratios.at(i));
    };

    RCLCPP_INFO_STREAM(this->get_logger(), "Drive '" << names.at(i) << "': node=" << can_node_ids.at(i)
                                                      << ", mode=" << modes.at(i)
                                                      << ", controller=micontrol"
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
                                                << traction_speed(limits.at(MIN_TRACTION_VELOCITY))
                                                << " ... "
                                                << traction_speed(limits.at(MAX_TRACTION_VELOCITY))
                                                << " rad/s");

    drives_.at(names.at(i)).preserveControllerConfiguration(!virtual_position_source_);
    drives_.at(names.at(i)).setAutomaticModeInitialization(virtual_position_source_);
  }

  configureWheelModules(module_names, module_steering_drives, module_traction_drives);

  for (const auto& [name, drive] : drives_)
  {
    sdo_node_names_.emplace(drive.rawCanNodeID(), name);
    configuration_.try_emplace(name);
  }
  for (const auto& [node, name] : sdo_node_names_)
  { (void)name; sdo_nodes_.push_back(node); }

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()),
      [this](const can_msgs::msg::Frame& frame) { this->canFrameCallback(frame); });

  pub_joint_states_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  pub_wheel_module_status_ = this->create_publisher<mobotic_interfaces::msg::WheelModuleStatusArray>(
      "wheel_modules/status", rclcpp::QoS(rclcpp::KeepLast(1)));

  sub_wheel_module_command_ = this->create_subscription<mobotic_interfaces::msg::WheelModuleOperationCommand>(
      "wheel_modules/operation_command", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const mobotic_interfaces::msg::WheelModuleOperationCommand& command) {
        this->wheelModuleCommandCallback(command);
      });

  sub_final_command_ = this->create_subscription<mobotic_interfaces::msg::WheelModuleCommandArray>(
      "wheel_modules/command", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const mobotic_interfaces::msg::WheelModuleCommandArray& command) { this->finalCommandCallback(command); });
  sub_mode_state_ = this->create_subscription<mobotic_interfaces::msg::VehicleModeState>(
      "vehicle/mode", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const mobotic_interfaces::msg::VehicleModeState& state) { this->modeStateCallback(state); });

  feedback_timer_ = this->create_timer(std::chrono::duration<double>{feedback_request_period},
                                       [this]() { this->feedbackTimerCallback(); });

  if (!virtual_position_source_)
  {
    sdo_dispatch_timer_ = this->create_wall_timer(SdoBusScheduler::minimum_gap,
        [this]() { this->dispatchSdo(); });
    RCLCPP_INFO(this->get_logger(), "Physical SDO scheduling: %.1f ms gap enforced by %s; round-robin acknowledged requests.",
        std::chrono::duration<double, std::milli>(can_scheduling::minimum_gap).count(),
        "shared final sender");
  }

  watchdog_timer_setpoints_ = this->create_wall_timer(std::chrono::duration<double>{watchdog_timeout_},
                                                 [this]() { this->watchdogTimerCallback(); });
  management_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
      [this]() { this->enforceManagementLease(); this->advanceStartup(); this->enforceFeedbackFreshness(); this->enforceModeAuthority(); });
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
      "internal/driver/initialize_all_drives", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (enable_all_drives_requested_ || std::any_of(drives_.begin(), drives_.end(), [](const auto& item) { return item.second.enabled(); }))
        { response->success = false; response->message = "Cannot reconfigure while enabled."; return; }
        if (!virtual_position_source_)
        { response->success = false; response->message = "Physical initialization is owned by supervisor startup."; return; }
        this->initializeAllDrives();
        response->success = true;
        response->message = "Initialization requested.";
      });

  srv_disable_all_drives_ = this->create_service<std_srvs::srv::Trigger>(
      "internal/driver/disable_all_drives", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->disableAllDrives();
        response->success = true;
        response->message = "Disable requested.";
      });

  if (virtual_position_source_) { initializeAllDrives(); }
}

MoboticDriver::~MoboticDriver()
{
  sendStopToAllDrives();
  disableAllDrives(true);
  // The executor has stopped, so make one bounded best-effort disable pass.
  // Shutdown uses the same shared scheduler and gap; no burst exception.
  if (!virtual_position_source_ && controller_control_started_ &&
      get_node_base_interface()->get_context()->is_valid())
  {
    for (std::size_t slot = 0; slot < sdo_nodes_.size(); ++slot)
    {
      std::this_thread::sleep_for(can_scheduling::minimum_gap);
      const auto stamp = this->now();
      const auto frame = sdo_bus_scheduler_.next(sdo_nodes_, [this, stamp](unsigned node) {
        return sdo_channels_[node].next(watchdog_timeout_, SdoChannel::Clock::now(), stamp.nanoseconds());
      });
      if (!frame) { break; }
      pub_can_frames_->publish(stampFrame(*frame, "can", stamp));
      sdo_bus_scheduler_.published();
    }
  }
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
    if (steering.targetMode() != POSITION ||
        traction.targetMode() != VELOCITY)
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
    status.startup_pending = steeringAlignmentPending() || traction_mode_pending_ || enable_probe_after_ns_ != 0 || (!virtual_position_source_ && startup_.active());
    status.startup_complete = !steeringAlignmentPending() && enable_probe_after_ns_ == 0 && !traction_mode_pending_ && !startup_.failed() && (virtual_position_source_ || startup_.complete());
    status.startup_failed = startup_.failed();
    status.initialized = steering.initialized() && traction.initialized() &&
        configuration_[binding.steering_drive].ready() && configuration_[binding.traction_drive].ready();
    status.position_valid = position_valid_[binding.traction_drive] && steering.hasFeedback(POSITION_FEEDBACK) &&
        steering.feedbackFreshness().fresh(FeedbackFreshness::POSITION, motion_feedback_timeout_, steady_now) &&
        traction.feedbackFreshness().fresh(FeedbackFreshness::POSITION, motion_feedback_timeout_, steady_now);
    status.position_reason = status.position_valid ? "current controller position feedback" :
        "controller position feedback missing, stale or discontinuous";
    status.reset_id = reset_id_;
    status.reset_acknowledged = reset_id_ != 0;
    for (const auto& name : {binding.steering_drive, binding.traction_drive})
    {
      const auto& drive = drives_.at(name);
      status.reset_acknowledged = status.reset_acknowledged && !drive.hasError() &&
          drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now) &&
          field_stamp_ns_[name][0] >= reset_requested_ns_ &&
          drive.feedbackFreshness().generation(FeedbackFreshness::ERROR_REGISTER) > reset_fault_generation_[name] &&
          field_stamp_ns_[name][1] >= reset_requested_ns_ &&
          drive.feedbackFreshness().fresh(FeedbackFreshness::ERROR_REGISTER, fault_feedback_timeout_, steady_now);
    }
    const auto describe = [&](const std::string& name) {
      const auto& drive = drives_.at(name);
      mobotic_interfaces::msg::DriveStatus item;
      item.name = name;
      item.initialized = drive.initialized() && configuration_[name].ready();
      item.status_known = drive.hasNativeStatus() &&
          drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now);
      item.enabled = item.status_known && drive.enabled();
      item.status_word = 0;
      item.status_word_known = false;
      item.device_state = drive.nativeDeviceState();
      item.device_state_known = item.status_known;
      item.control_mode = toStatusControlMode(drive.currentMode());
      item.native_control_mode = drive.currentMode();
      item.control_mode_known = drive.currentMode() != UNKNOWN &&
          drive.feedbackFreshness().fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, steady_now);
      item.error_register = drive.getError(); item.error_code = drive.activeErrorCode();
      item.fault_known = drive.feedbackFreshness().fresh(FeedbackFreshness::ERROR_REGISTER, fault_feedback_timeout_, steady_now);
      item.motor_current = drive.motorCurrentAmps();
      item.current_known = std::isfinite(item.motor_current) && drive.feedbackFreshness().fresh(FeedbackFreshness::CURRENT, motion_feedback_timeout_, steady_now);
      if (!item.current_known) { item.motor_current = std::numeric_limits<double>::quiet_NaN(); }
      item.temperature = std::numeric_limits<double>::quiet_NaN(); item.temperature_known = false;
      const auto& stamps = field_stamp_ns_[name];
      item.status_stamp = rclcpp::Time(stamps[0]);
      item.fault_stamp = rclcpp::Time(stamps[1]);
      item.current_stamp = rclcpp::Time(stamps[2]);
      return item;
    };
    status.steering = describe(binding.steering_drive);
    status.traction = describe(binding.traction_drive);
    status.enabled = steering.enabled() && traction.enabled();
    status.feedback_fresh = driveFeedbackFresh(steering, steady_now) && driveFeedbackFresh(traction, steady_now);
    const auto steering_problem = feedbackProblem(steering, steady_now);
    const auto traction_problem = feedbackProblem(traction, steady_now);
    status.status_message = !steering_problem.empty() ? binding.steering_drive + ": " + steering_problem :
        !traction_problem.empty() ? binding.traction_drive + ": " + traction_problem : "Feedback fresh";
    const bool steering_status_fresh = status.steering.status_known && status.steering.control_mode_known;
    const bool traction_status_fresh = status.traction.status_known && status.traction.control_mode_known;
    status.enabled = status.enabled && steering_status_fresh && traction_status_fresh;
    status.brake_state_known = false;
    status.brake_released = false;
    status.control_mode = traction_status_fresh ? toStatusControlMode(traction.currentMode()) :
        mobotic_interfaces::msg::WheelModuleStatus::CONTROL_MODE_UNKNOWN;
    status.motor_current = status.traction.motor_current;
    status.status_word = 0;

    const int traction_error = traction.activeErrorCode();
    const int steering_error = steering.activeErrorCode();
    status.error_code = traction_error != 0 ? traction_error : steering_error;
    if (current_overspeed_latched_)
    {
      status.error_code = 0x7FFF0001;
      status.status_message = "Current-mode speed limit exceeded; explicit reset required";
    }
    if (steeringAlignmentPending() && status.error_code == 0)
    { status.status_message = "Steering-to-zero alignment pending; traction held at zero"; }
    array.modules.push_back(status);
  }

  // Release the reset pulse after its serialized SDO exchange, even if the
  // fault register still needs initialization/enable to recover. Waiting for
  // fault clearance here prevents the authorized zero-target recovery sequence.
  if (!reset_release_sent_ && reset_id_ != 0 && std::none_of(sdo_channels_.begin(), sdo_channels_.end(),
        [](const auto& channel) { return channel.second.faultResetPending(); }))
  {
    for (const auto& [name, drive] : drives_)
    {
      (void)name;
      publishCanFrame(stampFrame(drive.clearErrorRequest(NO_OPERATION), "can", this->now()));
    }
    reset_release_sent_ = true;
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
    if (!virtual_position_source_ && is_sdo && (configuration_[name].ready() || configuration_[name].failed()) &&
        !sdo_channels_[node_id].receive(frame)) { continue; }
    if (is_sdo && frame.data[0] == WRITE_REPLY && frame.data[1] == 0x90 && frame.data[2] == 0x37 &&
        frame.data[3] == 0 && steering_alignment_ == SteeringAlignment::MOVING &&
        sample_stamp_ns > steering_alignment_motion_after_ns_)
    { steering_alignment_target_ack_ns_[name] = sample_stamp_ns; }
    // Only this node's SDO responses and boot-up/heartbeat frames belong
    // to the drive. Other CAN traffic cannot refresh its availability.
    const bool is_heartbeat = frame.id == 0x700 + node_id && frame.dlc == 1;
    if (is_sdo || is_heartbeat)
    { last_can_feedback_[name].record(sample_time, acquisition_stamp.nanoseconds()); }

    const auto velocity_generation = drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY);
    const auto position_generation = drive.feedbackFreshness().generation(FeedbackFreshness::POSITION);
    const auto mode_generation = drive.feedbackFreshness().generation(FeedbackFreshness::CONTROL_MODE);
    const std::array<FeedbackFreshness::Field, 3> fields{FeedbackFreshness::STATUS, FeedbackFreshness::ERROR_REGISTER, FeedbackFreshness::CURRENT};
    std::array<uint64_t, 3> generations{};
    for (std::size_t i = 0; i < fields.size(); ++i) { generations[i] = drive.feedbackFreshness().generation(fields[i]); }
    // Failed configuration still permits correlated read-only SDO updates,
    // including their acquisition stamps used by explicit reset admission.
    if (configuration_[name].failed() && !is_sdo) { continue; }
    const bool configuring = !configuration_[name].ready() && !configuration_[name].failed();
    configuration_[name].receive(frame);
    // Configuration writes and readback must complete before motion feedback is admitted.
    if (configuring && !configuration_[name].ready())
    {
      if (!virtual_position_source_ && configuration_[name].failed() && is_sdo)
      { drive.update(frame, sample_time); dispatchSdo(); }
      continue;
    }
    drive.preserveControllerConfiguration((!virtual_position_source_ && !startup_.started()) ||
        configuration_[name].failed() || startup_.failed());
    const auto frames_to_send = drive.update(frame, sample_time);
    if (frame.id == 0x700 + node_id && frame.dlc == 1 && frame.data[0] == 0 && configuration_[name].ready())
    {
      configuration_[name].invalidate();
      boot_reconfigure_pending_ = true;
      position_valid_[name] = false;
      traction_positions_.erase(name);
      motion_sample_stamp_ns_.erase(name);
      if (!virtual_position_source_) { sdo_channels_[node_id].clear(); }
      accepted_traction_targets_.clear();
      sendStopToAllDrives(); disableAllDrives();
      if (!virtual_position_source_) { startup_.fail(); }
    }
    for (std::size_t i = 0; i < fields.size(); ++i)
    { if (generations[i] != drive.feedbackFreshness().generation(fields[i])) { field_stamp_ns_[name][i] = sample_stamp_ns; } }
    if (mode_generation != drive.feedbackFreshness().generation(FeedbackFreshness::CONTROL_MODE))
    { native_mode_stamp_ns_[name] = sample_stamp_ns; }
    if (drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY) != velocity_generation)
    {
      motion_sample_stamp_ns_[name][0] = sample_stamp_ns;
    }
    if (drive.feedbackFreshness().generation(FeedbackFreshness::POSITION) != position_generation)
    {
      motion_sample_stamp_ns_[name][1] = sample_stamp_ns;
      if (drive.isTractionJoint())
      {
        const double ticks_per_second = (max_traction_velocity_ / wheel_radius_) * drive.positionCountsPerOutputRevolution() / (2.0 * M_PI);
        position_valid_[name] = traction_positions_[name].update(static_cast<int32_t>(drive.feedback(POSITION_FEEDBACK)),
            ticks_per_second * motion_feedback_timeout_ + 2.0);
      }
      else { position_valid_[name] = true; }
    }
    for (const auto& frame_to_send : frames_to_send)
    {
      publishCanFrame(stampFrame(frame_to_send, "can", this->now()));
    }
    if (!virtual_position_source_ && is_sdo && !configuring) { dispatchSdo(); }

    const bool drive_feedback_available =
        driveFeedbackFresh(drive, steady_now) && drive.hasFeedback(VELOCITY_FEEDBACK) &&
        drive.hasFeedback(POSITION_FEEDBACK);

    all_feedbacks_available = all_feedbacks_available && drive_feedback_available;

    if (!drive_feedback_available)
    {
      continue;
    }
    oldest_motion_stamp_ns = std::min(oldest_motion_stamp_ns, motion_sample_stamp_ns_.at(name)[0]);
    oldest_motion_stamp_ns = std::min(oldest_motion_stamp_ns, motion_sample_stamp_ns_.at(name)[1]);

    feedback_msg.name.emplace_back(drive.id());
    feedback_msg.velocity.push_back(drive.hasFeedback(VELOCITY_FEEDBACK) ? drive.velocityFeedbackRadPerSec() * (drive.isTractionJoint() ? wheel_radius_ : 1.0) : 0.0);

    if (!drive.isTractionJoint())
    {
      feedback_msg.position.push_back(ticksToAngle(drive.feedback(POSITION_FEEDBACK), drive.resolution()));
    }
    else
    {
      feedback_msg.position.push_back(traction_positions_[name].radians(
          drive.positionCountsPerOutputRevolution(), 1.0));
    }
  }

  int64_t newest_motion_stamp_ns = 0;
  for (const auto& [name, drive] : drives_)
  {
    (void)drive;
    for (const auto stamp : motion_sample_stamp_ns_[name])
    { newest_motion_stamp_ns = std::max(newest_motion_stamp_ns, stamp); }
  }
  const double skew_budget = virtual_position_source_ ? sample_skew_budget_ : physical_sample_skew_budget_;
  // Each SDO has its own acquisition time. Re-publication cannot renew an
  // old sample or turn an incomplete scan into a coherent joint-state set.
  if (all_feedbacks_available && oldest_motion_stamp_ns > published_cycle_ns_)
  {
    last_motion_scan_span_ms_ = static_cast<double>(newest_motion_stamp_ns - oldest_motion_stamp_ns) * 1e-6;
    if (last_motion_scan_span_ms_ > skew_budget * 1000) { ++incoherent_motion_sets_; }
  }
  const bool coherent = oldest_motion_stamp_ns > published_cycle_ns_ &&
      static_cast<double>(newest_motion_stamp_ns - oldest_motion_stamp_ns) * 1e-9 <= skew_budget &&
      std::all_of(position_valid_.begin(), position_valid_.end(), [](const auto& item) { return item.second; });
  if (all_feedbacks_available && coherent && feedback_msg.name.size() == drives_.size())
  {
    const rclcpp::Time feedback_stamp(oldest_motion_stamp_ns, ros_now.get_clock_type());
    feedback_msg.header.stamp = feedback_stamp;
    feedback_msg.header.frame_id = "can";
    feedback_msg.effort.clear();
    published_cycle_ns_ = oldest_motion_stamp_ns;
    pub_joint_states_->publish(feedback_msg);
    publishWheelModuleStatus(ros_now);
  }
  enforceFeedbackFreshness();
}

void MoboticDriver::scheduleFeedbackReads()
{
  const auto now = this->now();
  const auto shouldPoll = [now](std::map<std::string, rclcpp::Time>& attempts,
                                const std::string& name,
                                const double period_seconds) {
    const auto attempt = attempts.find(name);
    if (attempt == attempts.end() || now < attempt->second || (now - attempt->second).seconds() >= period_seconds)
    {
      attempts[name] = now;
      return true;
    }
    return false;
  };
  // Do not overlap acquisition generations when a requested feedback rate
  // exceeds the physical bus budget. Otherwise early nodes refill the motion
  // phase forever and starve status/current polling and ordinary targets.
  // Budget two motion reads, two native reads and one ordinary service
  // slot per drive. The requested poll rate cannot overfill a slower bus.
  const double physical_round_period = std::chrono::duration<double>(
      can_scheduling::minimum_gap * (5 * drives_.size())).count();
  // A complete publication is stamped at its oldest sample. Reserve 5%
  // of the remaining freshness window for reply and ROS romb latency.
  const double publication_period = 0.95 * (motion_feedback_timeout_ - physical_sample_skew_budget_);
  const double motion_period = virtual_position_source_ ? feedback_request_period_ :
      std::max(feedback_request_period_, std::min(physical_round_period, publication_period));
  // Required fault evidence is serviced before another complete scan. Admit
  // every due node, not just the first name in the map. Age is renewed only by
  // the validated decoder; attempts merely bound retries after missing replies.
  const auto steady_now = FeedbackFreshness::Clock::now();
  for (const auto& [name, drive] : drives_)
  {
    if (!configuration_[name].ready() && !configuration_[name].failed()) { continue; }
    if (sdo_channels_[drive.canNodeID() - 0x600].faultReadOutstanding()) { continue; }
    if (drive.feedbackFreshness().age(FeedbackFreshness::ERROR_REGISTER, steady_now) < diagnostic_sdo_period_) { continue; }
    if (!shouldPoll(last_error_poll_, name, 0.1)) { continue; }
    publishCanFrame(stampFrame(drive.readErrorRegisterRequest(), "can", now));
  }
  const bool fault_service_pending = std::any_of(sdo_channels_.begin(), sdo_channels_.end(),
      [](const auto& item) { return item.second.faultReadOutstanding(); });
  const bool motion_due = last_motion_batch_ns_ == 0 || now.nanoseconds() < last_motion_batch_ns_ ||
      now.nanoseconds() - last_motion_batch_ns_ >= motion_period * 1e9;
  const bool start_motion_batch = motion_due && !fault_service_pending && std::none_of(sdo_channels_.begin(), sdo_channels_.end(),
      [](const auto& item) { return item.second.motionReadOutstanding() || item.second.nativeReadOutstanding(); });

  const bool sdo_motion_active = !virtual_position_source_ &&
      std::any_of(drives_.begin(), drives_.end(), [](const auto& item) { return item.second.enabled(); });
  const bool start_native_batch = (!sdo_motion_active || start_motion_batch) &&
      std::none_of(sdo_channels_.begin(), sdo_channels_.end(),
          [](const auto& item) { return item.second.nativeReadOutstanding(); });
  bool motion_admitted = false;
  const bool bounded_background = !virtual_position_source_;
  bool background_busy = bounded_background && std::any_of(sdo_channels_.begin(), sdo_channels_.end(),
      [](const auto& item) { return item.second.backgroundReadOutstanding(); });
  const bool fault_debt = fault_service_pending;
  const double auxiliary_period = bounded_background && sdo_motion_active ?
      std::max(3.0, diagnostic_sdo_period_) : diagnostic_sdo_period_;
  const auto queue_background = [&](const std::string& name, const can_msgs::msg::Frame& frame,
                                    std::map<std::string, rclcpp::Time>& attempts, double period) {
    if (background_busy || !shouldPoll(attempts, name, period)) { return false; }
    publishCanFrame(stampFrame(frame, "can", now));
    if (bounded_background) { background_busy = true; }
    return true;
  };

  const double idle_status_period = std::min(0.1, status_feedback_timeout_ / 3.0);
  const double active_native_period = virtual_position_source_ ? idle_status_period :
      std::max(idle_status_period, std::chrono::duration<double>(
          can_scheduling::minimum_gap * (4 * drives_.size())).count());
  const bool active_native_monitoring = virtual_position_source_ || startup_.active() ||
      enable_all_drives_requested_ || enable_probe_after_ns_ != 0 || !reset_release_sent_ ||
      std::any_of(drives_.begin(), drives_.end(), [](const auto& item) { return item.second.enabled(); }) ||
      (motion_source_gate_.fresh(now.nanoseconds(), supervisor_timeout_) &&
       (authority_.reset_in_progress || authority_.transition_in_progress));
  const auto poll_native = [&](const std::string& name, const MoboticDriveCanCommunication& drive) {
    if (!start_native_batch) { return; }
    if (sdo_motion_active && start_motion_batch)
    {
      // Every round needs both fields even if a recovery read happened
      // recently. Skipping it here would defer it by another complete round.
      publishCanFrame(stampFrame(drive.readDeviceStateRequest(), "can", now));
      publishCanFrame(stampFrame(drive.readDeviceModeRequest(), "can", now));
      last_status_poll_[name] = now; last_mode_poll_[name] = now;
      return;
    }
    // B2 fault-clear admission needs fresh disabled-state proof. Idle mode
    // observations are diagnostic; they cannot authorize drive enable.
    if (shouldPoll(last_status_poll_, name, active_native_monitoring ? active_native_period : idle_status_period))
    { publishCanFrame(stampFrame(drive.readDeviceStateRequest(), "can", now)); }
    if (shouldPoll(last_mode_poll_, name, active_native_monitoring ? active_native_period : 1.0))
    { publishCanFrame(stampFrame(drive.readDeviceModeRequest(), "can", now)); }
  };

  for (auto& [name, drive] : drives_)
  {
    if (configuration_[name].failed())
    {
      // Failed initialization stops configuration/motion, not observation.
      // Failed startup still observes disabled-state/fault proof; it does
      // not require fast mode polling indefinitely without active recovery.
      poll_native(name, drive);
      continue;
    }
    if (!configuration_[name].ready()) { continue; }
    if (!virtual_position_source_)
    {
      // Acquire motion telemetry only after enable.
      if (start_motion_batch && drive.enabled())
      {
        motion_admitted = true;
        for (const auto parameter : {POSITION_FEEDBACK, VELOCITY_FEEDBACK})
        { publishCanFrame(stampFrame(drive.readFeedbackRequest(parameter), "can", now)); }
      }
      // Current is diagnostic, not part of position/velocity coherence or the
      // enable gate. Keep optional telemetry from consuming native-state slots.
      if (!fault_debt && drive.enabled())
      { queue_background(name, drive.readFeedbackRequest(CURRENT_FEEDBACK), last_current_poll_, auxiliary_period); }
    }
    // Status/fault polling continues in every state.
    poll_native(name, drive);
    if (!fault_debt && software_enable_diagnostics_)
    { queue_background(name, drive.readSoftwareEnableRequest(), last_software_enable_poll_, auxiliary_period); }
  }
  if (motion_admitted) { last_motion_batch_ns_ = now.nanoseconds(); }
}

void MoboticDriver::feedbackTimerCallback()
{
  enforceManagementLease();
  advanceStartup();
  enforceFeedbackFreshness();
  const auto now = this->now();
  if (virtual_position_source_ && boot_reconfigure_pending_ && std::none_of(drives_.begin(), drives_.end(), [](const auto& item) { return item.second.enabled(); }))
  {
    initializeAllDrives();
    boot_reconfigure_pending_ = false;
  }
  // Volatile ROS discovery can lose the first virtual SDO exchange. Retry a
  // failed simulated startup while disabled; physical controllers never use
  // this retry path and all initialization/feedback gates remain in force.
  if (virtual_position_source_ && !startup_.failed() && !enable_all_drives_requested_ &&
      std::none_of(drives_.begin(), drives_.end(), [](const auto& item) { return item.second.enabled(); }) &&
      std::any_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.failed(); }))
  {
    initializeAllDrives();
  }
  const auto shouldPoll = [now](std::map<std::string, rclcpp::Time>& attempts,
                                const std::string& name,
                                const double period_seconds) {
    const auto attempt = attempts.find(name);
    if (attempt == attempts.end() || now < attempt->second || (now - attempt->second).seconds() >= period_seconds)
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

  scheduleFeedbackReads();
  for (auto& [name, drive] : drives_)
  {
    if (configuration_[name].failed()) { continue; }
    if (!configuration_[name].ready())
    {
      if (virtual_position_source_)
      {
        const auto frame = configuration_[name].next(SdoConfiguration::Clock::now(), now.nanoseconds());
        if (frame) { pub_can_frames_->publish(stampFrame(*frame, "can", now)); }
      }
      continue;
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
          // Fault retries are allowed only while the supervisor grants enable.
          // A disable/reset transaction must never cause background retries.
          if (management_lease_.permitsEnable(supervisor_timeout_) &&
              (virtual_position_source_ || startup_.active() || enable_all_drives_requested_))
          {
            publishCanFrame(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
          }
        }
        publishCanFrame(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
      }
      continue;
    }

    if (enable_all_drives_requested_ && !drive.enabled())
    {
      // The initial batch verified every drive. Retries must also retain
      // fresh native evidence; a lost reply or changed mode cannot turn a
      // previously admitted enable into an unchecked background retry.
      const auto steady_now = FeedbackFreshness::Clock::now();
      if (!virtual_position_source_ && (!drive.hasNativeStatus() ||
          drive.currentMode() != drive.targetMode() ||
          !drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now) ||
          !drive.feedbackFreshness().fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, steady_now)))
      { continue; }
      if (shouldRetry(last_enable_attempt_, name))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Waiting for drive " << name << " to enable.");

        if (drive.hasError())
        {
          if (management_lease_.permitsEnable(supervisor_timeout_))
          {
            publishCanFrame(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
          }
        }
        if (virtual_position_source_ && drive.currentMode() != drive.targetMode())
        {
          pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now()));
        }
        // Clear old targets before releasing enable/brakes on a retry too.
        const auto idle_input = drive.targetMode() == POSITION ? STEERING_PROFILE_VELOCITY_INPUT : drive.inputId(drive.targetMode());

        publishCanFrame(stampFrame(drive.setTargetRequest(idle_input, 0), "can", this->now()));
        if (drive.targetMode() == POSITION)
        { publishCanFrame(stampFrame(drive.steeringProfileUpdateRequest(), "can", this->now())); }

        for (const auto& frame : drive.changeDeviceStateRequests(ENABLE))
        {
          publishCanFrame(stampFrame(frame, "can", this->now()));
        }

        publishCanFrame(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));

        publishCanFrame(stampFrame(drive.readDeviceModeRequest(), "can", this->now()));
        publishCanFrame(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
      }
      // Motion telemetry is acquired after enable.
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
      // The scheduled status poll above already observes disabled drives.
      // An extra read on every feedback tick doubles idle status traffic
      // without improving the configured status/fault freshness contract.
      if (!virtual_position_source_) { continue; }
    }

    // Physical acquisition is queued once, above, under shared-batch admission.
    // The virtual topic bus polls stationary feedback before enabling as well.
    if (!virtual_position_source_) { continue; }

    publishCanFrame(stampFrame(drive.readFeedbackRequest(CURRENT_FEEDBACK), "can", this->now()));

    publishCanFrame(stampFrame(drive.readFeedbackRequest(VELOCITY_FEEDBACK), "can", this->now()));

    publishCanFrame(stampFrame(drive.readFeedbackRequest(POSITION_FEEDBACK), "can", this->now()));
  }

  // This periodic publication also reports startup, disabled, and faulted
  // states, for which a complete JointState feedback cycle is unavailable.
  publishWheelModuleStatus(now);
}

bool MoboticDriver::jointSetpointsCallback(const sensor_msgs::msg::JointState& msg, MotionSourceGate::Source source)
{
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1'000'000'000u) { return false; }
  const rclcpp::Time command_stamp(msg.header.stamp);
  enforceModeAuthority();
  if (!motion_source_gate_.permits(source, command_stamp.nanoseconds(), this->now().nanoseconds(),
                                  supervisor_timeout_)) { return false; }
  if (command_stamp.nanoseconds() < motion_accept_after_ns_) { return false; }
  if (command_stamp.nanoseconds() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints without a timestamp.");
    return false;
  }

  const double command_age = (this->now() - command_stamp).seconds();
  if (command_age > watchdog_timeout_ || command_age < -0.1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints with invalid age: "
                                                 << command_age << " s.");
    return false;
  }

  if (msg.name.size() != drives_.size())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting joint setpoints containing "
                                                 << msg.name.size() << " joints; expected " << drives_.size() << ".");
    return false;
  }

  std::set<std::string> received_joints;
  for (const auto& name : msg.name)
  {
    if (drives_.find(name) == drives_.end() || !received_joints.insert(name).second)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting unknown or duplicate joint '" << name
                                                                                                          << "'.");
      return false;
    }
  }

  for (const auto& [name, drive] : drives_)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), name);
    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    const DeviceMode mode = effectiveMode(drive);
    if (!drive.initialized() || mode != drive.targetMode()) { sendStopToAllDrives(); return false; }
    const bool valid_position = hasIndex(msg.position, index) && std::isfinite(msg.position[index]);
    const bool valid_velocity = hasIndex(msg.velocity, index) && std::isfinite(msg.velocity[index]);
    const bool valid_effort = hasIndex(msg.effort, index) && std::isfinite(msg.effort[index]);

    if ((mode == POSITION && (!valid_position || !valid_velocity)) ||
        ((mode == VELOCITY || mode == S_VELOCITY) && !valid_velocity) || (mode == CURRENT && !valid_effort))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting incomplete joint setpoint for '" << name
                                                                                                             << "'.");
      return false;
    }
    if (mode == POSITION && !drive.steeringPositionRepresentable(msg.position[index]))
    {
      // Validate every joint before applyJointSetpoints can send any CAN frame.
      RCLCPP_WARN_STREAM(this->get_logger(), "[mobotic_driver] Rejecting entire joint command: steering target for '"
          << name << "' cannot be represented as signed 32-bit encoder ticks.");
      return false;
    }
  }

  return applyJointSetpoints(msg);
}

void MoboticDriver::finalCommandCallback(const mobotic_interfaces::msg::WheelModuleCommandArray& msg)
{
  if (steeringAlignmentPending() || traction_mode_pending_ || (!virtual_position_source_ && !startup_.complete())) { return; }
  using Command = mobotic_interfaces::msg::WheelModuleCommandArray;
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1'000'000'000u ||
      msg.source_stamp.sec < 0 || msg.source_stamp.nanosec >= 1'000'000'000u) { return; }
  const auto now_ns = this->now().nanoseconds();
  const auto stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  const auto source_ns = rclcpp::Time(msg.source_stamp).nanoseconds();
  if (msg.supervisor_instance != authority_.supervisor_instance ||
      msg.authority_generation != authority_.authority_generation || msg.selected_mode != authority_.selected_mode ||
      stamp_ns <= last_final_stamp_ns_ || stamp_ns < motion_accept_after_ns_ || msg.modules.size() != wheel_modules_.size() ||
      msg.purpose > Command::HOLD || !motion_source_gate_.fresh(now_ns, supervisor_timeout_)) { return; }
  if (msg.purpose == Command::MOTION && (!authority_.motion_permitted || authority_.transition_in_progress ||
      authority_.error_active || authority_.safety_stop_active || authority_.reset_in_progress || source_ns <= 0 ||
      now_ns - source_ns < 0 || now_ns - source_ns > 250000000LL)) { return; }
  if (msg.purpose == Command::CONTROLLED_STOP && (!authority_.transition_in_progress ||
      authority_.error_active || authority_.safety_stop_active || authority_.reset_in_progress)) { return; }
  sensor_msgs::msg::JointState command;
  command.header = msg.header;
  const bool current_control = msg.selected_mode == mobotic_interfaces::msg::VehicleModeState::KINEMATICOFF;
  if (msg.purpose == Command::MOTION && (current_overspeed_latched_ ||
      std::any_of(wheel_modules_.begin(), wheel_modules_.end(), [this, current_control](const auto& item) {
        const auto& drive = drives_.at(item.second.traction_drive);
        return drive.targetMode() != (current_control ? CURRENT : VELOCITY) ||
               drive.currentMode() != drive.targetMode();
      }))) { return; }
  std::set<std::string> names;
  for (const auto& module : msg.modules)
  {
    const auto found = wheel_modules_.find(module.name);
    if (found == wheel_modules_.end() || !names.insert(module.name).second ||
        !std::isfinite(module.steering_position) || !std::isfinite(module.steering_profile_velocity) ||
        !std::isfinite(module.traction_velocity) || !std::isfinite(module.traction_current) ||
        module.traction_current_control != current_control || module.steering_profile_velocity < 0 ||
        module.steering_profile_velocity > max_steering_velocity_ || std::abs(module.traction_velocity) > max_traction_velocity_ ||
        std::abs(module.traction_current) > max_traction_current_ ||
        (current_control ? module.traction_velocity != 0.0 : module.traction_current != 0.0)) { return; }
    if (msg.purpose != Command::MOTION && (module.steering_profile_velocity != 0 ||
        (msg.purpose == Command::HOLD && module.traction_velocity != 0) || module.traction_current != 0)) { return; }
    if (msg.purpose == Command::CONTROLLED_STOP)
    {
      const double previous = accepted_traction_targets_[module.name];
      if (module.traction_velocity * previous < 0 || std::abs(module.traction_velocity) > std::abs(previous) + 1e-9) { return; }
    }
    command.name.push_back(found->second.steering_drive);
    double steering_position = module.steering_position;
    if (msg.purpose != Command::MOTION)
    {
      // The driver chooses its own actual steering hold; a HOLD/STOP record
      // cannot introduce a different steering target through trusted metadata.
      const auto& steering = drives_.at(found->second.steering_drive);
      if (!steering.hasFeedback(POSITION_FEEDBACK) || !steering.feedbackFreshness().fresh(
            FeedbackFreshness::POSITION, motion_feedback_timeout_))
      {
        sendStopToAllDrives();
        // Disabled controllers may not provide position feedback until the
        // explicit zero-target reset enable completes. An inhibited HOLD must
        // not cancel that authorized recovery on every command callback.
        // No steering position is applied without fresh actual feedback.
        const bool zero_target_recovery = msg.purpose == Command::HOLD &&
            authority_.reset_in_progress && !authority_.safety_stop_active && reset_id_ != 0 &&
            authority_.reset_id == reset_id_ && reset_release_sent_ &&
            management_lease_.permitsEnable(supervisor_timeout_);
        if (!zero_target_recovery) { disableAllDrives(); }
        return;
      }
      steering_position = ticksToAngle(steering.feedback(POSITION_FEEDBACK), steering.resolution());
    }
    command.position.push_back(steering_position);
    command.velocity.push_back(module.steering_profile_velocity);
    command.effort.push_back(0.0);
    command.name.push_back(found->second.traction_drive);
    command.position.push_back(0.0);
    command.velocity.push_back(module.traction_velocity);
    command.effort.push_back(module.traction_current);
  }
  if (jointSetpointsCallback(command, msg.selected_mode == 2 ? MotionSourceGate::Source::SupervisorDirect : MotionSourceGate::Source::Kinematics))
  {
    last_final_stamp_ns_ = stamp_ns;
    for (const auto& module : msg.modules)
    { accepted_traction_targets_[module.name] = current_control ? module.traction_current : module.traction_velocity; }
  }
}

void MoboticDriver::wheelModuleCommandCallback(const mobotic_interfaces::msg::WheelModuleOperationCommand& msg)
{
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1'000'000'000u ||
      msg.supervisor_instance != authority_.supervisor_instance || msg.authority_generation != authority_.authority_generation ||
      !motion_source_gate_.fresh(this->now().nanoseconds(), supervisor_timeout_)) { return; }
  const bool reset_authorized = msg.reset_id != 0 && authority_.reset_in_progress &&
      msg.reset_id == authority_.reset_id && !authority_.safety_stop_active;
  const bool reset_reenable = reset_authorized && msg.reset_id == reset_id_ && reset_release_sent_ &&
      reset_requested_ns_ >= mode_transaction_boundary_ns_;
  if (msg.reset_id != 0 && !reset_authorized) { return; }
  if (msg.enable && steering_align_on_enable_ && msg.reset_id != 0 &&
      msg.reset_id <= steering_alignment_failed_reset_id_)
  { sendStopToAllDrives(); disableAllDrives(); return; }
  if (msg.enable && (authority_.safety_stop_active ||
      ((authority_.error_active || authority_.reset_in_progress) && !reset_reenable))) { return; }
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

  const auto goal = authority_.transition_in_progress ? authority_.requested_mode : authority_.selected_mode;
  if (msg.enable && (goal > 2 || msg.traction_current_control != (goal == 2)))
  {
    // Authority and management use separate DDS topics. Discard a command
    // from the other side of a transition without renewing its lease or
    // interrupting the still-authorized outgoing controlled stop.
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

  if (msg.reset_id != 0 && msg.reset_id > reset_id_ && !msg.enable)
  {
    // Missing motion telemetry is not standstill. A separately authorized
    // reset may clear faults with every configured drive freshly disabled.
    if (!reset_authorized || (!virtual_position_source_ && startup_.active()) ||
        (startup_.started() && !startup_.failed() && !std::all_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.ready(); })))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "B2 recovery waiting: previous drive initialization has not stopped; wait for disabled drive status and press B2 again.");
      sendStopToAllDrives(); disableAllDrives(); return;
    }
    const auto steady_now = FeedbackFreshness::Clock::now();
    std::vector<ResetDriveObservation> observations;
    for (const auto& [name, drive] : drives_)
    {
      (void)name;
      observations.push_back({drive.feedbackFreshness().motionFresh(motion_feedback_timeout_, steady_now),
          drive.feedbackFreshness().fresh(FeedbackFreshness::VELOCITY, motion_feedback_timeout_, steady_now),
          drive.velocityFeedbackRadPerSec(),
          drive.hasNativeStatus() && drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now), drive.enabled()});
    }
    if (!mayClearDriveFaults(observations))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "B2 recovery waiting: drive movement or missing disabled-state feedback; check drive power/CAN and wait for fresh status.");
      sendStopToAllDrives(); disableAllDrives(); return;
    }
    reset_id_ = msg.reset_id;
    current_overspeed_latched_ = false;
    controller_control_started_ = true;
    reset_requested_ns_ = this->now().nanoseconds();
    reset_release_sent_ = false;
    for (const auto& [name, drive] : drives_)
    {
      reset_fault_generation_[name] = drive.feedbackFreshness().generation(FeedbackFreshness::ERROR_REGISTER);
    }
    sendStopToAllDrives(); disableAllDrives();
    motion_accept_after_ns_ = reset_requested_ns_;
    motion_command_received_ = false;
    accepted_traction_targets_.clear();
    clearErrorsOnAllDrives();
  }

  if (current_overspeed_latched_)
  {
    sendStopToAllDrives();
    if (enable_all_drives_requested_ || std::any_of(drives_.begin(), drives_.end(),
        [](const auto& item) { return item.second.enabled(); })) { disableAllDrives(); }
    return;
  }
  if (msg.enable && reset_reenable && startup_.failed() && startup_.rearmAfterReset(msg.reset_id))
  {
    // An aborted mode transaction can leave the native target different from
    // the still-selected vehicle mode. Explicit disabled-state recovery may
    // rebuild the selected mode; an ordinary enable can never do this.
    traction_mode_initialized_ = false;
  }
  const DeviceMode desired = msg.traction_current_control ? CURRENT : VELOCITY;
  const bool change = std::any_of(wheel_modules_.begin(), wheel_modules_.end(), [this, desired](const auto& item) {
      return drives_.at(item.second.traction_drive).targetMode() != desired;
  });
  if (msg.enable && change)
  {
    if (traction_mode_initialized_)
    {
      if (authority_.transition_in_progress && !msg.reconfigure_traction_mode)
      {
        // The outgoing source still owns its controlled stop. A desired new
        // native mode alone must not replace that ramp with an immediate zero.
        module_management_received_ = true;
        return;
      }
      if (!msg.reconfigure_traction_mode || !authority_.transition_in_progress ||
          (!virtual_position_source_ && !startup_.complete()) || traction_mode_pending_ ||
          !allDriveFeedbackFresh(FeedbackFreshness::Clock::now()) ||
          std::any_of(drives_.begin(), drives_.end(), [](const auto& item) {
            return item.second.hasError() || std::abs(item.second.velocityFeedbackRadPerSec()) > 0.1;
          }))
      { sendStopToAllDrives(); return; }
      mode_transaction_boundary_ns_ = this->now().nanoseconds();
      sendStopToAllDrives(); disableAllDrives();
      if (!virtual_position_source_ && !startup_.prepareModeChange()) { return; }
    }
    for (const auto& [name, binding] : wheel_modules_)
    { (void)name; drives_.at(binding.traction_drive).setTractionOperatingMode(desired); }
    accepted_traction_targets_.clear();
    motion_command_received_ = false;
    last_final_stamp_ns_ = 0;
    motion_accept_after_ns_ = this->now().nanoseconds();
    traction_mode_pending_ = true;
    if (virtual_position_source_)
    {
      sendStopToAllDrives(); disableAllDrives();
      for (auto& [name, drive] : drives_) { (void)name; drive.prepareOperatingModeTransaction(); }
      initializeAllDrives();
    }
  }
  if (msg.enable) { traction_mode_initialized_ = true; }
  if (msg.enable && !virtual_position_source_ && !startup_.started()) { initializeAllDrives(); }
  if (msg.enable && !startup_.failed() && std::all_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.ready(); }))
  {
    // The supervisor republishes the desired state every control cycle.
    // Preserve active motion targets and the existing enable retry sequence.
    if (!enable_all_drives_requested_)
    {
      enableAllDrives();
    }
  }
  else if (!msg.enable && (!module_management_received_ || enable_all_drives_requested_ ||
           std::any_of(drives_.begin(), drives_.end(),
                       [](const auto& entry) { return entry.second.enabled(); })))
  {
    sendStopToAllDrives();
    disableAllDrives();
  }
  if (!msg.enable && !virtual_position_source_ && startup_.active()) { failStartup(); }
  module_management_received_ = true;
}

void MoboticDriver::clearErrorsOnAllDrives()
{
  RCLCPP_INFO(this->get_logger(), "[driver] Clearing errors on all drives.");
  for (auto& [name, drive] : drives_)
  {
    (void)name;
    publishCanFrame(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
  }
}

bool MoboticDriver::applyJointSetpoints(const sensor_msgs::msg::JointState& msg)
{
  enforceManagementLease();
  enforceFeedbackFreshness();
  if (feedback_motion_blocked_ || rclcpp::Time(msg.header.stamp).nanoseconds() < motion_accept_after_ns_) { return false; }
  if (!management_lease_.permitsEnable(supervisor_timeout_) || !enable_all_drives_requested_) { return false; }
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
    const ParameterIds input_id = drive.inputId(mode);

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

      const auto profile_velocity = drive.getTarget(STEERING_PROFILE_VELOCITY_INPUT, msg.velocity.at(idx));
      publishCanFrame(stampFrame(drive.setTargetRequest(STEERING_PROFILE_VELOCITY_INPUT, profile_velocity),
                                "can", this->now()));
      // Position moves latch profile settings. Zero must be explicitly applied
      // with DEV_Cmd Update; a profile write alone leaves an active move running.
      if (profile_velocity != 0)
      {
        publishCanFrame(stampFrame(drive.setTargetRequest(ABSOLUTE_POSITION_INPUT,
            drive.getTarget(ABSOLUTE_POSITION_INPUT, msg.position.at(idx))), "can", this->now()));
      }
      else
      {
        publishCanFrame(stampFrame(drive.steeringProfileUpdateRequest(), "can", this->now()));
      }
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

      publishCanFrame(stampFrame(drive.setTargetRequest(CURRENT_INPUT,
                                                                 drive.getTarget(CURRENT_INPUT, msg.effort.at(idx))),
                                          "can", this->now()));
      command_sent = true;
      continue;
    }

    // Traction ROS commands are contact speed (m/s). Convert once at the CAN boundary.
    if (!hasIndex(msg.velocity, idx))
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "Velocity-mode drive '" << drive.id() << "' has no velocity setpoint.");
      continue;
    }

    const int target = drive.getTarget(input_id, msg.velocity.at(idx) / (drive.isTractionJoint() ? wheel_radius_ : 1.0));

    publishCanFrame(stampFrame(drive.setTargetRequest(input_id, target), "can", this->now()));

    command_sent = true;
  }

  if (command_sent)
  {
    motion_command_received_ = true;
    last_motion_command_received_ = ManagementLease::Clock::now();
    watchdog_timer_setpoints_->reset();
  }
  return command_sent;
}

void MoboticDriver::sendStopToAllDrives()
{
  if (!virtual_position_source_ && !controller_control_started_) { return; }
  // Initialization has its own serialized disable/zero commands. Do not
  // interleave ordinary watchdog SDOs with configuration transactions.
  if (!virtual_position_source_ && startup_.active() && std::any_of(configuration_.begin(), configuration_.end(),
      [](const auto& item) { return !item.second.ready(); })) { return; }
  // Only the supervised reset's bounded steering-to-zero phase may retain
  // steering targets here. Traction is stopped throughout. Any lease, safety,
  // authority or feedback loss restores the ordinary stop/disable path.
  const bool retain_alignment = steering_alignment_ == SteeringAlignment::MOVING && steeringAlignmentPermitted();
  for (auto& [node, channel] : sdo_channels_)
  {
    if (!retain_alignment || drives_.at(sdo_node_names_.at(node)).isTractionJoint())
    { channel.cancelMotionTargets(); }
  }
  for (auto& [name, drive] : drives_)
  {
    (void)name;
    if (!drive.enabled())
    {
      continue;
    }

    const DeviceMode mode = effectiveMode(drive);
    const ParameterIds input_id = drive.inputId(mode);
    const ParameterIds stop_input_id = input_id == ABSOLUTE_POSITION_INPUT ? STEERING_PROFILE_VELOCITY_INPUT : input_id;
    if (retain_alignment && !drive.isTractionJoint()) { continue; }

    publishCanFrame(stampFrame(drive.setTargetRequest(stop_input_id, 0), "can", this->now()));
    if (input_id == ABSOLUTE_POSITION_INPUT)
    { publishCanFrame(stampFrame(drive.steeringProfileUpdateRequest(), "can", this->now())); }
  }
}

void MoboticDriver::disableAllDrives(bool force)
{
  if (steeringAlignmentPending()) { steering_alignment_failed_reset_id_ = reset_id_; }
  steering_alignment_ = SteeringAlignment::IDLE;
  steering_alignment_targets_.clear();
  enable_probe_after_ns_ = 0;
  if (!virtual_position_source_ && !controller_control_started_) { return; }
  // Every caller revokes queued moving/enabling writes immediately. Keep an
  // existing stop/disable transaction queued instead of deleting and rebuilding
  // it on each authority, watchdog and management callback.
  for (auto& [node, channel] : sdo_channels_) { (void)node; channel.cancelWrites(true, true); }
  enable_all_drives_requested_ = false;
  last_enable_attempt_.clear();
  const auto steady_now = ManagementLease::Clock::now();
  if (!virtual_position_source_ && !force && last_disable_request_ &&
      steady_now - *last_disable_request_ < std::chrono::milliseconds(100)) { return; }
  last_disable_request_ = steady_now;
  RCLCPP_INFO(this->get_logger(), "[driver] Disabling all drives.");

  for (auto& [name, drive] : drives_)
  {
    (void)name;
    if (drive.targetMode() != POSITION)
    {
      const ParameterIds stop_input_id = drive.inputId(effectiveMode(drive));

      publishCanFrame(stampFrame(drive.setTargetRequest(stop_input_id, 0), "can", this->now()));
    }

    for (const auto& frame : drive.changeDeviceStateRequests(DISABLE))
    {
      publishCanFrame(stampFrame(frame, "can", this->now()));
    }

    publishCanFrame(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
  }
}

void MoboticDriver::enableAllDrives()
{
  if (!management_lease_.permitsEnable(supervisor_timeout_) ||
      !std::all_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.ready(); })) { return; }
  if (virtual_position_source_ && (!allDriveFeedbackFresh(FeedbackFreshness::Clock::now()) ||
      std::any_of(drives_.begin(), drives_.end(), [](const auto& item) {
        return !item.second.initialized() || item.second.hasError();
      }) || std::any_of(position_valid_.begin(), position_valid_.end(), [](const auto& item) { return !item.second; }))) { return; }
  if (!virtual_position_source_ && !prepareEnableFeedback()) { return; }
  controller_control_started_ = true;
  // A newly admitted enable sequence starts a new transaction: its first
  // subsequent stop must never inherit a previous disable's retry interval.
  last_disable_request_.reset();
  motion_accept_after_ns_ = this->now().nanoseconds();
  motion_command_received_ = false;
  RCLCPP_INFO(this->get_logger(), "[driver] Enabling all drives.");
  if (steering_align_on_enable_ && !virtual_position_source_ && authority_.reset_in_progress)
  {
    steering_alignment_ = SteeringAlignment::WAITING;
    steering_alignment_after_ns_ = this->now().nanoseconds();
    steering_alignment_motion_after_ns_ = 0;
    steering_alignment_target_ack_ns_.clear();
    steering_alignment_targets_.clear();
    steering_alignment_started_ = ManagementLease::Clock::now();
    last_steering_alignment_command_ = ManagementLease::Clock::time_point{};
    steering_alignment_generation_ = authority_.authority_generation;
    steering_alignment_supervisor_ = authority_.supervisor_instance;
  }
  initialize_all_drives_requested_ = true;
  enable_all_drives_requested_ = true;
  last_initialize_attempt_.clear();
  last_enable_attempt_.clear();

  // Clear on entry, then retry reported faults at the existing
  // 200 ms initialization/enable cadence while supervisor enable is granted.
  if (!virtual_position_source_)
  {
    clearErrorsOnAllDrives();
  }

  for (auto& [name, drive] : drives_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Sending enable sequence to drive " << name << ".");
    // An authorized enable attempt sends the
    // zero-target sequence after fault reset, including with a reported fault.
    // Healthy feedback still gates startup completion and every motion command.
    if (virtual_position_source_)
    { pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now())); }
    const auto idle_input = drive.targetMode() == POSITION ? STEERING_PROFILE_VELOCITY_INPUT : drive.inputId(drive.targetMode());

    publishCanFrame(stampFrame(drive.setTargetRequest(idle_input, 0), "can", this->now()));
    if (drive.targetMode() == POSITION)
    { publishCanFrame(stampFrame(drive.steeringProfileUpdateRequest(), "can", this->now())); }

    for (const auto& frame : drive.changeDeviceStateRequests(ENABLE))
    {
      publishCanFrame(stampFrame(frame, "can", this->now()));
    }

    publishCanFrame(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));

    publishCanFrame(stampFrame(drive.readDeviceModeRequest(), "can", this->now()));
    publishCanFrame(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
  }
}

void MoboticDriver::initializeAllDrives()
{
  if (!virtual_position_source_ && (!management_lease_.permitsEnable(supervisor_timeout_) || !startup_.begin())) { return; }
  if (virtual_position_source_ && (enable_all_drives_requested_ || std::any_of(drives_.begin(), drives_.end(), [](const auto& d) { return d.second.enabled(); })))
  { return; }
  last_disable_request_.reset();
  enable_probe_after_ns_ = 0;
  RCLCPP_INFO(this->get_logger(), "[driver] Initializing all drives.");
  initialize_all_drives_requested_ = true;
  if (!virtual_position_source_) { controller_control_started_ = true; }
  last_initialize_attempt_.clear();

  for (auto& [name, drive] : drives_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing drive " << name << ".");
    // Initialize only native mode and state; preserve installed controller settings.
    std::vector<can_msgs::msg::Frame> frames;

    frames.push_back(drive.setDeviceModeRequest(drive.targetMode()));

    if (!virtual_position_source_)
    {
      // Stop/disable, reset the fault and select the native operating mode.
      // Never zero the encoder or save controller settings.
      const auto idle = drive.targetMode() == POSITION ? STEERING_PROFILE_VELOCITY_INPUT : drive.inputId(drive.targetMode());
      frames.insert(frames.begin(), {drive.changeDeviceStateRequest(DISABLE),
          drive.setTargetRequest(idle, 0), drive.clearErrorRequest(CLEAR), drive.clearErrorRequest(NO_OPERATION)});
      sdo_channels_[drive.rawCanNodeID()].clear();
      drive.prepareOperatingModeTransaction();
      position_valid_[name] = false;
      motion_sample_stamp_ns_.erase(name);
    }
    configuration_[name].start(std::move(frames));
    // Obtain the existing controller's value immediately. Never access or reset
    // the encoder directly; subsequent SDO polls supply cyclic updates.
    if (virtual_position_source_)
    { pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(POSITION_FEEDBACK), "can", this->now())); }
  }
}

void MoboticDriver::publishCanFrame(const can_msgs::msg::Frame& frame)
{
  if (!virtual_position_source_ && frame.id >= 0x601 && frame.id <= 0x67F)
  {
    const unsigned node = frame.id - 0x600;
    if (frame.data[0] != READ_FROM_DEVICE)
    {
      const auto drive = std::find_if(drives_.begin(), drives_.end(), [node](const auto& item) { return item.second.rawCanNodeID() == node; });
      const unsigned index = static_cast<unsigned>(frame.data[1]) | (static_cast<unsigned>(frame.data[2]) << 8);
      const bool permitted = drive != drives_.end() && (index == MICONTROL_DEVICE_STATE || index == MICONTROL_CLEAR_ERROR_STATUS ||
             index == STEERING_PROFILE_VELOCITY_INPUT || index == S_VELOCITY_INPUT ||
             index == CURRENT_INPUT || index == ABSOLUTE_POSITION_INPUT);
      if (!controller_control_started_ || !permitted || frame.data[3] != 0)
      { RCLCPP_ERROR(this->get_logger(), "Refusing physical controller configuration write to 0x%04X", index); return; }
    }
    sdo_channels_[node].push(frame, SdoChannel::Clock::now(), watchdog_timeout_);
    // Admission happens after this callback has finished queuing/cancelling
    // every affected node, so lease revocation cannot dispatch an old enable
    // from another drive partway through the stop transaction.
    return;
  }
  pub_can_frames_->publish(frame);
}

void MoboticDriver::advanceStartup()
{
  advanceSteeringAlignment();
  if (virtual_position_source_)
  {
    if (traction_mode_pending_ && std::all_of(configuration_.begin(), configuration_.end(),
          [](const auto& item) { return item.second.ready(); }) && allDriveFeedbackFresh(FeedbackFreshness::Clock::now()) &&
        std::all_of(drives_.begin(), drives_.end(), [](const auto& item) {
          return item.second.enabled() && item.second.initialized() && !item.second.hasError() &&
                 item.second.currentMode() == item.second.targetMode();
        })) { traction_mode_pending_ = false; }
    return;
  }
  if (!startup_.active()) { return; }
  if (!management_lease_.permitsEnable(supervisor_timeout_) || startup_.expired(startup_timeout_) ||
      std::any_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.failed(); }))
  { failStartup(); return; }
  if (!std::all_of(configuration_.begin(), configuration_.end(), [](const auto& item) { return item.second.ready(); })) { return; }
  if (!enable_all_drives_requested_) { enableAllDrives(); }
  if (!steeringAlignmentPending() && published_cycle_ns_ > 0 && allDriveFeedbackFresh(FeedbackFreshness::Clock::now()) &&
      std::all_of(drives_.begin(), drives_.end(), [](const auto& item) {
        return item.second.enabled() && item.second.initialized() && !item.second.hasError();
      }) && std::all_of(position_valid_.begin(), position_valid_.end(), [](const auto& item) { return item.second; }))
  { startup_.finish(); traction_mode_pending_ = false;
    RCLCPP_INFO(this->get_logger(), "Drive startup complete: fresh enabled-drive feedback acquired."); }
}

bool MoboticDriver::steeringAlignmentPending() const
{
  return steering_alignment_ == SteeringAlignment::WAITING || steering_alignment_ == SteeringAlignment::MOVING;
}

bool MoboticDriver::steeringAlignmentPermitted() const
{
  const auto now_ns = this->get_clock()->now().nanoseconds();
  if (!steeringAlignmentPending() || !management_lease_.permitsEnable(supervisor_timeout_) ||
      !motion_source_gate_.fresh(now_ns, supervisor_timeout_) || !authority_.reset_in_progress ||
      authority_.reset_id == 0 || authority_.reset_id != reset_id_ || !reset_release_sent_ ||
      authority_.safety_stop_active || authority_.transition_in_progress ||
      authority_.authority_generation != steering_alignment_generation_ ||
      authority_.supervisor_instance != steering_alignment_supervisor_ ||
      !allDriveFeedbackFresh(FeedbackFreshness::Clock::now())) { return false; }
  return std::all_of(drives_.begin(), drives_.end(), [](const auto& item) {
    const auto& drive = item.second;
    return drive.enabled() && drive.initialized() && !drive.hasError() &&
        (!drive.isTractionJoint() || std::abs(drive.velocityFeedbackRadPerSec()) <= 0.1);
  });
}

void MoboticDriver::advanceSteeringAlignment()
{
  if (!steeringAlignmentPending()) { return; }
  const auto now = this->now();
  const auto steady = ManagementLease::Clock::now();
  const bool context_valid = management_lease_.permitsEnable(supervisor_timeout_) &&
      motion_source_gate_.fresh(now.nanoseconds(), supervisor_timeout_) && authority_.reset_in_progress &&
      authority_.reset_id != 0 && authority_.reset_id == reset_id_ && !authority_.safety_stop_active &&
      !authority_.transition_in_progress && authority_.authority_generation == steering_alignment_generation_ &&
      authority_.supervisor_instance == steering_alignment_supervisor_;
  if (!context_valid || std::chrono::duration<double>(steady - steering_alignment_started_).count() >= steering_alignment_timeout_ ||
      (steering_alignment_ == SteeringAlignment::MOVING && !steeringAlignmentPermitted()))
  {
    RCLCPP_ERROR(this->get_logger(), "Steering alignment interrupted, feedback lost or timed out; drives disabled. Keep controls neutral and press B2 to retry after correcting the cause.");
    failStartup(true);
    return;
  }
  if (!steeringAlignmentPermitted()) { return; }
  bool fresh_after_enable = true;
  for (const auto& [name, drive] : drives_)
  {
    if (drive.isTractionJoint()) { continue; }
    for (const auto stamp : motion_sample_stamp_ns_[name])
    { fresh_after_enable = fresh_after_enable && stamp > steering_alignment_after_ns_; }
  }
  if (!fresh_after_enable) { return; }
  const bool beginning = steering_alignment_ == SteeringAlignment::WAITING;
  if (beginning)
  {
    // Validate the complete batch before admitting any alignment target. Never
    // recompute while moving: feedback near a half-turn must not flip the goal.
    std::map<std::string, int32_t> targets;
    for (const auto& [name, drive] : drives_)
    {
      if (drive.isTractionJoint()) { continue; }
      const auto target = nearestSteeringHome(drive.feedback(POSITION_FEEDBACK), drive.resolution());
      if (!target)
      {
        RCLCPP_ERROR(this->get_logger(), "Nearest steering home is not representable for %s; drives disabled.", name.c_str());
        failStartup(true);
        return;
      }
      targets.emplace(name, *target);
    }
    steering_alignment_targets_ = std::move(targets);
    steering_alignment_ = SteeringAlignment::MOVING;
    steering_alignment_motion_after_ns_ = now.nanoseconds();
    RCLCPP_INFO(this->get_logger(), "Aligning all steering units to nearest full-turn home; traction held at zero, joystick motion blocked.");
    for (const auto& [name, target] : steering_alignment_targets_)
    {
      const auto& drive = drives_.at(name);
      RCLCPP_INFO(this->get_logger(), "Steering home %s: actual=%.0f target=%d delta=%.0f counts_per_turn=%.0f",
          name.c_str(), drive.feedback(POSITION_FEEDBACK), target,
          target - drive.feedback(POSITION_FEEDBACK), drive.resolution());
    }
  }
  bool fresh_after_command = true, at_home = true;
  for (const auto& [name, drive] : drives_)
  {
    if (drive.isTractionJoint()) { continue; }
    const auto acknowledged = steering_alignment_target_ack_ns_.find(name);
    fresh_after_command = fresh_after_command && acknowledged != steering_alignment_target_ack_ns_.end();
    for (const auto stamp : motion_sample_stamp_ns_[name])
    {
      fresh_after_command = fresh_after_command && acknowledged != steering_alignment_target_ack_ns_.end() &&
          stamp > acknowledged->second;
    }
    at_home = at_home && std::abs(drive.feedback(POSITION_FEEDBACK) - steering_alignment_targets_.at(name)) <= 1 &&
        std::abs(drive.velocityFeedbackRadPerSec()) <= 0.05;
  }
  if (at_home && fresh_after_command)
  {
    steering_alignment_ = SteeringAlignment::COMPLETE;
    sendStopToAllDrives();
    RCLCPP_INFO(this->get_logger(), "Steering alignment complete: all steering positions within 1 controller unit of nearest home with fresh standstill feedback.");
    return;
  }
  if (std::chrono::duration<double>(steady - last_steering_alignment_command_).count() < 0.1) { return; }
  last_steering_alignment_command_ = steady;
  for (const auto& [name, drive] : drives_)
  {
    if (drive.isTractionJoint()) { continue; }
    const auto target = steering_alignment_targets_.at(name);
    // Even an already-home drive needs an acknowledged target, replacing any
    // retained older target before recovery can complete.
    const bool moving = beginning || steering_alignment_target_ack_ns_.count(name) == 0 ||
        std::abs(drive.feedback(POSITION_FEEDBACK) - target) > 1 || std::abs(drive.velocityFeedbackRadPerSec()) > 0.05;
    publishCanFrame(stampFrame(drive.setTargetRequest(STEERING_PROFILE_VELOCITY_INPUT,
        moving ? drive.getTarget(STEERING_PROFILE_VELOCITY_INPUT, steering_alignment_velocity_) : 0), "can", now));
    if (moving) { publishCanFrame(stampFrame(drive.setTargetRequest(ABSOLUTE_POSITION_INPUT, target), "can", now)); }
  }
}

bool MoboticDriver::prepareEnableFeedback()
{
  const auto now = this->now();
  const bool beginning = enable_probe_after_ns_ == 0 || now.nanoseconds() < enable_probe_after_ns_;
  if (beginning)
  {
    enable_probe_after_ns_ = now.nanoseconds();
    RCLCPP_INFO(this->get_logger(), "[driver] Refreshing native state/mode before enable.");
  }
  const auto steady_now = FeedbackFreshness::Clock::now();
  const double period = std::min(0.1, status_feedback_timeout_ / 3.0);
  const auto request = [&](const std::string& name, const can_msgs::msg::Frame& frame,
                           std::map<std::string, rclcpp::Time>& attempts) {
    const auto previous = attempts.find(name);
    if (beginning || previous == attempts.end() || now < previous->second ||
        (now - previous->second).seconds() >= period)
    { publishCanFrame(stampFrame(frame, "can", now)); attempts[name] = now; }
  };
  bool ready = true;
  for (const auto& [name, drive] : drives_)
  {
    const bool status = drive.hasNativeStatus() &&
        drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now) &&
        field_stamp_ns_[name][0] >= enable_probe_after_ns_;
    const bool mode = drive.currentMode() != UNKNOWN && drive.currentMode() == drive.targetMode() &&
        drive.feedbackFreshness().fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, steady_now) &&
        native_mode_stamp_ns_[name] >= enable_probe_after_ns_;
    if (!status || beginning) { request(name, drive.readDeviceStateRequest(), last_status_poll_); }
    if (!mode || beginning) { request(name, drive.readDeviceModeRequest(), last_mode_poll_); }
    ready = ready && status && mode;
  }
  if (!ready) { return false; }
  enable_probe_after_ns_ = 0;
  RCLCPP_INFO(this->get_logger(), "[driver] Fresh native state/mode verified for all drives.");
  return true;
}

void MoboticDriver::failStartup(bool abort_completed_mode)
{
  if (!startup_.active())
  {
    if (!abort_completed_mode || startup_.failed()) { return; }
    if (!startup_.started()) { startup_.begin(); }
  }
  startup_.fail();
  traction_mode_pending_ = false;
  for (auto& [name, setup] : configuration_) { (void)name; setup.invalidate(); }
  for (auto& [node, channel] : sdo_channels_) { (void)node; channel.clear(); }
  sendStopToAllDrives(); disableAllDrives(true);
  // Disable through native SDO even if initialization failed.
  for (const auto& [name, drive] : drives_)
  { (void)name; publishCanFrame(stampFrame(drive.changeDeviceStateRequest(DISABLE), "can", this->now())); }
  RCLCPP_WARN(this->get_logger(), "Drive startup interrupted or failed; drives disabled. Check drive power/CAN and the supervisor's reported blocker. The driver remains running; correct the problem, return joystick to neutral and press B2 to retry.");
}

void MoboticDriver::dispatchSdo()
{
  if (virtual_position_source_) { return; }
  // Revocation cancels queued motion/configuration before bus admission.
  // This does not wait or sleep in a CAN callback or a safety callback.
  enforceManagementLease();
  // Fill completed acquisition slots at bus cadence instead of waiting for
  // another coarse feedback timer tick. Batches never overlap.
  scheduleFeedbackReads();
  // Acquire position/velocity as a shared batch. Independent per-node write
  // phases otherwise drift apart and can produce fresh but incoherent samples.
  // Stops/disables retain priority; optional polling and normal targets resume
  // after the queued motion reads have received their bus slots.
  const bool motion_phase = std::any_of(sdo_channels_.begin(), sdo_channels_.end(),
      [](const auto& item) { return item.second.motionReadOutstanding(); });
  const auto frame = sdo_bus_scheduler_.next(sdo_nodes_, [this, motion_phase](unsigned node) -> std::optional<can_msgs::msg::Frame> {
    auto& setup = configuration_.at(sdo_node_names_.at(node));
    const auto stamp_ns = this->now().nanoseconds();
    if (!setup.ready() && !setup.failed())
    {
      if (!startup_.active() || !management_lease_.permitsEnable(supervisor_timeout_)) { return std::nullopt; }
      // Keep the old per-node configuration cadence. Sharing the bus must
      // spread the eight requests, not accelerate write/readback sequences.
      const auto now = SdoBusScheduler::Clock::now();
      const auto previous = last_configuration_request_.find(node);
      if (previous != last_configuration_request_.end() &&
          std::chrono::duration<double>(now - previous->second).count() < feedback_request_period_)
      { return std::nullopt; }
      const auto request = setup.next(now, stamp_ns);
      if (request) { last_configuration_request_[node] = now; }
      return request;
    }
    const auto& drive = drives_.at(sdo_node_names_.at(node));
    const auto steady_now = FeedbackFreshness::Clock::now();
    const bool native_enable_fresh = drive.hasNativeStatus() && drive.currentMode() == drive.targetMode() &&
        drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now) &&
        drive.feedbackFreshness().fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, steady_now);
    return sdo_channels_[node].next(watchdog_timeout_, SdoChannel::Clock::now(), stamp_ns, motion_phase, native_enable_fresh);
  });
  if (frame)
  {
    pub_can_frames_->publish(stampFrame(*frame, "can", this->now()));
    sdo_bus_scheduler_.published();
    // Re-arm from this publication so the next timer slot is a full gap away,
    // rather than rounding every request up to another free-running tick.
    if (sdo_dispatch_timer_) { sdo_dispatch_timer_->reset(); }
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
  if (!virtual_position_source_ && !controller_control_started_) { return; }
  if (management_lease_.permitsEnable(supervisor_timeout_)) { return; }
  enable_probe_after_ns_ = 0;
  if (!virtual_position_source_ && startup_.active()) { failStartup(); }
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
  if (msg.supervisor_instance.empty()) { return; }
  if (retired_authorities_.count(msg.supervisor_instance)) { return; }
  if (!authority_.supervisor_instance.empty() && msg.supervisor_instance == authority_.supervisor_instance &&
      msg.authority_generation < authority_.authority_generation) { return; }
  const bool new_epoch = msg.supervisor_instance != authority_.supervisor_instance || msg.authority_generation != authority_.authority_generation;
  if (!motion_source_gate_.update(msg.selected_mode, msg.transition_in_progress,
          rclcpp::Time(msg.header.stamp).nanoseconds(), now_ns, supervisor_timeout_)) { return; }
  if (!authority_.supervisor_instance.empty() && msg.supervisor_instance != authority_.supervisor_instance)
  {
    retired_authorities_.insert(authority_.supervisor_instance);
    reset_id_ = 0;
    startup_.newSupervisor();
    steering_alignment_failed_reset_id_ = 0;
    reset_fault_generation_.clear();
    reset_requested_ns_ = 0;
    if (!reset_release_sent_)
    {
      for (const auto& [name, drive] : drives_)
      {
        (void)name;
        publishCanFrame(stampFrame(drive.clearErrorRequest(NO_OPERATION), "can", this->now()));
      }
    }
    reset_release_sent_ = true;
  }
  authority_ = msg;
  if (!msg.transition_in_progress && traction_mode_initialized_ && msg.selected_mode <= 2 &&
      std::any_of(wheel_modules_.begin(), wheel_modules_.end(), [this, &msg](const auto& item) {
        return drives_.at(item.second.traction_drive).targetMode() != (msg.selected_mode == 2 ? CURRENT : VELOCITY);
      }))
  {
    // A transaction may have completed in hardware just as the supervisor
    // cancelled selection. Never expose that native mode under the old source.
    failStartup(true);
  }
  if (MotionSourceGate::immediateStop(msg.motion_permitted, msg.transition_in_progress,
          new_epoch, msg.safety_stop_active, msg.error_active, msg.reset_in_progress))
  { sendStopToAllDrives(); accepted_traction_targets_.clear(); }
  if (new_epoch) { motion_accept_after_ns_ = now_ns; motion_command_received_ = false; last_final_stamp_ns_ = 0; }
  if (generation != motion_source_gate_.generation())
  {
    motion_accept_after_ns_ = std::max(motion_accept_after_ns_, motion_source_gate_.boundary());
    motion_command_received_ = false;
    // Do not interrupt the old path's controlled deceleration at transition
    // start. At completion/recovery clear cached steering targets before
    // permitting any new source, including same-mode reselection.
    if (!was_fresh || old_mode != msg.selected_mode ||
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
  return drive.hasNativeStatus() && drive.currentMode() != UNKNOWN &&
         drive.hasFeedback(POSITION_FEEDBACK) && drive.hasFeedback(VELOCITY_FEEDBACK) &&
         drive.feedbackFreshness().ready(motion_feedback_timeout_,
                                        status_feedback_timeout_, fault_feedback_timeout_, now) &&
         drive.feedbackFreshness().fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, now);
}

std::string MoboticDriver::feedbackProblem(const MoboticDriveCanCommunication& drive,
    FeedbackFreshness::Clock::time_point now) const
{
  const auto& feedback = drive.feedbackFreshness();
  if (!feedback.fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, now)) { return "status stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::CONTROL_MODE, status_feedback_timeout_, now)) { return "control mode stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::VELOCITY, motion_feedback_timeout_, now)) { return "velocity stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::POSITION, motion_feedback_timeout_, now)) { return "position stale/missing"; }
  if (!feedback.fresh(FeedbackFreshness::ERROR_REGISTER, fault_feedback_timeout_, now)) { return "error register stale/missing"; }

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
  if (!current_overspeed_latched_ && std::any_of(drives_.begin(), drives_.end(), [this, steady_now](const auto& item) {
        const auto& drive = item.second;
        return drive.isTractionJoint() && drive.currentMode() == CURRENT &&
            drive.hasFeedback(VELOCITY_FEEDBACK) &&
            drive.feedbackFreshness().fresh(FeedbackFreshness::VELOCITY, motion_feedback_timeout_, steady_now) &&
            std::abs(drive.velocityFeedbackRadPerSec()) * wheel_radius_ > max_traction_velocity_;
      }))
  {
    current_overspeed_latched_ = true;
    sendStopToAllDrives(); disableAllDrives();
    RCLCPP_ERROR(this->get_logger(), "Current-mode speed limit exceeded; explicit reset required");
  }
  const bool complete_motion_stale = !virtual_position_source_ && startup_.complete() &&
      (published_cycle_ns_ <= 0 || this->now().nanoseconds() - published_cycle_ns_ > motion_feedback_timeout_ * 1e9);
  const bool blocked = steeringAlignmentPending() || complete_motion_stale || current_overspeed_latched_ || startup_.failed() || traction_mode_pending_ || (!virtual_position_source_ && !startup_.complete()) || !allDriveFeedbackFresh(steady_now) || std::any_of(drives_.begin(), drives_.end(),
      [](const auto& entry) { return !entry.second.initialized() || !entry.second.enabled() || entry.second.hasError(); });
  if (blocked != feedback_motion_blocked_)
  {
    // Recovery never reuses targets or a command queued before feedback returned.
    motion_accept_after_ns_ = this->now().nanoseconds();
    motion_command_received_ = false;
    if (blocked)
    {
      // A motion interlock must not erase passive acquisition state.
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
  add_value(motion, "last_motion_set_span_ms", std::to_string(last_motion_scan_span_ms_));
  add_value(motion, "incoherent_motion_observations", std::to_string(incoherent_motion_sets_));
  const double joint_age = published_cycle_ns_ > 0 ?
      (this->now().nanoseconds() - published_cycle_ns_) * 1e-9 : std::numeric_limits<double>::infinity();
  add_value(motion, "oldest_published_motion_age_s", std::to_string(joint_age));
  if (!virtual_position_source_ && startup_.complete() && joint_age > motion_feedback_timeout_)
  { motion.level = Status::ERROR; motion.message = "Complete joint feedback stale: motion inhibited"; }
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
    for (auto field : {FeedbackFreshness::POSITION, FeedbackFreshness::VELOCITY,
                       FeedbackFreshness::STATUS, FeedbackFreshness::CONTROL_MODE, FeedbackFreshness::ERROR_REGISTER})
    {
      const char* labels[] = {"status", "velocity", "position", "current", "fault", "mode"};
      add_value(status, std::string(labels[field]) + "_age_s", std::to_string(drive.feedbackFreshness().age(field, steady_now)));
    }
    const bool faulted = drive.hasError();
    status.level = !can_fresh || !feedback_problem.empty() ? Status::STALE : faulted ? Status::ERROR :
        enable_all_drives_requested_ && !drive.enabled() ? Status::WARN : Status::OK;
    status.message = !can_fresh ? "No recent CAN response" : !feedback_problem.empty() ? feedback_problem : faulted ? "Drive fault" :
        drive.enabled() ? "Enabled" : "Disabled";
    add_value(status, "enabled", drive.enabled() ? "true" : "false");
    add_value(status, "controller_type", "micontrol");

    const bool actual_known = drive.hasNativeStatus() &&
          drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, status_feedback_timeout_, steady_now);
    const bool requested_known = drive.softwareEnableKnown(2.0 * diagnostic_sdo_period_, steady_now);
    add_value(status, "actual_state_source", "miControl DEV_Status 0x3002:0; enabled = bit 0");
    add_value(status, "actual_state_0x3002_raw", actual_known ? std::to_string(drive.nativeDeviceState()) : "unknown");
    add_value(status, "software_enable_0x3004", requested_known ?
          (drive.softwareEnableRequested() ? "true" : "false") : "unknown");

    add_value(status, "error", driveErrorText(drive));
    add_value(status, "feedback_fresh", feedback_problem.empty() ? "true" : "false");
    add_value(status, "motion_blocked", feedback_motion_blocked_ ? "true" : "false");
    for (const auto& field : {std::make_pair("status_age_s", FeedbackFreshness::STATUS),
                              std::make_pair("velocity_age_s", FeedbackFreshness::VELOCITY),
                              std::make_pair("error_register_age_s", FeedbackFreshness::ERROR_REGISTER)})
    {
      add_value(status, field.first, std::to_string(drive.feedbackFreshness().age(field.second, steady_now)));
    }

    array.status.push_back(status);
  }
  pub_diagnostics_->publish(array);
}

#ifndef MOBOTIC_DRIVER_NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  MoboticDriver driver{rclcpp::NodeOptions()};
  rclcpp::spin(driver.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
#endif
