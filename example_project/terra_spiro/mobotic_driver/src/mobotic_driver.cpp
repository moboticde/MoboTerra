#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <mobotic_driver/communication.hpp>
#include <mobotic_driver/mobotic_driver.h>

namespace
{
constexpr int DEFAULT_TRACTION_PROFILE_ACCEL = 92714; // 1.0 m/s^2 for 225 mm wheel, 16:1, 4096 inc/rev
constexpr int DEFAULT_TRACTION_PROFILE_DECEL = 92714;
constexpr int DEFAULT_TRACTION_MAX_PROFILE_VELOCITY = 185429; // 2.0 m/s for 225 mm wheel, 16:1, 4096 inc/rev

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
} // namespace

MoboticDriver::MoboticDriver(const rclcpp::NodeOptions& options) : Node("mobotic_driver", options)
{
  this->declare_parameter("can_node_id", "");
  this->declare_parameter("feedback_request_period", 0.1);
  this->declare_parameter("can_heartbeat_period", 0.01);
  this->declare_parameter("watchdog_timeout", 1.0);
  this->declare_parameter("diagnostic_sdo_period", 1.0);
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

  bool params{true};
  params &= this->get_parameter("can_node_id", driver_can_node_id);
  params &= this->get_parameter("feedback_request_period", feedback_request_period);
  params &= this->get_parameter("can_heartbeat_period", can_heartbeat_period);
  params &= this->get_parameter("watchdog_timeout", watchdog_timeout_);
  params &= this->get_parameter("diagnostic_sdo_period", diagnostic_sdo_period_);
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

  if (traction_max_profile_velocity_ <= 0)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] traction_max_profile_velocity must be positive.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (diagnostic_sdo_period_ <= 0.0)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] diagnostic_sdo_period must be positive.");
    rclcpp::shutdown();
    std::exit(1);
  }

  if (telemetry_pdo_sync_divider_ < 1 || telemetry_pdo_sync_divider_ > 240)
  {
    RCLCPP_FATAL(this->get_logger(), "[mobotic_driver] telemetry_pdo_sync_divider must be in [1, 240].");
    rclcpp::shutdown();
    std::exit(1);
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

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));

  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()),
      [this](const can_msgs::msg::Frame& frame) { this->canFrameCallback(frame); });

  pub_joint_states_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  sub_joint_setpoints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_setpoints", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::JointState& state) { this->jointSetpointsCallback(state); });

  feedback_timer_ = this->create_timer(std::chrono::duration<double>{feedback_request_period},
                                       [this]() { this->feedbackTimerCallback(); });

  if (traction_pdo_enabled_ || telemetry_pdo_enabled_)
  {
    traction_pdo_timer_ = this->create_timer(std::chrono::duration<double>{traction_pdo_period},
                                             [this]() { this->tractionPdoTimerCallback(); });
  }

  watchdog_timer_setpoints_ = this->create_timer(std::chrono::duration<double>{watchdog_timeout_},
                                                 [this]() { this->watchdogTimerCallback(); });

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
        this->enableAllDrives();
        response->success = true;
        response->message = "Enable requested.";
      });

  srv_clear_errors_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_errors", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        for (auto& [name, drive] : drives_)
        {
          (void)name;
          pub_can_frames_->publish(stampFrame(drive.clearErrorRequest(CLEAR), "can", this->now()));
        }
        response->success = true;
        response->message = "Clear-error request sent.";
      });
}

MoboticDriver::~MoboticDriver()
{
  sendStopToAllDrives();
  disableAllDrives();
}

void MoboticDriver::canFrameCallback(const can_msgs::msg::Frame& frame)
{
  sensor_msgs::msg::JointState feedback_msg;
  bool all_feedbacks_available = true;

  for (auto& [name, drive] : drives_)
  {
    (void)name;

    const auto frames_to_send = drive.update(frame);
    for (const auto& frame_to_send : frames_to_send)
    {
      pub_can_frames_->publish(stampFrame(frame_to_send, "can", this->now()));
    }

    const bool needs_position_feedback = !drive.isTractionController();
    const bool drive_feedback_available =
        drive.enabled() && drive.hasFeedback(VELOCITY_FEEDBACK) &&
        (!needs_position_feedback || drive.hasFeedback(POSITION_FEEDBACK));

    all_feedbacks_available = all_feedbacks_available && drive_feedback_available;

    if (!drive_feedback_available)
    {
      continue;
    }

    feedback_msg.name.emplace_back(drive.id());
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
    feedback_msg.header.stamp = this->now();
    feedback_msg.header.frame_id = "can";
    pub_joint_states_->publish(feedback_msg);

    for (auto& [name, drive] : drives_)
    {
      (void)name;
      drive.resetAllFeedbacks();
    }

    feedbacks_requested_ = false;
  }
}

void MoboticDriver::tractionPdoTimerCallback()
{
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
    if (!drive.isTractionController())
    {
      continue;
    }

    pub_can_frames_->publish(stampFrame(drive.tractionPdoCommandFrame(), "can", this->now()));
    published_traction_command = true;
  }

  if (published_traction_command)
  {
    pub_can_frames_->publish(stampFrame(MoboticDriveCanCommunication::syncFrame(), "can", this->now()));
  }
}

void MoboticDriver::feedbackTimerCallback()
{
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
        if (traction_pdo_enabled_ && drive.isTractionController())
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
        if (drive.isTractionController())
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

    if (traction_pdo_enabled_ && drive.isTractionController())
    {
      if (shouldPoll(last_error_poll_, name, diagnostic_sdo_period_))
      {
        pub_can_frames_->publish(stampFrame(drive.readErrorRegisterRequest(), "can", this->now()));
        pub_can_frames_->publish(stampFrame(drive.readErrorCodeRequest(), "can", this->now()));
      }
      continue;
    }

    if (telemetry_pdo_enabled_ && drive.isSteeringController())
    {
      if (shouldPoll(last_status_poll_, name, diagnostic_sdo_period_))
      {
        pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
      }
      continue;
    }

    if (!(telemetry_pdo_enabled_ && drive.isSteeringController()))
    {
      pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(CURRENT_FEEDBACK), "can", this->now()));
    }
    pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(VELOCITY_FEEDBACK), "can", this->now()));
    if (shouldPoll(last_status_poll_, name, diagnostic_sdo_period_))
    {
      pub_can_frames_->publish(stampFrame(drive.readDeviceStateRequest(), "can", this->now()));
    }

    if (!drive.isTractionController())
    {
      pub_can_frames_->publish(stampFrame(drive.readFeedbackRequest(POSITION_FEEDBACK), "can", this->now()));
    }
  }

  feedbacks_requested_ = true;
}

void MoboticDriver::jointSetpointsCallback(const sensor_msgs::msg::JointState& msg)
{
  bool command_sent = false;

  for (auto& [name, drive] : drives_)
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), drive.id());

    if (it == msg.name.end())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "No setpoint for drive '" << drive.id() << "' in joint_setpoints message.");
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
    if (traction_pdo_enabled_ && drive.isTractionController() && input_id == EC_TRACTION_VELOCITY_INPUT)
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

    if (traction_pdo_enabled_ && drive.isTractionController() && stop_input_id == EC_TRACTION_VELOCITY_INPUT)
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
      if (traction_pdo_enabled_)
      {
        drive.setTractionPdoTarget(0);
        drive.setTractionPdoControlword(0x0000);
      }
      else
      {
        pub_can_frames_->publish(stampFrame(drive.setTargetRequest(VELOCITY_INPUT, 0), "can", this->now()));
      }
    }
    if (!(traction_pdo_enabled_ && drive.isTractionController()))
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
  RCLCPP_INFO(this->get_logger(), "[driver] Enabling all drives.");
  initialize_all_drives_requested_ = true;
  enable_all_drives_requested_ = true;
  last_initialize_attempt_.clear();
  last_enable_attempt_.clear();

  for (auto& [name, drive] : drives_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "[driver] Sending enable sequence to drive " << name << ".");
    if (traction_pdo_enabled_ && drive.isTractionController())
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
    if (traction_pdo_enabled_ && drive.isTractionController())
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
    if (drive.isTractionController())
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
    if (traction_pdo_enabled_ && drive.isTractionController())
    {
      for (const auto& frame : drive.tractionPdoConfigurationRequests(
               static_cast<unsigned>(telemetry_pdo_sync_divider_)))
      {
        pub_can_frames_->publish(stampFrame(frame, "can", this->now()));
      }
      pub_can_frames_->publish(stampFrame(drive.setDeviceModeRequest(drive.targetMode()), "can", this->now()));
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  MoboticDriver driver{rclcpp::NodeOptions()};
  rclcpp::spin(driver.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
