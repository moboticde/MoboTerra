#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <stdexcept>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <mobotic_driver/frame_operations.h>
#include <mobotic_driver/feedback_freshness.hpp>
#include <mobotic_driver/steering_target.hpp>

// -----------------------------------------------------------------------------
// Unit conversion helpers
// -----------------------------------------------------------------------------
// miControl steering/traction velocity unit is motor rpm.
// ROS joint velocity is wheel/output angular velocity in rad/s.
inline double rpmToAngVel(const double& rpm, const double& gear_ratio)
{
  return rpm * 2.0 * M_PI / (gear_ratio * 60.0);
}

inline double angVelToRpm(const double& ang_vel, const double& gear_ratio)
{
  return ang_vel * 60.0 * gear_ratio / (2.0 * M_PI);
}

inline double ticksToAngle(const double& ticks, const double& resolution)
{
  return ticks * 2.0 * M_PI / resolution;
}

inline double angleToTicks(const double& angle, const double& resolution)
{
  return angle * resolution / (2.0 * M_PI);
}

// -----------------------------------------------------------------------------
// CANopen SDO frame helpers
// -----------------------------------------------------------------------------
inline can_msgs::msg::Frame createAndFillMoboticHeader(const unsigned& canNodeId,
                                                       const FrameOperationsTypes& type,
                                                       const unsigned parameterId,
                                                       const unsigned subindex)
{
  can_msgs::msg::Frame frame;
  frame.id = canNodeId;
  frame.dlc = 8;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  std::fill(frame.data.begin(), frame.data.end(), 0);
  frame.data[0] = type;
  shortToFrame(frame, 1, parameterId);
  frame.data[3] = subindex;
  return frame;
}

inline can_msgs::msg::Frame createHeartbeatFrame(unsigned can_node_id)
{
  can_msgs::msg::Frame frame;
  frame.id = 0x700 + can_node_id;
  frame.dlc = 1;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  std::fill(frame.data.begin(), frame.data.end(), 0);
  frame.data[0] = 0x05;
  return frame;
}

inline int signedIntFromSdoReadReply(const can_msgs::msg::Frame& frame, const unsigned& index)
{
  if (frame.data[0] == READ_REPLY_1B)
  {
    return static_cast<std::int8_t>(frame.data[index]);
  }
  if (frame.data[0] == READ_REPLY_2B)
  {
    return static_cast<std::int16_t>(shortFromFrame(frame, index));
  }
  return intFromFrame(frame, index);
}

inline void uint32ToFrame(can_msgs::msg::Frame& frame, const unsigned& index, const std::uint32_t& message)
{
  frame.data[index + 0] = message & 0xFF;
  frame.data[index + 1] = (message >> 8) & 0xFF;
  frame.data[index + 2] = (message >> 16) & 0xFF;
  frame.data[index + 3] = (message >> 24) & 0xFF;
}

// -----------------------------------------------------------------------------
// Logical states/modes
// -----------------------------------------------------------------------------
enum ClearErrorAction
{
  NO_OPERATION = 0x00,
  CLEAR = 0x01
};

enum DeviceMode : std::uint32_t
{
  UNKNOWN = 0x00,
  CURRENT = 0x02,
  VELOCITY = 0x03,   // miControl native velocity mode
  S_VELOCITY = 0x05, // miControl native velocity mode (steering or traction)
  POSITION = 0x07    // MiControl steering position mode
};

inline DeviceMode fromString(const std::string& mode)
{
  if (mode == "position")
    return POSITION;
  if (mode == "velocity")
    return VELOCITY;
  if (mode == "current")
    return CURRENT;
  if (mode == "s_velocity")
    return S_VELOCITY;
  return UNKNOWN;
}

enum DeviceState
{
  ENABLE = 0x01,
  DISABLE = 0x00
};

// -----------------------------------------------------------------------------
// Object dictionary IDs
// -----------------------------------------------------------------------------
enum ParameterIds : std::uint16_t
{
  // miControl steering and traction objects
  MICONTROL_CLEAR_ERROR_STATUS = 0x3000,
  MICONTROL_ERROR_REGISTER = 0x3001,
  MICONTROL_STATUS_REGISTER = 0x3002,
  MICONTROL_DEVICE_MODE = 0x3003,
  // Software enable command/readback, distinct from actual DEV_Status.
  MICONTROL_DEVICE_STATE = 0x3004,
  CURRENT_INPUT = 0x3200,
  MICONTROL_CURRENT_FEEDBACK = 0x3262,
  MICONTROL_VELOCITY_INPUT = 0x3300,
  MICONTROL_S_VELOCITY_INPUT = 0x3500,
  MICONTROL_VELOCITY_FEEDBACK = 0x3A04,
  ABSOLUTE_POSITION_INPUT = 0x3790,
  RELATIVE_POSITION_INPUT = 0x3791,
  POSITION_FEEDBACK = 0x3762,
};

// Generic aliases used by mobotic_driver.cpp.
constexpr ParameterIds CURRENT_FEEDBACK = MICONTROL_CURRENT_FEEDBACK;
constexpr ParameterIds VELOCITY_INPUT = MICONTROL_VELOCITY_INPUT;
constexpr ParameterIds STEERING_PROFILE_VELOCITY_INPUT = MICONTROL_VELOCITY_INPUT;
constexpr ParameterIds S_VELOCITY_INPUT = MICONTROL_S_VELOCITY_INPUT;
constexpr ParameterIds VELOCITY_FEEDBACK = MICONTROL_VELOCITY_FEEDBACK;
constexpr ParameterIds DEVICE_MODE = MICONTROL_DEVICE_MODE;
constexpr ParameterIds DEVICE_STATE = MICONTROL_DEVICE_STATE;

enum DriveLimits
{
  MIN_STEERING_VELOCITY,
  MAX_STEERING_VELOCITY,
  MIN_STEERING_POSITION,
  MAX_STEERING_POSITION,
  MIN_TRACTION_VELOCITY,
  MAX_TRACTION_VELOCITY,
  MIN_CURRENT,
  MAX_CURRENT
};

inline ParameterIds modeToInputId(const DeviceMode& mode)
{
  switch (mode)
  {
  case CURRENT:
    return CURRENT_INPUT;
  case VELOCITY:
    return MICONTROL_VELOCITY_INPUT;
  case S_VELOCITY:
    return MICONTROL_S_VELOCITY_INPUT;
  case POSITION:
    return ABSOLUTE_POSITION_INPUT;
  default:
    return MICONTROL_VELOCITY_INPUT;
  }
}

class MoboticDriveCanCommunication
{
public:
  MoboticDriveCanCommunication(const std::string& id,
                               const unsigned& canNodeID,
                               const DeviceMode& mode,
                               const int& resolution,
                               const double& gear_ratio,
                               const std::map<DriveLimits, double>& limits,
                               const std::string& controller_type = "")
      : id_(id), requestNodeID_(0x600 + canNodeID), replyNodeID_(0x580 + canNodeID), current_mode_(UNKNOWN),
        target_mode_(mode),
        traction_joint_(mode == VELOCITY || mode == CURRENT), limits_(limits),
        resolution_(resolution), gear_ratio_(gear_ratio)
  {
    if (!controller_type.empty() && controller_type != "micontrol")
    { throw std::invalid_argument("only micontrol controllers are supported"); }
    feedbacks_ = {{CURRENT_FEEDBACK, std::numeric_limits<double>::infinity()},
                  {VELOCITY_FEEDBACK, std::numeric_limits<double>::infinity()},
                  {POSITION_FEEDBACK, std::numeric_limits<double>::infinity()}};
  }

  bool isTractionJoint() const { return traction_joint_; }
  void setTractionOperatingMode(DeviceMode mode)
  {
    if (!traction_joint_ || (mode != VELOCITY && mode != CURRENT))
    { throw std::invalid_argument("traction operating mode must be velocity or current"); }
    target_mode_ = mode;
    initialized_ = false;
  }
  void prepareOperatingModeTransaction()
  {
    // Keep acquisition-order watermarks, but never reuse enabled/mode/motion
    // caches as acknowledgement of a new disabled configuration transaction.
    enabled_ = initialized_ = native_status_valid_ = software_enable_valid_ = false;
    current_mode_ = UNKNOWN;
    resetAllFeedbacks();
  }
  ParameterIds inputId(DeviceMode mode) const { return modeToInputId(mode); }
  void preserveControllerConfiguration(bool preserve) { preserve_configuration_ = preserve; }
  void setAutomaticModeInitialization(bool automatic) { automatic_mode_initialization_ = automatic; }
  unsigned rawCanNodeID() const { return requestNodeID_ - 0x600; }

  can_msgs::msg::Frame clearErrorRequest(const ClearErrorAction& action) const
  {
    can_msgs::msg::Frame frame =
        createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_4B, MICONTROL_CLEAR_ERROR_STATUS, 0x00);
    shortToFrame(frame, 4, action == CLEAR ? 0x0001 : 0x0000);
    return frame;
  }

  can_msgs::msg::Frame steeringProfileUpdateRequest() const
  {
    auto frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_4B, MICONTROL_CLEAR_ERROR_STATUS, 0);
    intToFrame(frame, 4, 5); // DEV_Cmd Update: latch the zero profile into the active move.
    return frame;
  }

  can_msgs::msg::Frame readErrorRegisterRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE,
                                      MICONTROL_ERROR_REGISTER, 0x00);
  }

  can_msgs::msg::Frame setDeviceModeRequest(const DeviceMode& mode) const
  {
    can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE, MICONTROL_DEVICE_MODE, 0x00);
    intToFrame(frame, 4, static_cast<int>(mode));
    return frame;
  }

  can_msgs::msg::Frame readDeviceModeRequest() const
  {
    return createAndFillMoboticHeader(
        requestNodeID_, READ_FROM_DEVICE, MICONTROL_DEVICE_MODE,
        0x00);
  }

  can_msgs::msg::Frame changeDeviceStateRequest(const DeviceState& state) const
  {
    can_msgs::msg::Frame frame =
        createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_4B, MICONTROL_DEVICE_STATE, 0x00);
    shortToFrame(frame, 4, state == ENABLE ? 0x0001 : 0x0000);
    return frame;
  }

  std::vector<can_msgs::msg::Frame> changeDeviceStateRequests(const DeviceState& state) const
  {
    return {changeDeviceStateRequest(state)};
  }

  std::vector<can_msgs::msg::Frame> enableFromObservedState() const
  {
    if (!initialized() || hasError() || enabled()) { return {}; }
    return changeDeviceStateRequests(ENABLE);
  }

  can_msgs::msg::Frame readDeviceStateRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE,
                                      MICONTROL_STATUS_REGISTER, 0x00);
  }

  can_msgs::msg::Frame readSoftwareEnableRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE, MICONTROL_DEVICE_STATE, 0x00);
  }

  can_msgs::msg::Frame readFeedbackRequest(const ParameterIds& parameter) const
  {
    const unsigned subindex = parameter == VELOCITY_FEEDBACK ? 1 : 0;
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE, parameter, subindex);
  }

  can_msgs::msg::Frame setTargetRequest(const ParameterIds inputParameter, const int& target) const
  {
    can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE, inputParameter, 0x00);
    intToFrame(frame, 4, target);
    return frame;
  }

  std::vector<can_msgs::msg::Frame> update(const can_msgs::msg::Frame& frame,
      FeedbackFreshness::Clock::time_point received_at = FeedbackFreshness::Clock::now())
  {
    std::vector<can_msgs::msg::Frame> frames;
    if (frame.is_error || frame.is_rtr || frame.is_extended || frame.dlc > 8) { return frames; }
    const int64_t source_stamp_ns = sourceStampNanoseconds(frame);
    if (source_stamp_ns < 0) { return frames; }
    const auto accept = [this, received_at, source_stamp_ns](FeedbackFreshness::Field field) {
      return feedback_freshness_.record(field, received_at, source_stamp_ns);
    };
    if (frame.id == 0x700 + requestNodeID_ - 0x600 && frame.dlc == 1 && frame.data[0] == 0)
    {
      initialized_ = enabled_ = native_status_valid_ = software_enable_valid_ = false;
      software_enable_sample_ = OrderedFeedbackSample{};
      current_mode_ = UNKNOWN;
      resetAllFeedbacks();
      feedback_freshness_ = FeedbackFreshness{};
      return frames;
    }
    if (!(frame.id == replyNodeID_ && frame.dlc == 8 && isReadReplyFromDevice(frame)))
    {
      return frames;
    }

    const auto parameterId = static_cast<ParameterIds>(shortFromFrame(frame, 1));
    const unsigned subindex = frame.data[3];
    const unsigned size = 4 - ((frame.data[0] >> 2) & 3);

    if (parameterId == MICONTROL_ERROR_REGISTER && subindex == 0 && size == 4)
    {
      if (!accept(FeedbackFreshness::ERROR_REGISTER)) { return frames; }
      error_ = intFromFrame(frame, 4);
      if (!initialized_ && !error_ && !preserve_configuration_ && automatic_mode_initialization_)
      {
        frames.push_back(setDeviceModeRequest(target_mode_));
        frames.push_back(readDeviceModeRequest());
      }
    }
    else if (parameterId == MICONTROL_DEVICE_MODE && subindex == 0 && size == 4)
    {
      if (!accept(FeedbackFreshness::CONTROL_MODE)) { return frames; }
      current_mode_ = static_cast<DeviceMode>(intFromFrame(frame, 4));
      if (current_mode_ == target_mode_) { initialized_ = true; }
    }
    else if (parameterId == MICONTROL_STATUS_REGISTER && subindex == 0 && size == 4)
    {
      if (!accept(FeedbackFreshness::STATUS)) { return frames; }
      native_device_state_ = intFromFrame(frame, 4);
      native_status_valid_ = true;
      enabled_ = (static_cast<uint32_t>(native_device_state_) & 0x00000001u) != 0;
    }
    else if (parameterId == MICONTROL_DEVICE_STATE && subindex == 0 &&
             (size == 1 || size == 4) && frame.data[4] <= 1 &&
             (size == 1 || (frame.data[5] == 0 && frame.data[6] == 0 && frame.data[7] == 0)))
    {
      // Requested software enable never supplies actual power-stage evidence.
      if (!software_enable_sample_.record(received_at, source_stamp_ns)) { return frames; }
      software_enable_requested_ = frame.data[4] != 0;
      software_enable_valid_ = true;
    }
    else if (parameterId == MICONTROL_CURRENT_FEEDBACK && subindex <= 1 && size == 4)
    {
      if (!accept(FeedbackFreshness::CURRENT)) { return frames; }
      feedbacks_[CURRENT_FEEDBACK] = intFromFrame(frame, 4);
    }
    else if (parameterId == MICONTROL_VELOCITY_FEEDBACK && subindex == 1 && size == 4)
    {
      if (!accept(FeedbackFreshness::VELOCITY)) { return frames; }
      feedbacks_[VELOCITY_FEEDBACK] = intFromFrame(frame, 4);
    }
    else if (parameterId == POSITION_FEEDBACK && subindex == 0 && size == 4)
    {
      if (!accept(FeedbackFreshness::POSITION)) { return frames; }
      feedbacks_[POSITION_FEEDBACK] = intFromFrame(frame, 4);
    }

    return frames;
  }

  DeviceMode currentMode() const { return current_mode_; }
  DeviceMode targetMode() const { return target_mode_; }

  const std::string& id() const { return id_; }
  const unsigned& canNodeID() const { return requestNodeID_; }
  const unsigned& replyNodeID() const { return replyNodeID_; }
  const double& feedback(const ParameterIds feedback_type) const { return feedbacks_.at(feedback_type); }

  double velocityFeedbackRadPerSec() const
  {
    return rpmToAngVel(feedbacks_.at(VELOCITY_FEEDBACK), gear_ratio_);
  }

  bool enabled() const { return enabled_; }
  const FeedbackFreshness& feedbackFreshness() const { return feedback_freshness_; }
  bool hasError() const
  {
    return error_ != 0 || (native_status_valid_ &&
        (static_cast<uint32_t>(native_device_state_) & 0x00000002u));
  }
  int activeErrorCode() const { return !hasError() ? 0 : error_ != 0 ? error_ : 1; }
  bool initialized() const { return initialized_ && current_mode_ == target_mode_; }
  bool hasNativeStatus() const { return native_status_valid_; }
  int32_t nativeDeviceState() const { return native_device_state_; }
  bool softwareEnableRequested() const { return software_enable_requested_; }
  bool softwareEnableKnown(double timeout, FeedbackFreshness::Clock::time_point now) const
  {
    const auto age = software_enable_sample_.age(now);
    return software_enable_valid_ && age >= 0.0 && age <= timeout;
  }

  bool hasFeedback(const ParameterIds feedback_type) const
  {
    if (feedback_type != CURRENT_FEEDBACK && feedback_type != VELOCITY_FEEDBACK && feedback_type != POSITION_FEEDBACK)
    {
      return false;
    }
    return feedbacks_.at(feedback_type) != std::numeric_limits<double>::infinity();
  }

  int getError() const { return error_; }

  bool steeringPositionRepresentable(double angle) const
  {
    return checkedSteeringPosition(angle, resolution_).has_value();
  }

  int getTarget(const ParameterIds inputId, const double& input) const
  {
    if (inputId == ABSOLUTE_POSITION_INPUT)
    {
      if (!steeringPositionRepresentable(input))
      {
        throw std::out_of_range("steering position is outside signed 32-bit CAN target range");
      }
      // Preserve any existing explicitly supplied tick limits, but never use
      // them to disguise an unrepresentable raw command as a valid target.
      const auto target = checkedCanPosition(std::clamp(angleToTicks(input, resolution_),
          limits_.at(MIN_STEERING_POSITION), limits_.at(MAX_STEERING_POSITION)));
      if (!target) { throw std::out_of_range("bounded steering target is outside signed 32-bit CAN range"); }
      return static_cast<int>(*target);
    }

    if (inputId == MICONTROL_VELOCITY_INPUT || inputId == MICONTROL_S_VELOCITY_INPUT)
    {
      const bool traction = traction_joint_;
      return static_cast<int>(std::lround(std::clamp(angVelToRpm(input, gear_ratio_),
                                                     limits_.at(traction ? MIN_TRACTION_VELOCITY : MIN_STEERING_VELOCITY),
                                                     limits_.at(traction ? MAX_TRACTION_VELOCITY : MAX_STEERING_VELOCITY))));
    }

    if (inputId == CURRENT_INPUT)
    {
      return static_cast<int>(std::lround(std::clamp(input * 1000.0, limits_.at(MIN_CURRENT), limits_.at(MAX_CURRENT))));
    }

    return 0;
  }

  void resetAllFeedbacks()
  {
    feedbacks_[CURRENT_FEEDBACK] = std::numeric_limits<double>::infinity();
    feedbacks_[VELOCITY_FEEDBACK] = std::numeric_limits<double>::infinity();
    feedbacks_[POSITION_FEEDBACK] = std::numeric_limits<double>::infinity();
  }

  double resolution() const { return resolution_; }
  double gearRatio() const { return gear_ratio_; }
  // Traction feedback counts motor revolutions; steering retains its installed
  // output-angle scale. Use the same gearing for position and velocity.
  double positionCountsPerOutputRevolution() const
  { return resolution_ * (isTractionJoint() ? gear_ratio_ : 1.0); }

  // 0x3262:0 uses mA; :1 signed-current telemetry uses the same scale.
  double motorCurrentAmps() const
  {
    return hasFeedback(CURRENT_FEEDBACK) ? feedback(CURRENT_FEEDBACK) / 1000.0 :
        std::numeric_limits<double>::quiet_NaN();
  }

private:
  std::string id_;
  unsigned requestNodeID_{0};
  unsigned replyNodeID_{0};
  DeviceMode current_mode_;
  DeviceMode target_mode_;
  bool enabled_{false};
  bool initialized_{false};
  bool preserve_configuration_{false};
  bool automatic_mode_initialization_{true};
  bool traction_joint_{false};
  bool native_status_valid_{false};
  bool software_enable_valid_{false}, software_enable_requested_{false};
  OrderedFeedbackSample software_enable_sample_;
  int32_t native_device_state_{0};
  int error_{0};
  std::map<ParameterIds, double> feedbacks_;
  FeedbackFreshness feedback_freshness_;
  std::map<DriveLimits, double> limits_;
  double resolution_;
  double gear_ratio_;
};

#endif // COMMUNICATION_H
