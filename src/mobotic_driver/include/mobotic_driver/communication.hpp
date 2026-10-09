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
// MiControl steering velocity unit is motor rpm.
// ROS joint velocity is wheel/output angular velocity in rad/s.
inline double rpmToAngVel(const double& rpm, const double& gear_ratio)
{
  return rpm * 2.0 * M_PI / (gear_ratio * 60.0);
}

inline double angVelToRpm(const double& ang_vel, const double& gear_ratio)
{
  return ang_vel * 60.0 * gear_ratio / (2.0 * M_PI);
}

// Elmo EC traction velocity unit is encoder increments per second.
// ROS joint velocity is wheel/output angular velocity in rad/s.
inline double ecTractionVelocityToAngVel(const double& increments_per_second,
                                         const double& gear_ratio,
                                         const double& resolution)
{
  return increments_per_second * 2.0 * M_PI / (gear_ratio * resolution);
}

inline double angVelToEcTractionVelocity(const double& ang_vel,
                                         const double& gear_ratio,
                                         const double& resolution)
{
  return ang_vel * gear_ratio * resolution / (2.0 * M_PI);
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

inline can_msgs::msg::Frame createNmtFrame(const unsigned command, const unsigned can_node_id)
{
  can_msgs::msg::Frame frame;
  frame.id = 0x000;
  frame.dlc = 2;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  std::fill(frame.data.begin(), frame.data.end(), 0);
  frame.data[0] = command;
  frame.data[1] = can_node_id;
  return frame;
}

inline can_msgs::msg::Frame createSyncFrame()
{
  can_msgs::msg::Frame frame;
  frame.id = 0x080;
  frame.dlc = 0;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  std::fill(frame.data.begin(), frame.data.end(), 0);
  return frame;
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

enum DeviceMode
{
  UNKNOWN = 0x00,
  CURRENT = 0x02,
  VELOCITY = 0x03,   // Elmo traction DS402 profile velocity mode
  S_VELOCITY = 0x05, // MiControl steering velocity mode, legacy support
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
enum ParameterIds
{
  // Elmo / DS402 traction objects
  EC_CONTROLWORD = 0x6040,
  EC_STATUSWORD = 0x6041,
  EC_TRACTION_MODE_OF_OPERATION = 0x6060,
  EC_TRACTION_VELOCITY_INPUT = 0x60FF,
  EC_TRACTION_VELOCITY_FEEDBACK = 0x606C,
  EC_TRACTION_MAX_PROFILE_VELOCITY = 0x607F,
  EC_TRACTION_PROFILE_ACCEL = 0x6083,
  EC_TRACTION_PROFILE_DECEL = 0x6084,

  EC_TRACTION_MAX_ACCEL = 0x60C5,
  EC_TRACTION_MAX_DECEL = 0x60C6,
  EC_TRACTION_CURRENT_FEEDBACK = 0x6078,
  EC_ERROR_REGISTER = 0x1001,
  EC_ERROR_CODE = 0x603F,
  EC_RPDO1_COMMUNICATION_PARAMETER = 0x1400,
  EC_RPDO1_MAPPING_PARAMETER = 0x1600,
  EC_TPDO1_COMMUNICATION_PARAMETER = 0x1800,
  EC_TPDO2_COMMUNICATION_PARAMETER = 0x1801,
  EC_TPDO3_COMMUNICATION_PARAMETER = 0x1802,
  EC_TPDO1_MAPPING_PARAMETER = 0x1A00,
  EC_TPDO2_MAPPING_PARAMETER = 0x1A01,
  EC_TPDO3_MAPPING_PARAMETER = 0x1A02,

  // MiControl steering objects
  MICONTROL_CLEAR_ERROR_STATUS = 0x3000,
  MICONTROL_ERROR_REGISTER = 0x3001,
  MICONTROL_DEVICE_MODE = 0x3003,
  MICONTROL_DEVICE_STATE = 0x3004,
  CURRENT_INPUT = 0x3200,
  MICONTROL_CURRENT_FEEDBACK = 0x3262,
  MICONTROL_STEERING_VELOCITY_INPUT = 0x3300,
  MICONTROL_STEERING_S_VELOCITY_INPUT = 0x3500,
  MICONTROL_VELOCITY_FEEDBACK = 0x3A04,
  ABSOLUTE_POSITION_INPUT = 0x3790,
  RELATIVE_POSITION_INPUT = 0x3791,
  POSITION_FEEDBACK = 0x3762,
};

constexpr unsigned CANOPEN_NMT_START_REMOTE_NODE = 0x01;
constexpr unsigned CANOPEN_NMT_ENTER_PRE_OPERATIONAL = 0x80;
constexpr unsigned CANOPEN_PDO_COB_ID_INVALID_BIT = 0x80000000;
constexpr unsigned CANOPEN_SYNCHRONOUS_EVERY_SYNC = 0x01;

constexpr std::uint32_t EC_TRACTION_RPDO_CONTROLWORD_MAPPING = 0x60400010;
constexpr std::uint32_t EC_TRACTION_RPDO_MODE_MAPPING = 0x60600008;
constexpr std::uint32_t EC_TRACTION_RPDO_TARGET_VELOCITY_MAPPING = 0x60FF0020;
constexpr std::uint32_t EC_TRACTION_TPDO_STATUSWORD_MAPPING = 0x60410010;
constexpr std::uint32_t EC_TRACTION_TPDO_MODE_DISPLAY_MAPPING = 0x60610008;
constexpr std::uint32_t EC_TRACTION_TPDO_ACTUAL_VELOCITY_MAPPING = 0x606C0020;
constexpr std::uint32_t EC_TRACTION_TPDO_ACTUAL_CURRENT_MAPPING = 0x60780010;
constexpr std::uint32_t EC_TRACTION_TPDO_DRIVE_TEMPERATURE_MAPPING = 0x22A20010;

constexpr std::uint32_t MICONTROL_TPDO_BUS_VOLTAGE_MAPPING = 0x31110020;
constexpr std::uint32_t MICONTROL_TPDO_ACTUAL_CURRENT_MAPPING = 0x32620120;
constexpr std::uint32_t MICONTROL_TPDO_RMS_CURRENT_MAPPING = 0x32620020;
constexpr std::uint32_t MICONTROL_TPDO_CONTROLLER_TEMPERATURE_MAPPING = 0x31140020;
constexpr std::uint32_t MICONTROL_TPDO_POSITION_MAPPING = 0x37620020;
constexpr std::uint32_t MICONTROL_TPDO_VELOCITY_MAPPING = 0x3A040120;

// Generic aliases used by mobotic_driver.cpp.
constexpr ParameterIds CURRENT_FEEDBACK = EC_TRACTION_CURRENT_FEEDBACK;
constexpr ParameterIds VELOCITY_INPUT = EC_TRACTION_VELOCITY_INPUT;
constexpr ParameterIds STEERING_PROFILE_VELOCITY_INPUT = MICONTROL_STEERING_VELOCITY_INPUT;
constexpr ParameterIds S_VELOCITY_INPUT = MICONTROL_STEERING_S_VELOCITY_INPUT;
constexpr ParameterIds VELOCITY_FEEDBACK = EC_TRACTION_VELOCITY_FEEDBACK;
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
    return EC_TRACTION_VELOCITY_INPUT;
  case S_VELOCITY:
    return MICONTROL_STEERING_S_VELOCITY_INPUT;
  case POSITION:
    return ABSOLUTE_POSITION_INPUT;
  default:
    return EC_TRACTION_VELOCITY_INPUT;
  }
}

// Read-only register telemetry: newer aborts invalidate availability, but an
// older abort/read reply cannot erase or restore a newer result.
struct RawRegisterTelemetry
{
  uint32_t value{0};
  bool available{false};
  OrderedFeedbackSample ordering;

  bool update(const can_msgs::msg::Frame& frame, OrderedFeedbackSample::Clock::time_point acquired_at)
  {
    if (frame.is_error || frame.is_rtr || frame.is_extended || frame.dlc != 8) { return false; }
    const int64_t source_stamp_ns = sourceStampNanoseconds(frame);
    const bool read_reply = isReadReplyFromDevice(frame);
    if (source_stamp_ns < 0 || (!read_reply && frame.data[0] != ABORT_REPLY) ||
        !ordering.record(acquired_at, source_stamp_ns))
    {
      return false;
    }
    available = read_reply;
    if (read_reply)
    {
      value = 0;
      const unsigned size = 4 - ((frame.data[0] >> 2) & 3);
      for (unsigned byte = 0; byte < size; ++byte)
      {
        value |= static_cast<uint32_t>(frame.data[4 + byte]) << (8 * byte);
      }
    }
    return true;
  }
};

class MoboticDriveCanCommunication
{
public:
  MoboticDriveCanCommunication(const std::string& id,
                               const unsigned& canNodeID,
                               const DeviceMode& mode,
                               const int& resolution,
                               const double& gear_ratio,
                               const std::map<DriveLimits, double>& limits)
      : id_(id), requestNodeID_(0x600 + canNodeID), replyNodeID_(0x580 + canNodeID), current_mode_(UNKNOWN),
        target_mode_(mode), is_traction_controller_(mode == VELOCITY || mode == CURRENT), limits_(limits),
        resolution_(resolution), gear_ratio_(gear_ratio)
  {
    feedbacks_ = {{CURRENT_FEEDBACK, std::numeric_limits<double>::infinity()},
                  {VELOCITY_FEEDBACK, std::numeric_limits<double>::infinity()},
                  {POSITION_FEEDBACK, std::numeric_limits<double>::infinity()}};
  }

  // Current robot convention:
  //   velocity  -> Elmo traction
  //   position  -> MiControl steering
  //   s_velocity -> MiControl steering velocity mode
  bool isTractionController() const { return is_traction_controller_; }
  bool isSteeringController() const { return !isTractionController(); }
  bool supportsTractionVelocityPdo() const { return isTractionController() && target_mode_ == VELOCITY; }
  void setTractionController(const bool is_traction_controller) { is_traction_controller_ = is_traction_controller; }

  unsigned rawCanNodeID() const { return requestNodeID_ - 0x600; }
  unsigned tractionRpdoCobId() const { return 0x200 + rawCanNodeID(); }
  unsigned tractionTpdoCobId() const { return 0x180 + rawCanNodeID(); }
  unsigned telemetryTpdo2CobId() const { return 0x280 + rawCanNodeID(); }
  unsigned telemetryTpdo3CobId() const { return 0x380 + rawCanNodeID(); }

  static can_msgs::msg::Frame syncFrame()
  {
    return createSyncFrame();
  }

  can_msgs::msg::Frame nmtPreOperationalRequest() const
  {
    return createNmtFrame(CANOPEN_NMT_ENTER_PRE_OPERATIONAL, rawCanNodeID());
  }

  can_msgs::msg::Frame nmtOperationalRequest() const
  {
    return createNmtFrame(CANOPEN_NMT_START_REMOTE_NODE, rawCanNodeID());
  }

  std::vector<can_msgs::msg::Frame> tractionPdoConfigurationRequests(
      const unsigned telemetry_sync_divider) const
  {
    if (!isTractionController())
    {
      return {};
    }

    const auto make_sdo_u8 = [this](const ParameterIds index, const unsigned subindex, const unsigned value) {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_1B, index, subindex);
      frame.data[4] = value & 0xFF;
      return frame;
    };

    const auto make_sdo_u32 = [this](const ParameterIds index, const unsigned subindex, const std::uint32_t value) {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_4B, index, subindex);
      uint32ToFrame(frame, 4, value);
      return frame;
    };

    const std::uint32_t rpdo_cob_id = tractionRpdoCobId();
    const std::uint32_t tpdo_cob_id = tractionTpdoCobId();
    const std::uint32_t telemetry_tpdo_cob_id = telemetryTpdo2CobId();

    return {
      nmtPreOperationalRequest(),
      make_sdo_u32(EC_RPDO1_COMMUNICATION_PARAMETER, 0x01, CANOPEN_PDO_COB_ID_INVALID_BIT | rpdo_cob_id),
      make_sdo_u32(EC_TPDO1_COMMUNICATION_PARAMETER, 0x01, CANOPEN_PDO_COB_ID_INVALID_BIT | tpdo_cob_id),
      make_sdo_u32(EC_TPDO2_COMMUNICATION_PARAMETER, 0x01,
                   CANOPEN_PDO_COB_ID_INVALID_BIT | telemetry_tpdo_cob_id),
      make_sdo_u8(EC_RPDO1_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u8(EC_TPDO1_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u8(EC_TPDO2_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u32(EC_RPDO1_MAPPING_PARAMETER, 0x01, EC_TRACTION_RPDO_CONTROLWORD_MAPPING),
      make_sdo_u32(EC_RPDO1_MAPPING_PARAMETER, 0x02, EC_TRACTION_RPDO_TARGET_VELOCITY_MAPPING),
      make_sdo_u32(EC_TPDO1_MAPPING_PARAMETER, 0x01, EC_TRACTION_TPDO_STATUSWORD_MAPPING),
      make_sdo_u32(EC_TPDO1_MAPPING_PARAMETER, 0x02, EC_TRACTION_TPDO_MODE_DISPLAY_MAPPING),
      make_sdo_u32(EC_TPDO1_MAPPING_PARAMETER, 0x03, EC_TRACTION_TPDO_ACTUAL_VELOCITY_MAPPING),
      make_sdo_u32(EC_TPDO2_MAPPING_PARAMETER, 0x01, EC_TRACTION_TPDO_ACTUAL_CURRENT_MAPPING),
      make_sdo_u32(EC_TPDO2_MAPPING_PARAMETER, 0x02, EC_TRACTION_TPDO_DRIVE_TEMPERATURE_MAPPING),
      make_sdo_u8(EC_RPDO1_MAPPING_PARAMETER, 0x00, 2),
      make_sdo_u8(EC_TPDO1_MAPPING_PARAMETER, 0x00, 3),
      make_sdo_u8(EC_TPDO2_MAPPING_PARAMETER, 0x00, 2),
      make_sdo_u8(EC_RPDO1_COMMUNICATION_PARAMETER, 0x02, CANOPEN_SYNCHRONOUS_EVERY_SYNC),
      make_sdo_u8(EC_TPDO1_COMMUNICATION_PARAMETER, 0x02, CANOPEN_SYNCHRONOUS_EVERY_SYNC),
      make_sdo_u8(EC_TPDO2_COMMUNICATION_PARAMETER, 0x02, telemetry_sync_divider),
      make_sdo_u32(EC_RPDO1_COMMUNICATION_PARAMETER, 0x01, rpdo_cob_id),
      make_sdo_u32(EC_TPDO1_COMMUNICATION_PARAMETER, 0x01, tpdo_cob_id),
      make_sdo_u32(EC_TPDO2_COMMUNICATION_PARAMETER, 0x01, telemetry_tpdo_cob_id),
      nmtOperationalRequest(),
    };
  }

  std::vector<can_msgs::msg::Frame> steeringTelemetryPdoConfigurationRequests(
      const unsigned telemetry_sync_divider) const
  {
    if (!isSteeringController())
    {
      return {};
    }

    const auto make_sdo_u8 = [this](const ParameterIds index, const unsigned subindex, const unsigned value) {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_1B, index, subindex);
      frame.data[4] = value & 0xFF;
      return frame;
    };

    const auto make_sdo_u32 = [this](const ParameterIds index, const unsigned subindex, const std::uint32_t value) {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_4B, index, subindex);
      uint32ToFrame(frame, 4, value);
      return frame;
    };

    const std::uint32_t tpdo1_cob_id = tractionTpdoCobId();
    const std::uint32_t tpdo2_cob_id = telemetryTpdo2CobId();
    const std::uint32_t tpdo3_cob_id = telemetryTpdo3CobId();

    return {
      nmtPreOperationalRequest(),
      make_sdo_u32(EC_TPDO1_COMMUNICATION_PARAMETER, 0x01, CANOPEN_PDO_COB_ID_INVALID_BIT | tpdo1_cob_id),
      make_sdo_u32(EC_TPDO2_COMMUNICATION_PARAMETER, 0x01, CANOPEN_PDO_COB_ID_INVALID_BIT | tpdo2_cob_id),
      make_sdo_u32(EC_TPDO3_COMMUNICATION_PARAMETER, 0x01, CANOPEN_PDO_COB_ID_INVALID_BIT | tpdo3_cob_id),
      make_sdo_u8(EC_TPDO1_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u8(EC_TPDO2_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u8(EC_TPDO3_MAPPING_PARAMETER, 0x00, 0),
      make_sdo_u32(EC_TPDO1_MAPPING_PARAMETER, 0x01, MICONTROL_TPDO_POSITION_MAPPING),
      make_sdo_u32(EC_TPDO1_MAPPING_PARAMETER, 0x02, MICONTROL_TPDO_VELOCITY_MAPPING),
      make_sdo_u32(EC_TPDO2_MAPPING_PARAMETER, 0x01, MICONTROL_TPDO_BUS_VOLTAGE_MAPPING),
      make_sdo_u32(EC_TPDO2_MAPPING_PARAMETER, 0x02, MICONTROL_TPDO_ACTUAL_CURRENT_MAPPING),
      make_sdo_u32(EC_TPDO3_MAPPING_PARAMETER, 0x01, MICONTROL_TPDO_CONTROLLER_TEMPERATURE_MAPPING),
      make_sdo_u32(EC_TPDO3_MAPPING_PARAMETER, 0x02, MICONTROL_TPDO_RMS_CURRENT_MAPPING),
      make_sdo_u8(EC_TPDO1_MAPPING_PARAMETER, 0x00, 2),
      make_sdo_u8(EC_TPDO2_MAPPING_PARAMETER, 0x00, 2),
      make_sdo_u8(EC_TPDO3_MAPPING_PARAMETER, 0x00, 2),
      make_sdo_u8(EC_TPDO1_COMMUNICATION_PARAMETER, 0x02, telemetry_sync_divider),
      make_sdo_u8(EC_TPDO2_COMMUNICATION_PARAMETER, 0x02, telemetry_sync_divider),
      make_sdo_u8(EC_TPDO3_COMMUNICATION_PARAMETER, 0x02, telemetry_sync_divider),
      make_sdo_u32(EC_TPDO1_COMMUNICATION_PARAMETER, 0x01, tpdo1_cob_id),
      make_sdo_u32(EC_TPDO2_COMMUNICATION_PARAMETER, 0x01, tpdo2_cob_id),
      make_sdo_u32(EC_TPDO3_COMMUNICATION_PARAMETER, 0x01, tpdo3_cob_id),
      nmtOperationalRequest(),
    };
  }

  void setTractionPdoControlword(const unsigned short controlword)
  {
    traction_pdo_controlword_ = controlword;
  }

  void setTractionPdoTarget(const int target)
  {
    traction_pdo_target_velocity_ = target;
  }

  can_msgs::msg::Frame tractionPdoCommandFrame() const
  {
    can_msgs::msg::Frame frame;
    frame.id = tractionRpdoCobId();
    frame.dlc = 6;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    std::fill(frame.data.begin(), frame.data.end(), 0);
    shortToFrame(frame, 0, static_cast<short>(traction_pdo_controlword_));
    intToFrame(frame, 2, traction_pdo_target_velocity_);
    return frame;
  }

  can_msgs::msg::Frame clearErrorRequest(const ClearErrorAction& action) const
  {
    if (isTractionController())
    {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_2B, EC_CONTROLWORD, 0x00);
      shortToFrame(frame, 4, action == CLEAR ? 0x0080 : 0x0000);
      return frame;
    }

    can_msgs::msg::Frame frame =
        createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_2B, MICONTROL_CLEAR_ERROR_STATUS, 0x00);
    shortToFrame(frame, 4, action == CLEAR ? 0x0001 : 0x0000);
    return frame;
  }

  can_msgs::msg::Frame readErrorRegisterRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE,
                                      isTractionController() ? EC_ERROR_REGISTER : MICONTROL_ERROR_REGISTER, 0x00);
  }

  can_msgs::msg::Frame readErrorCodeRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE,
                                      isTractionController() ? EC_ERROR_CODE : MICONTROL_ERROR_REGISTER, 0x00);
  }

  can_msgs::msg::Frame setDeviceModeRequest(const DeviceMode& mode) const
  {
    if (isTractionController())
    {
      can_msgs::msg::Frame frame =
          createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_1B, EC_TRACTION_MODE_OF_OPERATION, 0x00);
      frame.data[4] = static_cast<unsigned char>(mode);
      return frame;
    }

    can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE, MICONTROL_DEVICE_MODE, 0x00);
    intToFrame(frame, 4, static_cast<int>(mode));
    return frame;
  }

  can_msgs::msg::Frame readDeviceModeRequest() const
  {
    return createAndFillMoboticHeader(
        requestNodeID_, READ_FROM_DEVICE, isTractionController() ? EC_TRACTION_MODE_OF_OPERATION : MICONTROL_DEVICE_MODE,
        0x00);
  }

  can_msgs::msg::Frame changeDeviceStateRequest(const DeviceState& state) const
  {
    if (isTractionController())
    {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_2B, EC_CONTROLWORD, 0x00);
      shortToFrame(frame, 4, state == ENABLE ? 0x000F : 0x0000);
      return frame;
    }

    can_msgs::msg::Frame frame =
        createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_2B, MICONTROL_DEVICE_STATE, 0x00);
    shortToFrame(frame, 4, state == ENABLE ? 0x0001 : 0x0000);
    return frame;
  }

  std::vector<can_msgs::msg::Frame> changeDeviceStateRequests(const DeviceState& state) const
  {
    if (!isTractionController() || state == DISABLE)
    {
      return {changeDeviceStateRequest(state)};
    }

    const auto make_controlword_frame = [this](const short controlword) {
      can_msgs::msg::Frame frame = createAndFillMoboticHeader(requestNodeID_, WRITE_TO_DEVICE_2B, EC_CONTROLWORD, 0x00);
      shortToFrame(frame, 4, controlword);
      return frame;
    };

    if (has_statusword_ && (statusword_ & 0x006F) == 0x0027)
    {
      return {};
    }

    std::vector<can_msgs::msg::Frame> frames;
    if (has_statusword_ && (statusword_ & 0x004F) == 0x0008)
    {
      frames.push_back(make_controlword_frame(0x0080));
    }

    frames.push_back(make_controlword_frame(0x0006));
    frames.push_back(make_controlword_frame(0x0007));
    frames.push_back(make_controlword_frame(0x000F));
    return frames;
  }

  can_msgs::msg::Frame readDeviceStateRequest() const
  {
    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE,
                                      isTractionController() ? EC_STATUSWORD : MICONTROL_DEVICE_STATE, 0x00);
  }

  can_msgs::msg::Frame readFeedbackRequest(const ParameterIds& parameter) const
  {
    ParameterIds feedback_parameter = parameter;
    unsigned subindex = 0x00;

    if (parameter == CURRENT_FEEDBACK)
    {
      feedback_parameter = isTractionController() ? EC_TRACTION_CURRENT_FEEDBACK : MICONTROL_CURRENT_FEEDBACK;
      subindex = isTractionController() ? 0x00 : 0x01;
    }
    else if (parameter == VELOCITY_FEEDBACK)
    {
      feedback_parameter = isTractionController() ? EC_TRACTION_VELOCITY_FEEDBACK : MICONTROL_VELOCITY_FEEDBACK;
      subindex = isTractionController() ? 0x00 : 0x01;
    }
    else if (parameter == POSITION_FEEDBACK)
    {
      if (isTractionController())
      {
        return readDeviceStateRequest();
      }
      feedback_parameter = POSITION_FEEDBACK;
      subindex = 0x00;
    }

    return createAndFillMoboticHeader(requestNodeID_, READ_FROM_DEVICE, feedback_parameter, subindex);
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
    if (isTractionController() && frame.id == tractionTpdoCobId() && frame.dlc >= 7)
    {
      // PDO and SDO sources share per-field ordering. A newer SDO status must
      // survive an older PDO, without discarding genuinely newer PDO velocity.
      if (accept(FeedbackFreshness::STATUS))
      {
        statusword_ = static_cast<unsigned short>(shortFromFrame(frame, 0));
        has_statusword_ = true;
        enabled_ = (statusword_ & 0x006F) == 0x0027;
      }
      if (accept(FeedbackFreshness::CONTROL_MODE))
      {
        current_mode_ = static_cast<DeviceMode>(static_cast<std::int8_t>(frame.data[2]));
        if (current_mode_ == target_mode_ && !initialized_) { initialized_ = true; }
      }
      if (accept(FeedbackFreshness::VELOCITY)) { feedbacks_[VELOCITY_FEEDBACK] = intFromFrame(frame, 3); }
      return frames;
    }

    if (isSteeringController() && frame.id == tractionTpdoCobId() && frame.dlc >= 8)
    {
      if (accept(FeedbackFreshness::POSITION)) { feedbacks_[POSITION_FEEDBACK] = intFromFrame(frame, 0); }
      if (accept(FeedbackFreshness::VELOCITY)) { feedbacks_[VELOCITY_FEEDBACK] = intFromFrame(frame, 4); }
      return frames;
    }

    if (frame.id == telemetryTpdo2CobId())
    {
      if (isTractionController() && frame.dlc >= 2 && accept(FeedbackFreshness::CURRENT))
      {
        feedbacks_[CURRENT_FEEDBACK] = static_cast<std::int16_t>(shortFromFrame(frame, 0));
      }
      else if (isSteeringController() && frame.dlc >= 8 && accept(FeedbackFreshness::CURRENT))
      {
        feedbacks_[CURRENT_FEEDBACK] = intFromFrame(frame, 4);
      }
      return frames;
    }

    if (!(frame.id == replyNodeID_ && frame.dlc == 8 && isReadReplyFromDevice(frame)))
    {
      return frames;
    }

    const auto parameterId = static_cast<ParameterIds>(shortFromFrame(frame, 1));
    const unsigned subindex = frame.data[3];
    const unsigned size = 4 - ((frame.data[0] >> 2) & 3);

    if (subindex == 0 && ((isTractionController() && parameterId == EC_ERROR_REGISTER) ||
        (isSteeringController() && parameterId == MICONTROL_ERROR_REGISTER && size == 4)))
    {
      if (!accept(FeedbackFreshness::ERROR_REGISTER)) { return frames; }
      error_ = isTractionController() ? frame.data[4] : intFromFrame(frame, 4);
      if (!initialized_ && !error_)
      {
        frames.push_back(setDeviceModeRequest(target_mode_));
        frames.push_back(readDeviceModeRequest());
      }
      else if (isTractionController() && error_)
      {
        frames.push_back(readErrorCodeRequest());
      }
    }
    else if (isTractionController() && parameterId == EC_ERROR_CODE && subindex == 0 && size >= 2)
    {
      if (!accept(FeedbackFreshness::ERROR_CODE)) { return frames; }
      error_code_ = static_cast<uint16_t>(shortFromFrame(frame, 4));
    }
    else if (subindex == 0 && ((isSteeringController() && parameterId == MICONTROL_DEVICE_MODE && size == 4) ||
             (isTractionController() && parameterId == EC_TRACTION_MODE_OF_OPERATION)))
    {
      if (!accept(FeedbackFreshness::CONTROL_MODE)) { return frames; }
      current_mode_ = static_cast<DeviceMode>(frame.data[4]);
      if (current_mode_ == target_mode_ && !initialized_)
      {
        initialized_ = true;
      }
    }
    else if (isTractionController() && parameterId == EC_STATUSWORD && subindex == 0 && size >= 2)
    {
      if (!accept(FeedbackFreshness::STATUS)) { return frames; }
      statusword_ = static_cast<unsigned short>(shortFromFrame(frame, 4));
      has_statusword_ = true;
      enabled_ = (statusword_ & 0x006F) == 0x0027;
    }
    else if (isSteeringController() && parameterId == MICONTROL_DEVICE_STATE && subindex == 0 && size >= 2)
    {
      if (!accept(FeedbackFreshness::STATUS)) { return frames; }
      enabled_ = frame.data[4] != 0;
    }
    else if ((isTractionController() && parameterId == EC_TRACTION_CURRENT_FEEDBACK && subindex == 0 && size >= 2) ||
             (isSteeringController() && parameterId == MICONTROL_CURRENT_FEEDBACK && subindex == 1 && size == 4))
    {
      if (!accept(FeedbackFreshness::CURRENT)) { return frames; }
      feedbacks_[CURRENT_FEEDBACK] = signedIntFromSdoReadReply(frame, 4);
    }
    else if (size == 4 && ((isTractionController() && parameterId == EC_TRACTION_VELOCITY_FEEDBACK && subindex == 0) ||
             (isSteeringController() && parameterId == MICONTROL_VELOCITY_FEEDBACK && subindex == 1)))
    {
      if (!accept(FeedbackFreshness::VELOCITY)) { return frames; }
      feedbacks_[VELOCITY_FEEDBACK] = intFromFrame(frame, 4);
    }
    else if (isSteeringController() && parameterId == POSITION_FEEDBACK && subindex == 0 && size == 4)
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
    if (isTractionController())
    {
      return ecTractionVelocityToAngVel(feedbacks_.at(VELOCITY_FEEDBACK), gear_ratio_, resolution_);
    }
    return rpmToAngVel(feedbacks_.at(VELOCITY_FEEDBACK), gear_ratio_);
  }

  bool enabled() const { return enabled_; }
  const FeedbackFreshness& feedbackFreshness() const { return feedback_freshness_; }
  bool hasError() const { return error_ != 0; }
  bool initialized() const { return initialized_; }
  bool hasStatusword() const { return has_statusword_; }
  unsigned short statusword() const { return statusword_; }

  bool hasFeedback(const ParameterIds feedback_type) const
  {
    if (feedback_type != CURRENT_FEEDBACK && feedback_type != VELOCITY_FEEDBACK && feedback_type != POSITION_FEEDBACK)
    {
      return false;
    }
    return feedbacks_.at(feedback_type) != std::numeric_limits<double>::infinity();
  }

  int getError() const { return error_; }
  int getErrorCode() const { return error_code_; }

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

    if (inputId == EC_TRACTION_VELOCITY_INPUT)
    {
      return static_cast<int>(std::lround(std::clamp(angVelToEcTractionVelocity(input, gear_ratio_, resolution_),
                                                     limits_.at(MIN_TRACTION_VELOCITY),
                                                     limits_.at(MAX_TRACTION_VELOCITY))));
    }

    if (inputId == MICONTROL_STEERING_VELOCITY_INPUT || inputId == MICONTROL_STEERING_S_VELOCITY_INPUT)
    {
      return static_cast<int>(std::lround(std::clamp(angVelToRpm(input, gear_ratio_),
                                                     limits_.at(MIN_STEERING_VELOCITY),
                                                     limits_.at(MAX_STEERING_VELOCITY))));
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

private:
  std::string id_;
  unsigned requestNodeID_{0};
  unsigned replyNodeID_{0};
  DeviceMode current_mode_;
  DeviceMode target_mode_;
  bool enabled_{false};
  bool initialized_{false};
  bool is_traction_controller_{false};
  bool has_statusword_{false};
  unsigned short statusword_{0};
  unsigned short traction_pdo_controlword_{0};
  int traction_pdo_target_velocity_{0};
  int error_{0};
  int error_code_{0};
  std::map<ParameterIds, double> feedbacks_;
  FeedbackFreshness feedback_freshness_;
  std::map<DriveLimits, double> limits_;
  double resolution_;
  double gear_ratio_;
};

#endif // COMMUNICATION_H
