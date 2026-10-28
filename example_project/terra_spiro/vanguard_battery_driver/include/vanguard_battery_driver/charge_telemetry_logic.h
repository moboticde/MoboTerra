#ifndef VANGUARD_BATTERY_DRIVER_CHARGE_TELEMETRY_LOGIC_H
#define VANGUARD_BATTERY_DRIVER_CHARGE_TELEMETRY_LOGIC_H

#include <can_msgs/msg/frame.hpp>

#include <cstdint>
#include <optional>

namespace vanguard_battery_driver
{

// Documented in the Vanguard 3000 W Charger Installation Guide, REV C.
constexpr uint32_t BMS_CHARGE_REQUEST_ID = 0x18FF10F3;
constexpr uint32_t CHARGER_STATUS_ID = 0x0CFFBC49;
constexpr uint32_t CHARGER_LIMITS_ID = 0x0CFFBD49;
constexpr uint8_t CONSTANT_CURRENT_REQUEST = 0xFE;
constexpr float CHARGER_VOLTAGE_SCALE = 0.05F;
constexpr float CHARGER_CURRENT_SCALE = 0.05F;
constexpr float CHARGER_CURRENT_OFFSET = -1600.0F;
constexpr float CHARGER_POWER_SCALE = 0.05F;
constexpr float AC_VOLTAGE_SCALE = 0.5F;

inline bool isValidJ1939DataFrame(const can_msgs::msg::Frame& frame)
{
  return frame.is_extended && !frame.is_rtr && !frame.is_error && frame.dlc == 8;
}

inline uint16_t decodeLittleEndianUint16(const uint8_t low, const uint8_t high)
{
  return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8U);
}

inline float decodeChargerCurrent(const uint8_t low, const uint8_t high)
{
  return static_cast<float>(decodeLittleEndianUint16(low, high)) * CHARGER_CURRENT_SCALE +
         CHARGER_CURRENT_OFFSET;
}

struct BmsChargeRequest
{
  uint8_t mode{0};
  float requested_current{0.0F};
};

struct ChargerStatus
{
  float output_voltage{0.0F};
  float output_current{0.0F};
  uint8_t status{0};
  bool ac_connected{false};
  float ac_voltage{0.0F};
};

struct ChargerLimits
{
  float voltage_limit{0.0F};
  float current_limit{0.0F};
  float power_rating_kw{0.0F};
};

inline std::optional<BmsChargeRequest> decodeBmsChargeRequest(const can_msgs::msg::Frame& frame)
{
  if (!isValidJ1939DataFrame(frame) || frame.id != BMS_CHARGE_REQUEST_ID ||
      decodeLittleEndianUint16(frame.data[1], frame.data[2]) == 0xFFFFU)
  {
    return std::nullopt;
  }
  return BmsChargeRequest{frame.data[0], decodeChargerCurrent(frame.data[1], frame.data[2])};
}

inline std::optional<ChargerStatus> decodeChargerStatus(const can_msgs::msg::Frame& frame)
{
  if (!isValidJ1939DataFrame(frame) || frame.id != CHARGER_STATUS_ID)
  {
    return std::nullopt;
  }
  const uint16_t raw_output_voltage = decodeLittleEndianUint16(frame.data[0], frame.data[1]);
  const uint16_t raw_output_current = decodeLittleEndianUint16(frame.data[2], frame.data[3]);
  const uint16_t raw_ac_voltage = decodeLittleEndianUint16(frame.data[6], frame.data[7]);
  if (raw_output_voltage == 0xFFFFU || raw_output_current == 0xFFFFU || frame.data[4] == 0xFFU ||
      frame.data[5] > 1U || raw_ac_voltage == 0xFFFFU)
  {
    return std::nullopt;
  }
  return ChargerStatus{
      static_cast<float>(raw_output_voltage) * CHARGER_VOLTAGE_SCALE,
      decodeChargerCurrent(frame.data[2], frame.data[3]), frame.data[4], frame.data[5] == 1,
      static_cast<float>(raw_ac_voltage) * AC_VOLTAGE_SCALE};
}

inline std::optional<ChargerLimits> decodeChargerLimits(const can_msgs::msg::Frame& frame)
{
  if (!isValidJ1939DataFrame(frame) || frame.id != CHARGER_LIMITS_ID)
  {
    return std::nullopt;
  }
  const uint16_t raw_voltage_limit = decodeLittleEndianUint16(frame.data[0], frame.data[1]);
  const uint16_t raw_current_limit = decodeLittleEndianUint16(frame.data[2], frame.data[3]);
  const uint16_t raw_power_rating = decodeLittleEndianUint16(frame.data[4], frame.data[5]);
  if (raw_voltage_limit == 0xFFFFU || raw_current_limit == 0xFFFFU || raw_power_rating == 0xFFFFU)
  {
    return std::nullopt;
  }
  return ChargerLimits{
      static_cast<float>(raw_voltage_limit) * CHARGER_VOLTAGE_SCALE,
      decodeChargerCurrent(frame.data[2], frame.data[3]),
      static_cast<float>(raw_power_rating) * CHARGER_POWER_SCALE};
}

}  // namespace vanguard_battery_driver

#endif  // VANGUARD_BATTERY_DRIVER_CHARGE_TELEMETRY_LOGIC_H
