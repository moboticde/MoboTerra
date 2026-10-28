#ifndef VANGUARD_BATTERY_DRIVER_BATTERY_TELEMETRY_LOGIC_H
#define VANGUARD_BATTERY_DRIVER_BATTERY_TELEMETRY_LOGIC_H

#include "vanguard_battery_driver/charge_telemetry_logic.h"

#include <can_msgs/msg/frame.hpp>

#include <cstdint>
#include <optional>

namespace vanguard_battery_driver
{

// Default parallel battery controller identifiers used by this machine.
constexpr uint8_t DEFAULT_BATTERY_SOURCE_ADDRESS = 0xF3;
constexpr uint32_t DEFAULT_BATTERY_SOC_PGN = 0xF091;
constexpr uint32_t DEFAULT_BATTERY_ELECTRICAL_PGN = 0xF090;
constexpr float BATTERY_SOC_SCALE = 0.001563F;
constexpr float BATTERY_VOLTAGE_SCALE = 0.05F;
constexpr float BATTERY_CURRENT_SCALE = 0.05F;
constexpr float BATTERY_CURRENT_OFFSET = -1600.0F;

struct BatterySoc
{
  float percent{0.0F};
};

struct BatteryElectrical
{
  float pack_voltage{0.0F};
  float pack_current{0.0F};
};

inline uint8_t j1939SourceAddress(const can_msgs::msg::Frame& frame)
{
  return static_cast<uint8_t>(frame.id & 0xFFU);
}

inline uint32_t j1939Pgn(const can_msgs::msg::Frame& frame)
{
  return (frame.id >> 8U) & 0x3FFFFU;
}

inline bool isBatteryFrame(const can_msgs::msg::Frame& frame, const uint8_t source_address,
                           const uint32_t pgn)
{
  return isValidJ1939DataFrame(frame) && j1939SourceAddress(frame) == source_address && j1939Pgn(frame) == pgn;
}

inline std::optional<BatterySoc> decodeBatterySoc(
    const can_msgs::msg::Frame& frame, const uint8_t source_address = DEFAULT_BATTERY_SOURCE_ADDRESS,
    const uint32_t pgn = DEFAULT_BATTERY_SOC_PGN)
{
  if (!isBatteryFrame(frame, source_address, pgn))
  {
    return std::nullopt;
  }

  const uint16_t raw_soc = decodeLittleEndianUint16(frame.data[0], frame.data[1]);
  if (raw_soc == 0xFFFFU)
  {
    return std::nullopt;
  }
  return BatterySoc{static_cast<float>(raw_soc) * BATTERY_SOC_SCALE};
}

inline std::optional<BatteryElectrical> decodeBatteryElectrical(
    const can_msgs::msg::Frame& frame, const uint8_t source_address = DEFAULT_BATTERY_SOURCE_ADDRESS,
    const uint32_t pgn = DEFAULT_BATTERY_ELECTRICAL_PGN)
{
  if (!isBatteryFrame(frame, source_address, pgn))
  {
    return std::nullopt;
  }

  const uint16_t raw_voltage = decodeLittleEndianUint16(frame.data[4], frame.data[5]);
  const uint16_t raw_current = decodeLittleEndianUint16(frame.data[6], frame.data[7]);
  if (raw_voltage == 0xFFFFU || raw_current == 0xFFFFU)
  {
    return std::nullopt;
  }
  return BatteryElectrical{static_cast<float>(raw_voltage) * BATTERY_VOLTAGE_SCALE,
                           static_cast<float>(raw_current) * BATTERY_CURRENT_SCALE + BATTERY_CURRENT_OFFSET};
}

}  // namespace vanguard_battery_driver

#endif  // VANGUARD_BATTERY_DRIVER_BATTERY_TELEMETRY_LOGIC_H
