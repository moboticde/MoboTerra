#ifndef VANGUARD_BATTERY_DRIVER_CHARGE_RATE_REQUEST_LOGIC_H
#define VANGUARD_BATTERY_DRIVER_CHARGE_RATE_REQUEST_LOGIC_H

#include <can_msgs/msg/frame.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace vanguard_battery_driver
{

// Vanguard "Customer Specified Charge Rate Message" (EFF3), Rev B (2023-04-17).
constexpr uint8_t CUSTOM_CHARGE_PRIORITY = 6;
constexpr uint32_t CUSTOM_CHARGE_PGN_AND_DESTINATION = 0xEFF3;
constexpr uint16_t BRIGGS_MANUFACTURER_ID = 982;
constexpr uint8_t CUSTOM_CHARGE_FEATURE_ID = 0xF0;
constexpr uint8_t DEFAULT_RAMP_CONTROL_VERSION = 1;
constexpr double CUSTOM_CHARGE_CURRENT_SCALE = 0.05;
constexpr double CUSTOM_CHARGE_CURRENT_OFFSET = -1600.0;
constexpr double CUSTOM_CHARGE_MAX_CURRENT = 1612.75;

inline uint16_t encodeCustomChargeCurrent(const float current_a)
{
  if (!std::isfinite(current_a) || current_a < 0.0F || current_a > CUSTOM_CHARGE_MAX_CURRENT)
    throw std::out_of_range("custom charge current is outside the documented range");
  return static_cast<uint16_t>(
      std::lround((current_a - CUSTOM_CHARGE_CURRENT_OFFSET) / CUSTOM_CHARGE_CURRENT_SCALE));
}

// A charge-rate request is a safety cap. Round down to the protocol's 0.05 A
// resolution so encoding cannot raise the request above a configured limit.
inline float quantizeCustomChargeCurrentDown(const double current_a)
{
  if (!std::isfinite(current_a) || current_a < 0.0 || current_a > CUSTOM_CHARGE_MAX_CURRENT)
    throw std::out_of_range("custom charge current is outside the documented range");
  double quantized_current_a =
      std::floor(current_a / CUSTOM_CHARGE_CURRENT_SCALE + 1e-9) * CUSTOM_CHARGE_CURRENT_SCALE;
  if (quantized_current_a > current_a + 1e-9)
    quantized_current_a -= CUSTOM_CHARGE_CURRENT_SCALE;
  return static_cast<float>(std::max(0.0, quantized_current_a));
}

inline can_msgs::msg::Frame createCustomChargeRateRequest(const float current_a,
                                                           const uint8_t source_address)
{
  const uint16_t raw_current = encodeCustomChargeCurrent(current_a);
  can_msgs::msg::Frame frame;
  frame.id = (static_cast<uint32_t>(CUSTOM_CHARGE_PRIORITY) << 26U) |
             (CUSTOM_CHARGE_PGN_AND_DESTINATION << 8U) | source_address;
  frame.is_extended = true;
  frame.is_rtr = false;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data = {static_cast<uint8_t>(BRIGGS_MANUFACTURER_ID & 0xFFU),
                static_cast<uint8_t>((BRIGGS_MANUFACTURER_ID >> 8U) & 0xFFU),
                CUSTOM_CHARGE_FEATURE_ID, static_cast<uint8_t>(raw_current & 0xFFU),
                static_cast<uint8_t>((raw_current >> 8U) & 0xFFU),
                DEFAULT_RAMP_CONTROL_VERSION, 0x00, 0x00};
  return frame;
}

// Charging starts at or below the lower threshold, remains enabled inside the
// band, and stops at or above the upper threshold. Invalid inputs fail closed.
inline bool updateSocChargeHysteresis(const bool charging_allowed, const double soc_percent,
                                      const double start_threshold_percent,
                                      const double stop_threshold_percent)
{
  if (!std::isfinite(soc_percent) || !std::isfinite(start_threshold_percent) ||
      !std::isfinite(stop_threshold_percent) || soc_percent < 0.0 || soc_percent > 100.0 ||
      start_threshold_percent < 0.0 || start_threshold_percent >= stop_threshold_percent ||
      stop_threshold_percent > 100.0)
    return false;
  if (soc_percent <= start_threshold_percent)
    return true;
  if (soc_percent >= stop_threshold_percent)
    return false;
  return charging_allowed;
}

struct SocChargePolicyInputs
{
  bool enabled{false};
  bool soc_fresh{false};
  bool soc_hysteresis_allows_charging{false};
  double soc_percent{0.0};
  double soc_start_threshold_percent{50.0};
  double soc_stop_threshold_percent{80.0};
  double system_capacity_ah{0.0};
  double maximum_c_rate{0.5};
  double absolute_current_limit_a{60.0};
};

struct ChargeRateDecision
{
  float requested_current_a{0.0F};
  std::string reason;
};

inline ChargeRateDecision socChargeRateDecision(const SocChargePolicyInputs& input)
{
  if (!input.enabled)
    return {0.0F, "controller_disabled"};
  if (!input.soc_fresh)
    return {0.0F, "battery_soc_stale"};
  if (!std::isfinite(input.soc_percent) || input.soc_percent < 0.0 || input.soc_percent > 100.0)
    return {0.0F, "invalid_battery_soc"};
  if (!std::isfinite(input.soc_start_threshold_percent) ||
      !std::isfinite(input.soc_stop_threshold_percent) ||
      input.soc_start_threshold_percent < 0.0 ||
      input.soc_start_threshold_percent >= input.soc_stop_threshold_percent ||
      input.soc_stop_threshold_percent > 100.0)
    return {0.0F, "invalid_soc_thresholds"};
  if (!std::isfinite(input.system_capacity_ah) || input.system_capacity_ah <= 0.0)
    return {0.0F, "system_capacity_not_configured"};
  if (!std::isfinite(input.maximum_c_rate) || input.maximum_c_rate <= 0.0 ||
      input.maximum_c_rate > 0.5)
    return {0.0F, "invalid_c_rate"};
  if (!std::isfinite(input.absolute_current_limit_a) || input.absolute_current_limit_a <= 0.0)
    return {0.0F, "invalid_absolute_limit"};
  if (input.soc_percent >= input.soc_stop_threshold_percent)
    return {0.0F, "soc_at_or_above_stop_threshold"};
  if (!input.soc_hysteresis_allows_charging)
    return {0.0F, "soc_waiting_for_start_threshold"};

  const double requested_current_a =
      std::min({input.system_capacity_ah * input.maximum_c_rate,
                input.absolute_current_limit_a, CUSTOM_CHARGE_MAX_CURRENT});
  return {quantizeCustomChargeCurrentDown(requested_current_a), "soc_charge_requested"};
}

}  // namespace vanguard_battery_driver

#endif  // VANGUARD_BATTERY_DRIVER_CHARGE_RATE_REQUEST_LOGIC_H
