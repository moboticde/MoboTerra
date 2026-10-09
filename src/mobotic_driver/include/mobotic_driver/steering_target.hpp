#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

// CAN absolute position is signed 32-bit encoder ticks. Check in floating point
// BEFORE converting to an integer; lround's native long width is not the wire range.
inline std::optional<int32_t> checkedCanPosition(double ticks)
{
  if (!std::isfinite(ticks)) { return std::nullopt; }
  const double rounded = std::round(ticks); // Halfway values round away from zero.
  if (rounded < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      rounded > static_cast<double>(std::numeric_limits<int32_t>::max()))
  {
    return std::nullopt;
  }
  return static_cast<int32_t>(rounded);
}

inline std::optional<int32_t> checkedSteeringPosition(double angle, double resolution)
{
  if (!std::isfinite(angle) || !std::isfinite(resolution) || resolution <= 0.0)
  {
    return std::nullopt;
  }
  constexpr double PI = 3.14159265358979323846;
  return checkedCanPosition(angle * resolution / (2.0 * PI));
}
