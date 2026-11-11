#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

// CAN absolute position is signed 32-bit native user units (not necessarily raw
// encoder increments). Check in floating point
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

// B2 completion requires at most one degree, retaining stricter one-count
// precision on finer controllers. One count at 121 units/turn is 2.98 degrees
// and must still be corrected. This predicate never disables a drive.
inline bool steeringPositionWithinTolerance(double position, double target, double resolution)
{
  return std::isfinite(position) && std::isfinite(target) && std::isfinite(resolution) &&
      resolution > 0.0 && std::abs(position - target) <= std::min(1.0, resolution / 360.0) + 1e-12;
}

// Preserve the existing zero reference and select only full-turn equivalents.
// Freeze the result for an alignment attempt; half-turn ties favor the original
// zero. Reject an unrepresentable nearest home instead of choosing a longer move.
inline std::optional<int32_t> nearestSteeringHome(double position, double counts_per_turn)
{
  const auto current = checkedCanPosition(position);
  if (!current || !std::isfinite(counts_per_turn) || counts_per_turn < 1.0 ||
      counts_per_turn != std::floor(counts_per_turn) ||
      counts_per_turn > static_cast<double>(std::numeric_limits<int32_t>::max()))
  { return std::nullopt; }
  const int64_t period = static_cast<int64_t>(counts_per_turn);
  const int64_t ticks = *current;
  const int64_t remainder = ticks % period;
  int64_t target = (ticks / period) * period;
  const int64_t distance = remainder < 0 ? -remainder : remainder;
  if (2 * distance > period) { target += ticks > 0 ? period : -period; }
  if (target < std::numeric_limits<int32_t>::min() || target > std::numeric_limits<int32_t>::max())
  { return std::nullopt; }
  return static_cast<int32_t>(target);
}
