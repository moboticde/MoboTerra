#pragma once
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>

// Continuous signed counter unwrapping. This alone cannot recover unpowered
// movement after a drive reset; hardware must supply a retained reference.
class TractionPosition
{
public:
  bool update(int32_t raw, double max_delta)
  {
    if (!std::isfinite(max_delta) || max_delta < 0 || max_delta >= 2147483648.0) { return false; }
    if (!received_) { raw_ = raw; count_ = raw; received_ = true; return true; }
    const uint32_t modular = static_cast<uint32_t>(raw) - static_cast<uint32_t>(raw_);
    const int64_t delta = modular <= INT32_MAX ? modular : static_cast<int64_t>(modular) - 4294967296LL;
    if (std::abs(static_cast<double>(delta)) > max_delta ||
        (delta > 0 && count_ > INT64_MAX - delta) || (delta < 0 && count_ < INT64_MIN - delta)) { return false; }
    count_ += delta; raw_ = raw; return true;
  }
  int64_t count() const { return count_; }
  double radians(double resolution, double gearing) const
  { return static_cast<double>(count_) * 6.2831853071795864769 / (resolution * gearing); }
private:
  bool received_{false}; int32_t raw_{0}; int64_t count_{0};
};
