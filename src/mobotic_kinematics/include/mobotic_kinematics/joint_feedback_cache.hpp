#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// ROS-independent cache used by the node and its regression tests. Source time
// bounds acquisition age; steady time prevents a paused ROS clock extending it.
class JointFeedbackCache
{
public:
  using Clock = std::chrono::steady_clock;

  JointFeedbackCache(std::vector<std::string> steering_names,
                     std::vector<std::string> traction_names, double timeout)
      : steering_names_(std::move(steering_names)), traction_names_(std::move(traction_names)),
        timeout_(timeout), steering_positions_(steering_names_.size(), 0.0)
  {
    if (!std::isfinite(timeout_) || timeout_ <= 0.0 || steering_names_.empty() ||
        steering_names_.size() != traction_names_.size())
    {
      throw std::invalid_argument("joint feedback requires matching module names and a positive finite timeout");
    }
  }

  // Validate and compute derived values against temporary vectors. Nothing is
  // committed until every joint and the caller's forward-kinematics check pass.
  template<typename Validator>
  bool update(const std::vector<std::string>& names, const std::vector<double>& positions,
              const std::vector<double>& velocities, int64_t source_stamp_ns, int64_t now_ns,
              const Validator& validate, Clock::time_point received_at = Clock::now())
  {
    if (valid_ && !fresh(now_ns, received_at)) { invalidate(); }
    if (source_stamp_ns > 0 && source_stamp_ns <= last_source_stamp_ns_)
    {
      return false; // Duplicates/reordered samples cannot overwrite or renew the cache.
    }
    invalidate();
    if (!sourceFresh(source_stamp_ns, now_ns) || positions.size() != names.size() ||
        velocities.size() != names.size())
    {
      return false;
    }

    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      if (names[i].empty() || !indices.emplace(names[i], i).second ||
          !std::isfinite(positions[i]) || !std::isfinite(velocities[i]))
      {
        return false;
      }
    }

    std::vector<double> candidate_positions(steering_names_.size());
    std::vector<double> candidate_velocities(traction_names_.size());
    for (std::size_t i = 0; i < steering_names_.size(); ++i)
    {
      const auto steering = indices.find(steering_names_[i]);
      const auto traction = indices.find(traction_names_[i]);
      if (steering == indices.end() || traction == indices.end()) { return false; }
      candidate_positions[i] = positions[steering->second];
      candidate_velocities[i] = velocities[traction->second];
    }
    if (!validate(candidate_positions, candidate_velocities)) { return false; }

    steering_positions_.swap(candidate_positions);
    last_source_stamp_ns_ = source_stamp_ns;
    received_at_ = received_at;
    valid_ = true;
    return true;
  }

  bool fresh(int64_t now_ns, Clock::time_point now = Clock::now()) const
  {
    const double receipt_age = std::chrono::duration<double>(now - received_at_).count();
    return valid_ && sourceFresh(last_source_stamp_ns_, now_ns) &&
           receipt_age >= 0.0 && receipt_age <= timeout_;
  }

  void invalidate() { valid_ = false; }
  const std::vector<double>& steeringPositions() const { return steering_positions_; }

private:
  bool sourceFresh(int64_t stamp_ns, int64_t now_ns) const
  {
    if (stamp_ns <= 0) { return false; }
    const double source_age = std::chrono::duration<double>(
        std::chrono::nanoseconds(now_ns - stamp_ns)).count();
    // Same 100 ms future-clock tolerance as the existing command path.
    return source_age >= -0.1 && source_age <= timeout_;
  }

  std::vector<std::string> steering_names_, traction_names_;
  double timeout_;
  std::vector<double> steering_positions_;
  int64_t last_source_stamp_ns_{0};
  Clock::time_point received_at_{};
  bool valid_{false};
};
