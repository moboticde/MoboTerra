#include <mobotic_kinematics/joint_feedback_cache.hpp>

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using Clock = JointFeedbackCache::Clock;
constexpr int64_t source_start = 10'000'000'000;
const auto receipt_start = Clock::time_point{} + std::chrono::seconds(10);
const std::vector<std::string> steering_names{
    "front_left_steering", "front_right_steering", "rear_left_steering", "rear_right_steering"};
const std::vector<std::string> traction_names{
    "front_left_traction", "front_right_traction", "rear_left_traction", "rear_right_traction"};

void check(bool condition, const char* description)
{
  if (!condition) { throw std::runtime_error(description); }
}

struct Sample
{
  std::vector<std::string> names;
  std::vector<double> positions, velocities;
  Sample()
  {
    for (std::size_t i = 0; i < steering_names.size(); ++i)
    {
      names.push_back(steering_names[i]); names.push_back(traction_names[i]);
      positions.push_back(0.1 * (i + 1)); positions.push_back(0.0);
      velocities.push_back(0.0); velocities.push_back(1.0 + i);
    }
  }
};

bool update(JointFeedbackCache& cache, const Sample& sample, int64_t stamp = source_start,
            int64_t now = source_start, Clock::time_point received_at = receipt_start)
{
  return cache.update(sample.names, sample.positions, sample.velocities, stamp, now,
      [](const auto&, const auto&) { return true; }, received_at);
}

JointFeedbackCache makeCache()
{
  return JointFeedbackCache(steering_names, traction_names, 0.3);
}

void startupAndCompleteFeedback()
{
  auto cache = makeCache();
  check(!cache.fresh(source_start, receipt_start), "startup must not have feedback");
  check(update(cache, Sample{}), "complete feedback must be accepted");
  check(cache.fresh(source_start, receipt_start), "accepted feedback must be fresh");
  check(cache.steeringPositions() == std::vector<double>({0.1, 0.2, 0.1 * 3, 0.4}),
        "all four steering positions must be cached");
}

void sourceExpiryAndDelayedAcquisition()
{
  auto cache = makeCache();
  check(update(cache, Sample{}, source_start, source_start + 290'000'000), "290 ms old sample is usable");
  check(cache.fresh(source_start + 299'000'000, receipt_start + std::chrono::milliseconds(9)),
        "delayed sample has a short remaining budget");
  check(!cache.fresh(source_start + 301'000'000, receipt_start + std::chrono::milliseconds(11)),
        "receipt must not give a delayed sample another full timeout");
}

void steadyExpiryWithPausedRosClock()
{
  auto cache = makeCache();
  check(update(cache, Sample{}), "setup feedback");
  check(cache.fresh(source_start, receipt_start + std::chrono::milliseconds(300)), "timeout boundary is inclusive");
  check(!cache.fresh(source_start, receipt_start + std::chrono::milliseconds(301)),
        "paused ROS time must not extend receipt freshness");
  check(!cache.fresh(source_start, receipt_start - std::chrono::milliseconds(1)),
        "negative receipt age is invalid");
}

void invalidTimestamps()
{
  for (const int64_t stamp : {int64_t{-1}, int64_t{0}, source_start - 301'000'000, source_start + 101'000'000})
  {
    auto cache = makeCache();
    check(!update(cache, Sample{}, stamp), "zero/stale/future sample must be rejected");
    check(!cache.fresh(source_start, receipt_start), "invalid stamp must not initialize feedback");
  }
  auto cache = makeCache();
  check(update(cache, Sample{}, source_start + 100'000'000), "100 ms clock skew is tolerated");
  check(!cache.fresh(source_start - 1, receipt_start), "larger backward-clock skew must block feedback");
}

void duplicateAndReorderedSamples()
{
  auto cache = makeCache();
  Sample changed;
  check(update(cache, changed), "setup feedback");
  const auto old_positions = cache.steeringPositions();
  changed.positions[0] = 9.0;
  check(!update(cache, changed, source_start, source_start + 200'000'000,
                receipt_start + std::chrono::milliseconds(200)), "duplicate must be ignored");
  check(!update(cache, changed, source_start - 1, source_start + 210'000'000,
                receipt_start + std::chrono::milliseconds(210)), "reordered sample must be ignored");
  check(cache.steeringPositions() == old_positions, "replay must not overwrite newer positions");
  check(cache.fresh(source_start + 250'000'000, receipt_start + std::chrono::milliseconds(250)),
        "ignored replay does not invalidate a still-fresh newer sample");
  check(!cache.fresh(source_start + 301'000'000, receipt_start + std::chrono::milliseconds(301)),
        "replay must not extend the cache lifetime");
  check(!update(cache, changed, source_start, source_start + 301'000'000,
                receipt_start + std::chrono::milliseconds(301)), "replay cannot recover expired feedback");
  check(!cache.fresh(source_start + 250'000'000, receipt_start + std::chrono::milliseconds(302)),
        "expiry observed on replay must leave cache invalid");
}

void malformedMessagesAreAtomic()
{
  std::vector<std::function<void(Sample&)>> corruptions{
      [](Sample& s) { s.names.back() = "unknown_joint"; },
      [](Sample& s) { s.names.back() = s.names.front(); },
      [](Sample& s) { s.names.back().clear(); },
      [](Sample& s) { s.positions.pop_back(); },
      [](Sample& s) { s.velocities.pop_back(); },
      [](Sample& s) { s.positions.back() = std::numeric_limits<double>::quiet_NaN(); },
      [](Sample& s) { s.velocities.back() = std::numeric_limits<double>::infinity(); },
      [](Sample& s) { s.names.clear(); s.positions.clear(); s.velocities.clear(); },
  };
  for (const auto& corrupt : corruptions)
  {
    auto cache = makeCache();
    check(update(cache, Sample{}), "setup feedback");
    const auto old_positions = cache.steeringPositions();
    Sample malformed;
    malformed.positions[0] = 9.0; // Earlier modules must not be committed before a later failure.
    corrupt(malformed);
    check(!update(cache, malformed, source_start + 1, source_start + 1), "malformed sample must be rejected");
    check(!cache.fresh(source_start + 1, receipt_start), "malformed newer sample must invalidate availability");
    check(cache.steeringPositions() == old_positions, "malformed message must not partially mutate positions");
    check(!update(cache, Sample{}), "old accepted timestamp cannot recover invalidated feedback");
    check(update(cache, Sample{}, source_start + 2, source_start + 2), "new complete sample recovers feedback");
  }
}

void derivedValidationIsAtomic()
{
  auto cache = makeCache();
  check(update(cache, Sample{}), "setup feedback");
  const auto old_positions = cache.steeringPositions();
  Sample changed;
  changed.positions[0] = 9.0;
  check(!cache.update(changed.names, changed.positions, changed.velocities,
                      source_start + 1, source_start + 1,
                      [](const auto&, const auto&) { return false; }, receipt_start),
        "failed derived kinematics must reject the entire sample");
  check(cache.steeringPositions() == old_positions, "failed derived values cannot mutate cache");
  check(!cache.fresh(source_start + 1, receipt_start), "failed derived values block commands");
}

void reorderedNamesAndAdditionalJoints()
{
  auto cache = makeCache();
  Sample sample;
  std::reverse(sample.names.begin(), sample.names.end());
  std::reverse(sample.positions.begin(), sample.positions.end());
  std::reverse(sample.velocities.begin(), sample.velocities.end());
  sample.names.push_back("other_joint"); sample.positions.push_back(0.0); sample.velocities.push_back(0.0);
  check(cache.update(sample.names, sample.positions, sample.velocities, source_start, source_start,
      [](const auto& positions, const auto& velocities) {
        return positions == std::vector<double>({0.1, 0.2, 0.1 * 3, 0.4}) &&
               velocities == std::vector<double>({1.0, 2.0, 3.0, 4.0});
      }, receipt_start), "joint names, not message ordering, define module mapping");
}

void explicitInvalidationAndRecovery()
{
  auto cache = makeCache();
  check(update(cache, Sample{}), "setup feedback");
  cache.invalidate();
  check(!cache.fresh(source_start, receipt_start), "explicit invalidation must block commands");
  check(!update(cache, Sample{}), "replay cannot recover invalidation");
  check(update(cache, Sample{}, source_start + 1, source_start + 1), "new complete feedback recovers");
}

void invalidTimeouts()
{
  for (const double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()})
  {
    bool threw = false;
    try { JointFeedbackCache cache(steering_names, traction_names, timeout); }
    catch (const std::invalid_argument&) { threw = true; }
    check(threw, "invalid feedback timeout must prevent startup");
  }
}
} // namespace

int main()
{
  const std::vector<std::pair<const char*, std::function<void()>>> cases{
      {"startup and complete feedback", startupAndCompleteFeedback},
      {"acquisition age and delayed samples", sourceExpiryAndDelayedAcquisition},
      {"steady age and paused ROS clock", steadyExpiryWithPausedRosClock},
      {"invalid timestamps and clock skew", invalidTimestamps},
      {"duplicates and reordered samples", duplicateAndReorderedSamples},
      {"malformed samples and atomic recovery", malformedMessagesAreAtomic},
      {"forward-kinematics validation is atomic", derivedValidationIsAtomic},
      {"joint order and additional joints", reorderedNamesAndAdditionalJoints},
      {"invalidation and recovery", explicitInvalidationAndRecovery},
      {"invalid timeout parameters", invalidTimeouts},
  };
  for (const auto& test : cases)
  {
    try { test.second(); }
    catch (const std::exception& error)
    {
      std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
      return 1;
    }
    std::cout << "PASS: " << test.first << '\n';
  }
  std::cout << cases.size() << " feedback-cache regression groups passed\n";
  return 0;
}
