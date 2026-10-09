#include <mobotic_driver/motion_source_gate.hpp>
#include <iostream>
#include <stdexcept>

using Gate = MotionSourceGate;
using Source = Gate::Source;
constexpr int64_t SECOND = 1'000'000'000;
const auto START = Gate::Clock::time_point{} + std::chrono::seconds(10);

void check(bool condition, const char* text)
{
  if (!condition) { throw std::runtime_error(text); }
}

int main()
{
  try
  {
    Gate gate;
    check(!gate.permits(Source::Kinematics, 10 * SECOND, 10 * SECOND, .3, START), "startup must reject motion");
    check(!gate.permits(Source::SupervisorDirect, 10 * SECOND, 10 * SECOND, .3, START), "startup rejects direct too");
    for (uint8_t mode = 0; mode < 3; ++mode)
    {
      const auto stamp = (10 + mode) * SECOND;
      const auto steady = START + std::chrono::seconds(mode);
      check(gate.update(mode, false, stamp, stamp, .3, steady), "mode update accepted");
      check(gate.permits(Source::Kinematics, stamp, stamp, .3, steady) == (mode < 2), "kinematics only for 0/1");
      check(gate.permits(Source::SupervisorDirect, stamp, stamp, .3, steady) == (mode == 2), "direct only for 2");
      check(!gate.permits(mode == 2 ? Source::SupervisorDirect : Source::Kinematics,
                         stamp - 1, stamp, .3, steady), "pre-boundary command cannot resume");
    }
    std::cout << "PASS: mutually exclusive routing, startup and command boundaries\n";

    for (uint8_t from = 0; from < 3; ++from)
    {
      for (uint8_t to = 0; to < 3; ++to)
      {
        Gate transition;
        check(transition.update(from, false, 10 * SECOND, 10 * SECOND, .3, START), "initial mode");
        auto steady = START + std::chrono::milliseconds(10);
        auto stamp = 10 * SECOND + 10'000'000;
        check(transition.update(from, true, stamp, stamp, .3, steady), "transition starts in old mode");
        check(transition.permits(Source::Kinematics, stamp, stamp, .3, steady) == (from < 2), "only old stop path");
        check(transition.permits(Source::SupervisorDirect, stamp, stamp, .3, steady) == (from == 2), "only old direct stop path");
        const auto epoch = transition.generation();
        steady += std::chrono::milliseconds(10);
        stamp += 10'000'000;
        check(transition.update(from, true, stamp, stamp, .3, steady), "heartbeat renews old mode");
        check(transition.generation() == epoch, "heartbeat must not move command boundary");
        steady += std::chrono::milliseconds(10);
        stamp += 10'000'000;
        check(transition.update(to, false, stamp, stamp, .3, steady), "completed switch");
        check(transition.generation() > epoch, "completion advances epoch, even same mode");
        check(!transition.permits(to == 2 ? Source::SupervisorDirect : Source::Kinematics,
                                  stamp - 1, stamp, .3, steady), "delayed old commands blocked");
        check(transition.permits(Source::Kinematics, stamp, stamp, .3, steady) == (to < 2), "new exclusive route");
        check(transition.permits(Source::SupervisorDirect, stamp, stamp, .3, steady) == (to == 2), "new exclusive direct route");
      }
    }
    std::cout << "PASS: all nine stop-path/completion pairs, including same-mode reselection\n";

    Gate ordered;
    check(ordered.update(2, false, 10 * SECOND, 10 * SECOND, .3, START), "ordered setup");
    check(!ordered.update(0, false, 10 * SECOND, 10 * SECOND, .3, START), "equal stamp replay rejected");
    check(!ordered.update(0, false, 9 * SECOND, 10 * SECOND, .3, START), "older state rejected");
    check(!ordered.update(255, false, 10 * SECOND + 1, 10 * SECOND + 1, .3, START), "unknown mode rejected");
    check(!ordered.update(0, false, 0, 10 * SECOND, .3, START), "zero stamp rejected");
    check(!ordered.update(0, false, 11 * SECOND, 10 * SECOND, .3, START), "future invalid state rejected");
    check(ordered.mode() == 2, "bad updates cannot change active mode");
    check(!ordered.fresh(10 * SECOND, .3, START + std::chrono::milliseconds(301)), "paused ROS time cannot preserve lease");
    check(!ordered.fresh(10 * SECOND + 301'000'000, .3, START), "source age also enforced");
    check(!ordered.fresh(9 * SECOND, .3, START), "backward ROS time fails closed");
    check(!ordered.fresh(10 * SECOND, .3, START - std::chrono::milliseconds(1)), "backward steady age fails closed");
    std::cout << "PASS: stale, replayed, malformed mode IDs and clock loss\n";

    auto recovered_at = START + std::chrono::milliseconds(500);
    check(ordered.update(2, false, 10 * SECOND + 400'000'000,
                         10 * SECOND + 500'000'000, .3, recovered_at), "delayed but fresh recovery");
    check(!ordered.permits(Source::SupervisorDirect, 10 * SECOND + 450'000'000,
                           10 * SECOND + 500'000'000, .3, recovered_at), "recovery does not restore buffered motion");
    check(ordered.permits(Source::SupervisorDirect, 10 * SECOND + 500'000'000,
                          10 * SECOND + 500'000'000, .3, recovered_at), "new recovery command accepted");
    check(!ordered.fresh(10 * SECOND + 710'000'000, .3,
                         recovered_at + std::chrono::milliseconds(210)), "delivery age backdated to source time");
    std::cout << "PASS: recovery epoch and original-age lease\n";
  }
  catch (const std::exception& error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  std::cout << "4 motion-source regression groups passed\n";
  return 0;
}
