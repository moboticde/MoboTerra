#include <mobotic_driver/communication.hpp>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double MIN_TICKS = static_cast<double>(std::numeric_limits<int32_t>::min());
constexpr double MAX_TICKS = static_cast<double>(std::numeric_limits<int32_t>::max());

void check(bool condition, const char* message)
{
  if (!condition) { throw std::runtime_error(message); }
}

MoboticDriveCanCommunication drive(double resolution = 4096.0,
    double minimum = -std::numeric_limits<double>::infinity(),
    double maximum = std::numeric_limits<double>::infinity())
{
  return MoboticDriveCanCommunication("steering", 3, POSITION, resolution, 121.0,
      {{MIN_STEERING_POSITION, minimum}, {MAX_STEERING_POSITION, maximum}});
}

void signedWireBoundaries()
{
  check(checkedCanPosition(MIN_TICKS).value() == std::numeric_limits<int32_t>::min(), "signed minimum accepted");
  check(checkedCanPosition(MAX_TICKS).value() == std::numeric_limits<int32_t>::max(), "signed maximum accepted");
  check(checkedCanPosition(MIN_TICKS - 0.25).value() == std::numeric_limits<int32_t>::min(), "roundable lower fractional tick accepted");
  check(checkedCanPosition(MAX_TICKS + 0.25).value() == std::numeric_limits<int32_t>::max(), "roundable upper fractional tick accepted");
  check(!checkedCanPosition(MIN_TICKS - 0.5), "lower half-tick boundary rejected before cast");
  check(!checkedCanPosition(MAX_TICKS + 0.5), "upper half-tick boundary rejected before cast");
  check(checkedCanPosition(std::nextafter(MIN_TICKS - 0.5, MIN_TICKS)).has_value(), "immediately inside lower boundary accepted");
  check(checkedCanPosition(std::nextafter(MAX_TICKS + 0.5, MAX_TICKS)).has_value(), "immediately inside upper boundary accepted");
  check(!checkedCanPosition(MIN_TICKS - 1.0) && !checkedCanPosition(MAX_TICKS + 1.0), "out-of-wire-range ticks rejected");
}

void roundingAndMultiturn()
{
  for (const auto& item : {std::make_pair(0.0, 0), std::make_pair(0.49, 0), std::make_pair(-0.49, 0),
                          std::make_pair(0.5, 1), std::make_pair(-0.5, -1),
                          std::make_pair(1.5, 2), std::make_pair(-1.5, -2)})
  {
    check(checkedCanPosition(item.first).value() == item.second, "rounding must remain away from zero at halves");
  }
  auto steering = drive();
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, 0.0) == 0, "zero target unchanged");
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, PI / 2.0) == 1024, "normal quarter-turn unchanged");
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, -PI / 2.0) == -1024, "negative quarter-turn unchanged");
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, 20.0 * PI) == 40960, "multi-turn position must not be wrapped or clipped");
}

void encoderResolutionAndWireRoundtrip()
{
  for (const double resolution : {1.0, 4096.0, 8192.0, 1'000'000.0})
  {
    auto steering = drive(resolution);
    for (const double ticks : {MIN_TICKS, MAX_TICKS, -12345.0, 12345.0})
    {
      const double angle = ticksToAngle(ticks, resolution);
      check(steering.steeringPositionRepresentable(angle), "encoder endpoint angle must be representable");
      const auto target = steering.getTarget(ABSOLUTE_POSITION_INPUT, angle);
      check(target == static_cast<int32_t>(ticks), "resolution-specific tick conversion must preserve target");
      const auto frame = steering.setTargetRequest(ABSOLUTE_POSITION_INPUT, target);
      check(intFromFrame(frame, 4) == target, "signed endpoint must survive CAN encoding");
    }
    check(!steering.steeringPositionRepresentable(ticksToAngle(MAX_TICKS + 1.0, resolution)), "upper overflow rejected for every resolution");
    check(!steering.steeringPositionRepresentable(ticksToAngle(MIN_TICKS - 1.0, resolution)), "lower overflow rejected for every resolution");
  }
}

void invalidFloatingInputs()
{
  for (const double angle : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::max(),
                            -std::numeric_limits<double>::max(), 1e100, -1e100})
  {
    check(!checkedSteeringPosition(angle, 4096.0), "invalid or huge angle cannot reach integer conversion");
  }
  for (const double resolution : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
  {
    check(!checkedSteeringPosition(0.1, resolution), "invalid resolution cannot convert steering");
  }
  check(!checkedSteeringPosition(10.0, std::numeric_limits<double>::max()), "overflow in intermediate multiplication must be rejected");
}

void backendCannotBypassValidation()
{
  auto steering = drive();
  for (const double angle : {1e100, -1e100, std::numeric_limits<double>::quiet_NaN(),
                            ticksToAngle(MAX_TICKS + 1.0, 4096.0), ticksToAngle(MIN_TICKS - 1.0, 4096.0)})
  {
    check(!steering.steeringPositionRepresentable(angle), "preflight rejects unsafe target");
    bool threw = false;
    try { (void)steering.getTarget(ABSOLUTE_POSITION_INPUT, angle); }
    catch (const std::out_of_range&) { threw = true; }
    check(threw, "backend must defend against callers bypassing preflight");
  }
}

void existingExplicitTickLimits()
{
  auto steering = drive(4096.0, -1000.0, 1000.0);
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, ticksToAngle(2000.0, 4096.0)) == 1000,
        "existing explicit upper tick clamp remains supported");
  check(steering.getTarget(ABSOLUTE_POSITION_INPUT, ticksToAngle(-2000.0, 4096.0)) == -1000,
        "existing explicit lower tick clamp remains supported");
  bool threw = false;
  try { (void)steering.getTarget(ABSOLUTE_POSITION_INPUT, 1e100); }
  catch (const std::out_of_range&) { threw = true; }
  check(threw, "finite legacy clamp must not hide an unrepresentable raw command");
}
} // namespace

int main()
{
  const std::vector<std::pair<const char*, std::function<void()>>> cases{
      {"signed wire and half-tick boundaries", signedWireBoundaries},
      {"rounding and unchanged multi-turn commands", roundingAndMultiturn},
      {"encoder resolutions and signed CAN roundtrip", encoderResolutionAndWireRoundtrip},
      {"non-finite, huge and intermediate-overflow inputs", invalidFloatingInputs},
      {"preflight and backend defense", backendCannotBypassValidation},
      {"preserved explicit tick limits without overflow clipping", existingExplicitTickLimits},
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
  std::cout << cases.size() << " steering-target regression groups passed\n";
  return 0;
}
