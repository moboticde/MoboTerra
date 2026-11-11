#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <mobotic_driver/traction_position.hpp>
#include <functional>
#include <iostream>

using Frame = can_msgs::msg::Frame;
void check(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }
Frame request(unsigned node, unsigned index, unsigned sub = 0, unsigned size = 0, uint32_t value = 0)
{
  const auto command = size == 1 ? WRITE_TO_DEVICE_1B : size == 2 ? WRITE_TO_DEVICE_2B :
                       size == 4 ? WRITE_TO_DEVICE_4B : READ_FROM_DEVICE;
  auto frame = createAndFillMoboticHeader(0x600 + node, command, index, sub);
  uint32ToFrame(frame, 4, value); return frame;
}
Frame reply(VirtualDriveProtocol& drive, const Frame& request)
{
  const auto frames = drive.handle(request);
  check(frames.size() == 1, "one SDO response required"); return frames.front();
}
int read(VirtualDriveProtocol& drive, unsigned node, unsigned index, unsigned sub = 0)
{
  const auto frame = reply(drive, request(node, index, sub));
  check(isReadReplyFromDevice(frame), "read must succeed");
  return signedIntFromSdoReadReply(frame, 4);
}
void write(VirtualDriveProtocol& drive, unsigned node, unsigned index, unsigned sub, unsigned size, uint32_t value)
{ check(reply(drive, request(node, index, sub, size, value)).data[0] == WRITE_REPLY, "write must acknowledge"); }
MoboticDriveCanCommunication backend(unsigned node, bool traction)
{ return MoboticDriveCanCommunication("test", node, traction ? VELOCITY : POSITION, 4096,
    traction ? 16.0 : 121.0, {{MIN_STEERING_POSITION, -1e10}, {MAX_STEERING_POSITION, 1e10}}); }

void enable(VirtualDriveProtocol& drive, unsigned node, bool traction)
{
  auto driver = backend(node, traction);
  reply(drive, driver.setDeviceModeRequest(traction ? VELOCITY : POSITION));
  for (const auto& frame : driver.changeDeviceStateRequests(ENABLE)) { reply(drive, frame); }
  check(drive.enabled(), "production enable sequence must enable simulator");
}
void initialization()
{
  for (bool traction : {false, true})
  {
    auto driver = backend(traction ? 2 : 1, traction);
    VirtualDriveProtocol drive(traction ? 2 : 1, traction);
    auto frames = driver.update(reply(drive, driver.readErrorRegisterRequest()));
    check(frames.size() == 2, "initialization must request mode write/read");
    for (const auto& frame : frames) { driver.update(reply(drive, frame)); }
    check(driver.initialized(), "production decoder must initialize from virtual replies");
    for (const auto& frame : driver.changeDeviceStateRequests(ENABLE)) { reply(drive, frame); }
    driver.update(reply(drive, driver.readDeviceStateRequest()));
    check(driver.enabled(), "production decoder must recognize virtual enabled state");
    for (const auto& frame : driver.changeDeviceStateRequests(DISABLE)) { reply(drive, frame); }
    driver.update(reply(drive, driver.readDeviceStateRequest()));
    check(!driver.enabled(), "production decoder must recognize disable");
  }
}
void steeringMotion()
{
  VirtualDriveProtocol drive(1, false); enable(drive, 1, false);
  write(drive, 1, 0x3300, 0, 4, 1210); // 10 output rev/min = 682.67 ticks/s.
  write(drive, 1, 0x3790, 0, 4, 1024);
  check(read(drive, 1, 0x3762) == 0, "position must not jump on command");
  drive.advance(0.5);
  check(read(drive, 1, 0x3762) == 341, "position must follow configured encoder/gearing");
  check(read(drive, 1, 0x3A04, 1) == 1210, "motion feedback must use motor rpm");
  drive.advance(2.0); drive.advance(0.01);
  check(read(drive, 1, 0x3762) == 1024 && read(drive, 1, 0x3A04, 1) == 0, "reached target must be stationary");
  write(drive, 1, 0x3790, 0, 4, static_cast<uint32_t>(-1024));
  drive.advance(0.5);
  check(read(drive, 1, 0x3A04, 1) == -1210, "reverse steering velocity must be signed");
  const auto position = read(drive, 1, 0x3762);
  write(drive, 1, 0x3300, 0, 4, 0); drive.advance(0.1);
  check(read(drive, 1, 0x3762) != position, "a profile write alone must not change an active move");
  const auto stopped_position = read(drive, 1, 0x3762);
  write(drive, 1, 0x3000, 0, 4, 5); drive.advance(1.0);
  check(read(drive, 1, 0x3762) == stopped_position && read(drive, 1, 0x3A04, 1) == 0, "zero profile speed must hold position");
}
void heldSteeringInInstalledUserUnits()
{
  constexpr int units_per_turn = 121;
  constexpr double profile_rad_s = 0.3;
  for (const int home_turns : {-2, 0, 3})
  {
    for (const int direction : {-1, 1})
    {
      const int home = home_turns * units_per_turn;
      VirtualDriveProtocol controller(1, false, units_per_turn, 121.0, "micontrol");
      controller.setInitialPosition(home);
      MoboticDriveCanCommunication driver("front_left_steering", 1, POSITION, units_per_turn, 121.0,
          {{MIN_STEERING_POSITION, -1e10}, {MAX_STEERING_POSITION, 1e10},
           {MIN_STEERING_VELOCITY, -5000}, {MAX_STEERING_VELOCITY, 5000}}, "micontrol");
      reply(controller, driver.setDeviceModeRequest(POSITION));
      for (const auto& frame : driver.changeDeviceStateRequests(ENABLE)) { reply(controller, frame); }
      const double angle = home_turns * 2 * M_PI + direction * M_PI / 2;
      const int target = driver.getTarget(ABSOLUTE_POSITION_INPUT, angle);
      check(target == home + direction * 30, "quarter turn must use 30 native user units, never 1024 encoder increments");
      const int rpm = driver.getTarget(STEERING_PROFILE_VELOCITY_INPUT, profile_rad_s);
      for (unsigned tick = 0; tick < 1000; ++tick)
      {
        // Repeated held commands are absolute moves to the same native target.
        reply(controller, driver.setTargetRequest(STEERING_PROFILE_VELOCITY_INPUT, rpm));
        reply(controller, driver.setTargetRequest(ABSOLUTE_POSITION_INPUT, target));
        controller.advance(0.02);
        const int actual = read(controller, 1, POSITION_FEEDBACK);
        check(std::abs(actual - home) <= 30, "held steering must never accumulate extra turns");
      }
      check(read(controller, 1, POSITION_FEEDBACK) == target, "held steering reaches its absolute target");
      check(read(controller, 1, MICONTROL_VELOCITY_FEEDBACK, 1) == 0, "held steering remains stationary after arrival");
      driver.update(reply(controller, driver.readFeedbackRequest(POSITION_FEEDBACK)));
      check(std::abs(ticksToAngle(driver.feedback(POSITION_FEEDBACK), units_per_turn) - angle) < 2*M_PI/units_per_turn,
            "position feedback and target use the same native user-unit scale");

      // Neutral must also stop an unfinished move, using the production Update.
      reply(controller, driver.setTargetRequest(ABSOLUTE_POSITION_INPUT, home));
      controller.advance(0.2);
      reply(controller, driver.setTargetRequest(STEERING_PROFILE_VELOCITY_INPUT, 0));
      reply(controller, driver.steeringProfileUpdateRequest());
      const int stopped = read(controller, 1, POSITION_FEEDBACK);
      controller.advance(10.0);
      check(read(controller, 1, POSITION_FEEDBACK) == stopped, "neutral holds the current angle without recentering");
      check(read(controller, 1, MICONTROL_VELOCITY_FEEDBACK, 1) == 0, "neutral latches zero profile speed");
    }
  }
}
void micontrolTractionPositionScaling()
{
  VirtualDriveProtocol drive(2, true, 4096, 16, "micontrol");
  MoboticDriveCanCommunication decoder("front_right_traction", 2, VELOCITY, 4096, 16,
      {}, "micontrol");
  write(drive, 2, 0x3003, 0, 4, 3);
  write(drive, 2, 0x3004, 0, 4, 1);
  write(drive, 2, 0x3300, 0, 4, 60);  // One motor revolution/second.
  drive.advance(0.25);
  const auto frame = reply(drive, decoder.readFeedbackRequest(POSITION_FEEDBACK));
  decoder.update(frame);
  check(decoder.feedback(POSITION_FEEDBACK) == 1024,
        "miControl traction position is motor encoder counts");
  TractionPosition position;
  check(position.update(0, 2000) && position.update(1024, 2000), "motor counts unwrap");
  const double angle = position.radians(decoder.positionCountsPerOutputRevolution(), 1.0);
  check(std::abs(angle - M_PI / 32.0) < 1e-9, "traction position includes 16:1 reduction");
  decoder.update(reply(drive, decoder.readFeedbackRequest(VELOCITY_FEEDBACK)));
  check(std::abs(angle / 0.25 - decoder.velocityFeedbackRadPerSec()) < 1e-9,
        "position derivative and motor-rpm velocity agree");
  // Saved first-failure sample: node 4, 80 -> -107 in 156 ms. With the
  // physical 0.3 m/s / 0.325 m limit, the old unscaled guard allowed 182.5 counts.
  const double bound = (0.3 / 0.325) * decoder.positionCountsPerOutputRevolution()
      / (2 * M_PI) * 0.3 + 2;
  TractionPosition recorded;
  check(recorded.update(80, bound) && recorded.update(-107, bound),
        "recorded normal reverse movement must preserve position continuity");
  check(!recorded.update(1000000, bound), "true encoder discontinuities remain blocked");
}
void tractionSdo()
{
  VirtualDriveProtocol drive(2, true); enable(drive, 2, true);
  write(drive, 2, 0x3300, 0, 4, static_cast<uint32_t>(-1234));
  auto frame = reply(drive, request(2, 0x3A04, 1));
  check(frame.data[0] == READ_REPLY_4B && intFromFrame(frame, 4) == -1234, "signed motor-rpm traction feedback");
  write(drive, 2, 0x3200, 0, 4, static_cast<uint32_t>(-2345));
  frame = reply(drive, request(2, 0x3262, 1));
  check(frame.data[0] == READ_REPLY_4B && intFromFrame(frame, 4) == -2345, "signed four-byte current telemetry");
  check(read(drive, 2, 0x3262) == 2345, "legacy current channel reports magnitude in mA");
  write(drive, 2, 0x3004, 0, 4, 0);
  check(read(drive, 2, 0x3A04, 1) == 0 && read(drive, 2, 0x3262) == 0, "disabled motion and current remain zero");
  }

void faultsAndSafetyUnknown()
{
  VirtualDriveProtocol drive(2, true); enable(drive, 2, true); drive.setFault(1);
  check(!drive.enabled() && read(drive, 2, 0x3001) == 1, "fault inhibits enable and remains readable");
  write(drive, 2, 0x3004, 0, 4, 1);
  check(!drive.enabled(), "software enable cannot bypass a fault");
  write(drive, 2, 0x3000, 0, 4, 1); enable(drive, 2, true);
  check(read(drive, 2, 0x3001) == 0 && drive.enabled(), "explicit reset permits a new enable");
  for (auto index : {0x2086, 0x60FD, 0x6040, 0x606C, 0x1400, 0x1A00})
  { check(reply(drive, request(2, index)).data[0] == ABORT_REPLY, "unsupported objects cannot fabricate healthy telemetry"); }
  }
void disableNoResume()
{
  for (bool traction : {false, true})
  {
    const unsigned node = traction ? 2 : 1; VirtualDriveProtocol drive(node, traction); enable(drive, node, traction);
    write(drive, node, 0x3300, 0, 4, 1210);
    if (!traction) { write(drive, node, 0x3790, 0, 4, 1000); drive.advance(0.5); }
    auto driver = backend(node, traction); reply(drive, driver.changeDeviceStateRequest(DISABLE));
    reply(drive, driver.setTargetRequest(traction ? VELOCITY_INPUT : ABSOLUTE_POSITION_INPUT, 500));
    enable(drive, node, traction); const auto before = traction ? 0 : read(drive, node, 0x3762); drive.advance(1);
    check(read(drive, node, 0x3A04, 1) == 0, "re-enable must not revive old target");
    if (!traction) { check(read(drive, node, 0x3762) == before, "re-enable must hold actual steering position"); }
  }
}
void malformedAndIsolation()
{
  VirtualDriveProtocol drive(2, true);
  for (auto frame : {request(3, 0x3004, 0, 4, 1), request(2, 0x3004, 0, 4, 1)})
  {
    if (frame.id == 0x602) { frame.dlc = 7; }
    check(drive.handle(frame).empty() && !drive.enabled(), "wrong node/truncated frame ignored");
  }
  for (unsigned flag = 0; flag < 3; ++flag)
  {
    auto frame = request(2, 0x3004, 0, 4, 1);
    frame.is_error = flag == 0; frame.is_extended = flag == 1; frame.is_rtr = flag == 2;
    check(drive.handle(frame).empty() && !drive.enabled(), "flagged frame must not mutate drive");
  }
  check(reply(drive, request(2, 0x3004, 0, 2, 1)).data[0] == ABORT_REPLY, "wrong write width aborted");
  check(reply(drive, request(2, 0x3004, 0, 2, 1)).data[0] == ABORT_REPLY, "wrong controller protocol aborted");
  check(reply(drive, request(2, 0x3002, 0, 4, 1)).data[0] == ABORT_REPLY, "read-only write aborted");
  check(reply(drive, request(2, 0x3A04, 0)).data[0] == ABORT_REPLY, "wrong subindex aborted");
}

void configurationValidation()
{
  for (unsigned id : {0u, 128u})
  {
    bool threw = false; try { VirtualDriveProtocol drive(id, true); } catch (const std::invalid_argument&) { threw = true; }
    check(threw, "CANopen node range validated");
  }
  for (double scaling : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    bool threw = false; try { VirtualDriveProtocol drive(1, false, scaling); } catch (const std::invalid_argument&) { threw = true; }
    check(threw, "encoder scaling validated");
  }
}
void allEightDriveReadiness()
{
  for (unsigned node = 1; node <= 8; ++node)
  {
    const bool traction = node % 2 == 0;
    auto driver = backend(node, traction); VirtualDriveProtocol drive(node, traction);
    enable(drive, node, traction);
    for (const auto& request : {driver.readDeviceStateRequest(), driver.readErrorRegisterRequest(),
         driver.readFeedbackRequest(CURRENT_FEEDBACK),
         driver.readFeedbackRequest(VELOCITY_FEEDBACK), driver.readFeedbackRequest(POSITION_FEEDBACK)})
    { driver.update(reply(drive, request)); }
    check(driver.feedbackFreshness().ready(0.3, 0.3, 2.5), "every virtual drive must provide production required feedback fields");
    check(driver.enabled() && !driver.hasError(), "each virtual unit is enabled and fault-free");
  }
}
void currentModeProtocol()
{
  for (bool traction : {false, true})
  {
    const unsigned node = traction ? 2 : 1;
    VirtualDriveProtocol drive(node, traction);
    auto driver = MoboticDriveCanCommunication("test", node, CURRENT, 4096, 16.0, {});
    // Steering is identified by its configured position role in the real driver;
    // use its actual MiControl requests rather than the traction backend above.
    if (traction)
    {
      reply(drive, driver.setDeviceModeRequest(CURRENT));
      for (const auto& frame : driver.changeDeviceStateRequests(ENABLE)) { reply(drive, frame); }
    }
    else { write(drive, node, 0x3003, 0, 4, CURRENT); write(drive, node, 0x3004, 0, 4, 1); }
    write(drive, node, 0x3200, 0, 4, static_cast<uint32_t>(-1000));
    const auto frame = reply(drive, request(node, 0x3262, 1));
    check(signedIntFromSdoReadReply(frame, 4) == -1000, "repository current command convention must have signed feedback");
    check(read(drive, node, 0x3A04, 1) == 0, "current mode must not invent velocity dynamics");
  }
}
int main()
{
  const std::vector<std::pair<const char*, std::function<void()>>> cases{
    {"production initialization and enable/disable", initialization}, {"rate-limited steering motion", steeringMotion},
    {"held steering and neutral stop at installed 121-unit scale", heldSteeringInInstalledUserUnits},
    {"miControl traction position and velocity scaling", micontrolTractionPositionScaling},
    {"miControl SDO velocity/current widths", tractionSdo},
    {"fault reset and unknown safety registers", faultsAndSafetyUnknown},
    {"disable/re-enable does not revive targets", disableNoResume}, {"malformed frames and controller isolation", malformedAndIsolation},
    {"configuration validation", configurationValidation},
    {"all eight drives satisfy production feedback readiness", allEightDriveReadiness},
    {"current-mode protocol convention without invented dynamics", currentModeProtocol},
  };
  for (const auto& test : cases)
  {
    try { test.second(); } catch (const std::exception& error)
    { std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n'; return 1; }
    std::cout << "PASS: " << test.first << '\n';
  }
  std::cout << cases.size() << " virtual protocol regression groups passed\n";
}
