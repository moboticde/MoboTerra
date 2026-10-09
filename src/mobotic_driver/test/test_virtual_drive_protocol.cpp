#include <mobotic_driver/virtual_drive_protocol.hpp>
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
void configure(VirtualDriveProtocol& drive, const std::vector<Frame>& requests)
{
  for (const auto& request : requests)
  {
    const auto frames = drive.handle(request);
    if (request.id != 0) { check(frames.size() == 1 && frames[0].data[0] == WRITE_REPLY, "production configuration must succeed"); }
  }
}
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
  write(drive, 1, 0x3300, 0, 4, 0); drive.advance(1.0);
  check(read(drive, 1, 0x3762) == position && read(drive, 1, 0x3A04, 1) == 0, "zero profile speed must hold position");
}
void tractionSdo()
{
  VirtualDriveProtocol drive(2, true); enable(drive, 2, true);
  write(drive, 2, 0x60FF, 0, 4, static_cast<uint32_t>(-123456));
  auto frame = reply(drive, request(2, 0x606C));
  check(frame.data[0] == READ_REPLY_4B && intFromFrame(frame, 4) == -123456, "signed traction SDO velocity");
  write(drive, 2, 0x3200, 0, 4, static_cast<uint32_t>(-2345));
  frame = reply(drive, request(2, 0x6078));
  check(frame.data[0] == READ_REPLY_2B && signedIntFromSdoReadReply(frame, 4) == -2345, "traction current uses signed two bytes");
  write(drive, 2, 0x6040, 0, 2, 0);
  check(read(drive, 2, 0x606C) == 0 && read(drive, 2, 0x6078) == 0, "disabled feedback remains available and zero");
}
void tractionPdo()
{
  auto driver = backend(2, true); VirtualDriveProtocol drive(2, true);
  configure(drive, driver.tractionPdoConfigurationRequests(1));
  reply(drive, driver.setDeviceModeRequest(VELOCITY));
  driver.setTractionPdoControlword(0xF); driver.setTractionPdoTarget(-76543);
  drive.handle(driver.tractionPdoCommandFrame());
  check(!drive.enabled() && read(drive, 2, 0x606C) == 0, "RPDO must wait for SYNC");
  const auto frames = drive.handle(MoboticDriveCanCommunication::syncFrame());
  check(frames.size() == 2 && frames[0].id == 0x182 && frames[0].dlc == 7, "ELMO motion PDO layout");
  check(shortFromFrame(frames[0], 0) == 0x27 && frames[0].data[2] == VELOCITY &&
        intFromFrame(frames[0], 3) == -76543, "ELMO PDO contents");
  driver.update(frames[0]); driver.update(frames[1]);
  check(driver.enabled() && driver.currentMode() == VELOCITY && driver.feedback(VELOCITY_FEEDBACK) == -76543,
        "production decoder accepts virtual PDOs");
  driver.setTractionPdoTarget(0); drive.handle(driver.tractionPdoCommandFrame());
  drive.handle(MoboticDriveCanCommunication::syncFrame());
  check(read(drive, 2, 0x606C) == 0, "watchdog-style zero RPDO stops traction");
  driver.setTractionPdoTarget(999); drive.handle(driver.tractionPdoCommandFrame());
  reply(drive, driver.changeDeviceStateRequest(DISABLE));
  drive.handle(MoboticDriveCanCommunication::syncFrame());
  check(!drive.enabled() && read(drive, 2, 0x606C) == 0, "disable must discard pending RPDO so SYNC cannot revive motion");
}
void steeringPdos()
{
  auto driver = backend(1, false); VirtualDriveProtocol drive(1, false);
  configure(drive, driver.steeringTelemetryPdoConfigurationRequests(2)); enable(drive, 1, false);
  write(drive, 1, 0x3300, 0, 4, 1210); write(drive, 1, 0x3790, 0, 4, 1000); drive.advance(0.5);
  check(drive.handle(MoboticDriveCanCommunication::syncFrame()).empty(), "SYNC divider respected");
  const auto frames = drive.handle(MoboticDriveCanCommunication::syncFrame());
  check(frames.size() == 3 && frames[0].dlc == 8 && frames[1].dlc == 8 && frames[2].dlc == 8, "steering telemetry PDO layouts");
  check(intFromFrame(frames[0], 0) == 341 && intFromFrame(frames[0], 4) == 1210, "steering motion PDO contents");
  for (const auto& frame : frames) { driver.update(frame); }
  check(driver.feedback(POSITION_FEEDBACK) == 341 && driver.feedback(VELOCITY_FEEDBACK) == 1210,
        "production decoder accepts steering PDOs");
}
void faultsAndSafetyUnknown()
{
  VirtualDriveProtocol traction(2, true); enable(traction, 2, true); traction.setFault(1, 0x1234);
  check(!traction.enabled() && read(traction, 2, 0x603F) == 0x1234, "fault inhibits enable and is readable");
  write(traction, 2, 0x6040, 0, 2, 0xF); check(!traction.enabled(), "fault cannot be bypassed by enable");
  write(traction, 2, 0x6040, 0, 2, 0x80); enable(traction, 2, true);
  check(read(traction, 2, 0x1001) == 0 && read(traction, 2, 0x603F) == 0, "reset clears fault register/code");
  for (auto index : {0x2086, 0x60FD})
  { check(reply(traction, request(2, index)).data[0] == ABORT_REPLY, "unknown safety telemetry must not be healthy fabricated data"); }
  VirtualDriveProtocol steering(1, false); steering.setFault(2, 0);
  write(steering, 1, 0x3000, 0, 2, 1); enable(steering, 1, false);
}
void disableNoResume()
{
  for (bool traction : {false, true})
  {
    const unsigned node = traction ? 2 : 1; VirtualDriveProtocol drive(node, traction); enable(drive, node, traction);
    write(drive, node, traction ? 0x60FF : 0x3300, 0, 4, 1210);
    if (!traction) { write(drive, node, 0x3790, 0, 4, 1000); drive.advance(0.5); }
    auto driver = backend(node, traction); reply(drive, driver.changeDeviceStateRequest(DISABLE));
    reply(drive, driver.setTargetRequest(traction ? EC_TRACTION_VELOCITY_INPUT : ABSOLUTE_POSITION_INPUT, 500));
    enable(drive, node, traction); const auto before = traction ? 0 : read(drive, node, 0x3762); drive.advance(1);
    check(read(drive, node, traction ? 0x606C : 0x3A04, traction ? 0 : 1) == 0, "re-enable must not revive old target");
    if (!traction) { check(read(drive, node, 0x3762) == before, "re-enable must hold actual steering position"); }
  }
}
void malformedAndIsolation()
{
  VirtualDriveProtocol drive(2, true);
  for (auto frame : {request(3, 0x6040, 0, 2, 15), request(2, 0x6040, 0, 2, 15)})
  {
    if (frame.id == 0x602) { frame.dlc = 7; }
    check(drive.handle(frame).empty() && !drive.enabled(), "wrong node/truncated frame ignored");
  }
  for (unsigned flag = 0; flag < 3; ++flag)
  {
    auto frame = request(2, 0x6040, 0, 2, 15);
    frame.is_error = flag == 0; frame.is_extended = flag == 1; frame.is_rtr = flag == 2;
    check(drive.handle(frame).empty() && !drive.enabled(), "flagged frame must not mutate drive");
  }
  check(reply(drive, request(2, 0x6040, 0, 4, 15)).data[0] == ABORT_REPLY, "wrong write width aborted");
  check(reply(drive, request(2, 0x3004, 0, 2, 1)).data[0] == ABORT_REPLY, "wrong controller protocol aborted");
  check(reply(drive, request(2, 0x6041, 0, 2, 1)).data[0] == ABORT_REPLY, "read-only write aborted");
  check(reply(drive, request(2, 0x606C, 1)).data[0] == ABORT_REPLY, "wrong subindex aborted");
}
void mappingAndNmt()
{
  auto driver = backend(2, true); VirtualDriveProtocol drive(2, true);
  configure(drive, driver.tractionPdoConfigurationRequests(1)); enable(drive, 2, true);
  check(reply(drive, request(2, 0x1A00, 1, 4, 0xDEAD0020)).data[0] == ABORT_REPLY, "live mapping edits rejected");
  write(drive, 2, 0x1A00, 0, 1, 0); write(drive, 2, 0x1A00, 1, 4, 0xDEAD0020);
  check(reply(drive, request(2, 0x1A00, 0, 1, 1)).data[0] == ABORT_REPLY, "unknown mapping activation rejected");
  drive.handle(createNmtFrame(0x80, 3)); check(drive.enabled(), "other node NMT ignored");
  drive.handle(createNmtFrame(0x80, 0));
  check(!drive.enabled() && drive.handle(createSyncFrame()).empty(), "broadcast preop stops PDO/motion");
  configure(drive, driver.tractionPdoConfigurationRequests(1));
  drive.handle(createNmtFrame(0x81, 2));
  check(drive.handle(createSyncFrame()).empty(), "NMT reset clears mapping and stops PDO");
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
    configure(drive, traction ? driver.tractionPdoConfigurationRequests(1) : driver.steeringTelemetryPdoConfigurationRequests(1));
    enable(drive, node, traction);
    for (const auto& request : {driver.readDeviceStateRequest(), driver.readErrorRegisterRequest(),
         driver.readErrorCodeRequest(), driver.readFeedbackRequest(CURRENT_FEEDBACK),
         driver.readFeedbackRequest(VELOCITY_FEEDBACK), driver.readFeedbackRequest(POSITION_FEEDBACK)})
    { driver.update(reply(drive, request)); }
    check(driver.feedbackFreshness().ready(!traction, 0.3, 0.3, 2.5), "every virtual drive must provide production required feedback fields");
    check(driver.enabled() && !driver.hasError() && driver.getErrorCode() == 0, "each virtual unit is enabled and fault-free");
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
    else { write(drive, node, 0x3003, 0, 4, CURRENT); write(drive, node, 0x3004, 0, 2, 1); }
    write(drive, node, 0x3200, 0, 4, static_cast<uint32_t>(-1000));
    const auto frame = reply(drive, request(node, traction ? 0x6078 : 0x3262, traction ? 0 : 1));
    check(signedIntFromSdoReadReply(frame, 4) == -1000, "repository current command convention must have signed feedback");
    check(read(drive, node, traction ? 0x606C : 0x3A04, traction ? 0 : 1) == 0, "current mode must not invent velocity dynamics");
  }
}
int main()
{
  const std::vector<std::pair<const char*, std::function<void()>>> cases{
    {"production initialization and enable/disable", initialization}, {"rate-limited steering motion", steeringMotion},
    {"ELMO SDO velocity/current widths", tractionSdo}, {"production RPDO/SYNC/TPDO roundtrip", tractionPdo},
    {"steering telemetry and SYNC divider", steeringPdos}, {"fault reset and unknown safety registers", faultsAndSafetyUnknown},
    {"disable/re-enable does not revive targets", disableNoResume}, {"malformed frames and controller isolation", malformedAndIsolation},
    {"PDO mapping validation and NMT", mappingAndNmt}, {"configuration validation", configurationValidation},
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
