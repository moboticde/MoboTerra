#include <mobotic_driver/pdo_configuration.hpp>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <mobotic_driver/traction_position.hpp>
#include <mobotic_driver/position_state_store.hpp>
#include <iostream>
#include <cstdlib>

void check(bool condition, const char* text) { if (!condition) { throw std::runtime_error(text); } }
int main()
{
  try {
    MoboticDriveCanCommunication backend("traction", 2, VELOCITY, 4096, 16, {});
    VirtualDriveProtocol virtual_drive(2, true, 4096, 16);
    PdoConfiguration setup; setup.start(backend.tractionPdoConfigurationRequests(1));
    for (unsigned attempt = 0; !setup.ready() && attempt < 200; ++attempt)
    { const auto frame = setup.next(); if (frame) { for (const auto& reply : virtual_drive.handle(*frame)) { setup.receive(reply); } } }
    check(setup.ready() && !setup.failed(), "acknowledged/readback PDO setup");
    const auto write = createAndFillMoboticHeader(backend.canNodeID(), WRITE_TO_DEVICE_1B, 0x1A02, 0);
    PdoConfiguration timeout; timeout.start({write});
    const auto start = PdoConfiguration::Clock::now();
    timeout.next(start, 100);
    auto ack = createAndFillMoboticHeader(backend.replyNodeID(), WRITE_REPLY, 0x1A02, 0);
    ack.header.stamp.nanosec = 99;
    timeout.receive(ack);
    check(!timeout.next(start+std::chrono::milliseconds(1), 101), "old config ACK cannot advance transaction");
    timeout.next(start+std::chrono::seconds(3), 102);
    check(timeout.failed(), "configuration timeout is bounded");
    PdoConfiguration aborted; aborted.start({write}); aborted.next();
    ack.data[0] = ABORT_REPLY; aborted.receive(ack);
    check(aborted.failed(), "configuration abort fails closed");
    const auto samples = virtual_drive.handle(MoboticDriveCanCommunication::syncFrame());
    check(samples.size() == 3 && samples.back().id == 0x382 && samples.back().dlc == 4, "position TPDO3 while disabled");
    for (const auto& frame : samples) { backend.update(frame); }
    check(backend.hasFeedback(POSITION_FEEDBACK) && backend.feedback(POSITION_FEEDBACK) == 0, "traction position decode");
    check(!std::isfinite(backend.motorCurrentAmps()), "current needs rated-current reference");
    auto rated = createAndFillMoboticHeader(backend.canNodeID(), READ_FROM_DEVICE, EC_TRACTION_RATED_CURRENT, 0);
    for (const auto& reply : virtual_drive.handle(rated)) { backend.update(reply); }
    auto current = createAndFillMoboticHeader(backend.replyNodeID(), static_cast<FrameOperationsTypes>(0x4B), EC_TRACTION_CURRENT_FEEDBACK, 0);
    current.data[4] = 0xF4; current.data[5] = 0x01;  // +500 per mille of 1 A
    backend.update(current);
    check(std::abs(backend.motorCurrentAmps() - 0.5) < 1e-9, "rated-current conversion to amperes");
    check(!std::isfinite(backend.motorCurrentAmps(FeedbackFreshness::Clock::now()+std::chrono::seconds(3))), "stale rated-current reference rejected");
    TractionPosition position;
    check(position.update(INT32_MAX-1, 100) && position.update(INT32_MIN+1, 100), "positive wrap accepted");
    check(position.count() == static_cast<int64_t>(INT32_MAX)+2, "continuous wrap count");
    check(!position.update(12345, 100), "implausible jump rejected");
    char directory[] = "/tmp/moboterra-position-test-XXXXXX";
    check(::mkdtemp(directory) != nullptr, "temporary state root");
    PositionState state{"robot-1", "encoder-2", "front_left_traction", "scaling-v1", "calibration-1", "hardware-1", 123, 123, 0, 123};
    {
      PositionStateStore store(directory); store.write("front_left_traction", state);
      check(store.read("front_left_traction", state)->count == 123, "durable roundtrip");
      bool rejected = false; try { PositionStateStore second(directory); } catch (...) { rejected = true; }
      check(rejected, "second writer rejected");
      auto bad = state; bad.robot = "another-robot";
      rejected = false; try { store.read("front_left_traction", bad); } catch (...) { rejected = true; }
      check(rejected, "identity mismatch rejected");
      bad = state; bad.calibration_epoch = "another-calibration";
      rejected = false; try { store.read("front_left_traction", bad); } catch (...) { rejected = true; }
      check(rejected, "calibration epoch mismatch rejected");
    }
    ::unlink((std::string(directory)+"/front_left_traction.state").c_str());
    ::unlink((std::string(directory)+"/writer.lock").c_str()); ::rmdir(directory);
    std::cout << "PASS: PDO readback, passive position, counter wrap and durable exclusive state\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
  return 0;
}
