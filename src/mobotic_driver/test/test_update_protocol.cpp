#include <mobotic_driver/sdo_configuration.hpp>
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
    // A strict SDO server requires zero unused upload-request bytes. Checking
    // that response verification still rejects the wrong value also proves
    // canonicalizing the request has not overwritten the expected setting.
    MoboticDriveCanCommunication strict_backend("traction", 2, VELOCITY, 4096, 16, {}, "micontrol");
    auto mode_write = strict_backend.setDeviceModeRequest(VELOCITY);
    for (const bool mismatch : {false, true})
    {
      SdoConfiguration readback; readback.start({mode_write});
      const auto request = readback.next();
      check(request && request->data == mode_write.data, "configuration write value preserved");
      auto ack = createAndFillMoboticHeader(backend.replyNodeID(), WRITE_REPLY, MICONTROL_DEVICE_MODE, 0);
      readback.receive(ack);
      const auto upload = readback.next();
      check(upload && upload->data[0] == READ_FROM_DEVICE, "write acknowledgement requires readback");
      for (unsigned i = 4; i < 8; ++i)
      { check(upload->data[i] == 0, "unused upload request bytes must be zero"); }
      auto response = createAndFillMoboticHeader(backend.replyNodeID(), READ_REPLY_4B, MICONTROL_DEVICE_MODE, 0);
      intToFrame(response, 4, mismatch ? POSITION : VELOCITY);
      readback.receive(response);
      check(mismatch ? readback.failed() : readback.ready(), "readback must compare original expected mode");
    }
    const auto write = createAndFillMoboticHeader(backend.canNodeID(), WRITE_TO_DEVICE_4B, MICONTROL_DEVICE_MODE, 0);
    SdoConfiguration timeout; timeout.start({write});
    const auto start = SdoConfiguration::Clock::now();
    timeout.next(start, 100);
    auto ack = createAndFillMoboticHeader(backend.replyNodeID(), WRITE_REPLY, MICONTROL_DEVICE_MODE, 0);
    ack.header.stamp.nanosec = 99;
    timeout.receive(ack);
    check(!timeout.next(start+std::chrono::milliseconds(1), 101), "old config ACK cannot advance transaction");
    timeout.next(start+std::chrono::seconds(3), 102);
    check(timeout.failed(), "configuration timeout is bounded");
    SdoConfiguration aborted; aborted.start({write}); aborted.next();
    ack.data[0] = ABORT_REPLY; aborted.receive(ack);
    check(aborted.failed(), "configuration abort fails closed");
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
    std::cout << "PASS: SDO readback and timeout, counter wrap and durable exclusive state\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
  return 0;
}
