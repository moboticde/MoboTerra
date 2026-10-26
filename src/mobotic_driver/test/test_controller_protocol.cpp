#include <mobotic_driver/sdo_channel.hpp>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

void require(bool value, const char* text) { if (!value) { throw std::runtime_error(text); } }
int main()
{
  try
  {
    MoboticDriveCanCommunication drive("traction", 4, VELOCITY, 4096, 16, {});
    drive.preserveControllerConfiguration(true);
    VirtualDriveProtocol controller(4, true);
    controller.handle(drive.setDeviceModeRequest(VELOCITY)); // Installed controller project fixture.
    for (const auto& reply : controller.handle(drive.readErrorRegisterRequest()))
    { require(drive.update(reply).empty(), "fault polling must not change installed mode"); }
    for (const auto& reply : controller.handle(drive.readDeviceModeRequest())) { drive.update(reply); }
    for (const int word : {6, 7, 15})
    {
      for (const auto& reply : controller.handle(drive.readDeviceStateRequest())) { drive.update(reply); }
      auto commands = drive.enableFromObservedState();
      require(commands.size() == 1 && shortFromFrame(commands.front(), 4) == word, "enable advances only the observed DS402 state");
      controller.handle(commands.front());
    }
    for (const auto& reply : controller.handle(drive.readDeviceStateRequest())) { drive.update(reply); }
    require(drive.enabled() && drive.enableFromObservedState().empty(), "already enabled drive is left enabled");
    controller.setFault(1, 0x2310);
    for (const auto& request : {drive.readDeviceStateRequest(), drive.readErrorRegisterRequest(), drive.readErrorCodeRequest()})
    { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
    require(drive.enableFromObservedState().empty(), "enable never clears a fault");
    auto reset = drive.clearErrorRequest(CLEAR);
    require(shortFromFrame(reset, 4) == 0x80, "explicit reset uses DS402 fault reset bit");
    controller.handle(reset);
    controller.handle(drive.clearErrorRequest(NO_OPERATION));
    for (const auto& request : {drive.readDeviceStateRequest(), drive.readErrorRegisterRequest(), drive.readErrorCodeRequest()})
    { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
    require(!drive.hasError() && drive.getErrorCode() == 0 && !drive.enableFromObservedState().empty(), "B2 reset allows a new enable sequence");
    controller.handle(drive.setDeviceModeRequest(CURRENT));
    for (const auto& reply : controller.handle(drive.readDeviceModeRequest())) { drive.update(reply); }
    require(!drive.initialized() && drive.enableFromObservedState().empty(), "incompatible installed mode blocks enable without rewriting it");
    auto unknown_pdo = controller.handle(drive.readFeedbackRequest(POSITION_FEEDBACK)).front();
    unknown_pdo.id = drive.telemetryTpdo3CobId(); unknown_pdo.dlc = 4;
    uint32ToFrame(unknown_pdo, 0, 999);
    drive.update(unknown_pdo);
    require(!drive.hasFeedback(POSITION_FEEDBACK), "unknown installed PDO mappings cannot fabricate controller feedback");

    SdoChannel channel;
    const auto now = SdoChannel::Clock::now();
    auto read = drive.readDeviceStateRequest();
    channel.push(read, now); channel.push(read, now);
    auto sent = channel.next(.1, now, 100);
    require(sent && !channel.next(.1, now, 101), "exactly one in-flight request and duplicate reads suppressed");
    auto reply = controller.handle(*sent).front(); reply.header.stamp.nanosec = 99;
    require(!channel.receive(reply), "pre-request response rejected");
    reply.header.stamp.nanosec = 100;
    require(channel.receive(reply) && !channel.next(.1, now), "matching read completes transaction");
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    require(!channel.next(.1, now + std::chrono::milliseconds(101)), "queued enable cannot outlive watchdog");
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.cancelWrites();
    require(!channel.next(.1, now), "disable cancels queued enable");
    channel.push(drive.clearErrorRequest(CLEAR), now);
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.cancelWrites(true);
    sent = channel.next(.1, now);
    require(sent && shortFromFrame(*sent, 4) == 0x80, "repeated disable must preserve an explicit pending B2 reset");
    channel.clear();
    channel.push(drive.setTargetRequest(EC_TRACTION_VELOCITY_INPUT, 100), now);
    channel.push(drive.setTargetRequest(EC_TRACTION_VELOCITY_INPUT, 200), now);
    sent = channel.next(.1, now);
    require(sent && intFromFrame(*sent, 4) == 200, "obsolete queued motion target replaced");
    channel.push(read, now);
    require(channel.next(.1, now + std::chrono::milliseconds(101)).has_value(), "missing response has bounded recovery");
    channel.clear();
    channel.push(read, now);
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.push(drive.changeDeviceStateRequest(DISABLE), now);
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && shortFromFrame(*sent, 1) == EC_CONTROLWORD && shortFromFrame(*sent, 4) == 0,
            "disable survives queue delay and takes priority over polling and expired enable");
    channel.clear();
    channel.push(read, now);
    channel.push(drive.setTargetRequest(EC_TRACTION_VELOCITY_INPUT, 0), now);
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && shortFromFrame(*sent, 1) == EC_TRACTION_VELOCITY_INPUT && intFromFrame(*sent, 4) == 0,
            "zero target cannot expire behind telemetry reads");
    auto stop_ack = createAndFillMoboticHeader(drive.replyNodeID(), WRITE_REPLY, EC_TRACTION_VELOCITY_INPUT, 0);
    require(channel.receive(stop_ack), "zero target acknowledged");
    channel.push(drive.setTargetRequest(EC_TRACTION_VELOCITY_INPUT, 0), now + std::chrono::milliseconds(250));
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && sent->data[0] == READ_FROM_DEVICE, "repeated idle zeros cannot starve status reads");
    std::cout << "PASS: preserved configuration, observed-state enable, explicit fault reset and serialized SDOs\n";
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
