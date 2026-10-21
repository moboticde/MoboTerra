// Runs the actual physical driver against an in-process CANopen responder.
// No SocketCAN adapter, motor or fake physical encoder-reference parameter.
#include <mobotic_driver/mobotic_driver.h>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

using Frame = can_msgs::msg::Frame;
using State = mobotic_interfaces::msg::VehicleModeState;
using Operation = mobotic_interfaces::msg::WheelModuleOperationCommand;
using Modules = mobotic_interfaces::msg::WheelModuleStatusArray;
void require(bool value, const char* message) { if (!value) { throw std::runtime_error(message); } }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const std::vector<std::string> names{"front_right_steering", "front_right_traction", "rear_left_steering", "rear_left_traction"};
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"can_node_id", "0x0B"}, {"virtual_position_source", false},
      {"feedback_request_period", .05}, {"can_heartbeat_period", .1},
      {"drives.names", names}, {"drives.modes", std::vector<std::string>{"position", "velocity", "position", "velocity"}},
      {"drives.can_node_ids", std::vector<std::string>{"3", "4", "7", "8"}},
      {"drives.gear_ratios", std::vector<double>{121, 16, 121, 16}},
      {"drives.resolutions", std::vector<int64_t>(4, 4096)},
      {"drives.min_steering_velocities", std::vector<int64_t>(4, -5000)},
      {"drives.max_steering_velocities", std::vector<int64_t>(4, 5000)},
      {"drives.min_traction_velocities", std::vector<int64_t>(4, -116053)},
      {"drives.max_traction_velocities", std::vector<int64_t>(4, 116053)},
      {"drives.min_currents", std::vector<int64_t>(4, -4000)},
      {"drives.max_currents", std::vector<int64_t>(4, 4000)},
      {"modules.names", std::vector<std::string>{"front_right", "rear_left"}},
      {"modules.steering_drives", std::vector<std::string>{names[0], names[2]}},
      {"modules.traction_drives", std::vector<std::string>{names[1], names[3]}}
    });
    auto bus = std::make_shared<rclcpp::Node>("controller_startup_test_bus");
    auto rx = bus->create_publisher<Frame>("can_rx", rclcpp::QoS(rclcpp::KeepAll()));
    auto authority = bus->create_publisher<State>("vehicle/mode", 1);
    auto operations = bus->create_publisher<Operation>("wheel_modules/operation_command", 1);
    std::map<unsigned, VirtualDriveProtocol> controllers;
    for (unsigned node : {3u, 4u, 7u, 8u})
    {
      const bool traction = node == 4 || node == 8;
      controllers.emplace(node, VirtualDriveProtocol(node, traction));
      MoboticDriveCanCommunication fixture("fixture", node, traction ? VELOCITY : POSITION, 4096, 16, {});
      controllers.at(node).handle(fixture.setDeviceModeRequest(fixture.targetMode()));
    }
    std::vector<Frame> transmitted;
    bool drop_replies = false;
    auto tx = bus->create_subscription<Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()), [&](const Frame& frame) {
      transmitted.push_back(frame);
      if (drop_replies || frame.id < 0x601 || frame.id > 0x67F) { return; }
      const auto node = frame.id - 0x600;
      if (!controllers.count(node)) { return; }
      for (auto reply : controllers.at(node).handle(frame))
      {
        const unsigned index = static_cast<unsigned>(frame.data[1]) | (static_cast<unsigned>(frame.data[2]) << 8);
        if (frame.data[0] == READ_FROM_DEVICE && (index == 0x6064 || index == 0x3762))
        { intToFrame(reply, 4, index == 0x6064 ? 123456 : 2048); }
        reply.header.stamp = bus->now();
        rx->publish(reply);
      }
    });
    Modules latest;
    sensor_msgs::msg::JointState joints;
    auto status_sub = bus->create_subscription<Modules>("wheel_modules/status", 10, [&](const Modules& s) { latest = s; });
    auto joint_sub = bus->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, [&](const sensor_msgs::msg::JointState& s) { joints = s; });
    auto driver = std::make_shared<MoboticDriver>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(bus); executor.add_node(driver);
    const auto spin_until = [&](const std::function<bool()>& predicate, double timeout) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
      while (std::chrono::steady_clock::now() < deadline)
      {
        executor.spin_some();
        if (predicate()) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return false;
    };
    const auto all = [&](const std::function<bool(const mobotic_interfaces::msg::WheelModuleStatus&)>& predicate) {
      return latest.modules.size() == 2 && std::all_of(latest.modules.begin(), latest.modules.end(), predicate);
    };
    require(spin_until([&] { return all([](const auto& m) { return m.initialized && m.position_valid && m.feedback_fresh && !m.enabled; }) && joints.name.size() == 4; }, 8),
            "physical startup must acquire disabled drive state and real controller positions");
    for (const auto& frame : transmitted)
    { require(frame.id >= 0x700 || (frame.id >= 0x600 && frame.data[0] == READ_FROM_DEVICE), "startup must be read-only apart from master heartbeat"); }
    for (std::size_t i = 0; i < joints.name.size(); ++i)
    {
      const bool traction = joints.name[i].find("traction") != std::string::npos;
      const double expected = traction ? 123456.0 * 2 * M_PI / (4096 * 16) : M_PI;
      require(std::abs(joints.position[i] - expected) < 1e-8, "position comes from existing controller count without zeroing it");
    }
    bool lease = true, enable = true, resetting = false;
    uint64_t reset_id = 0;
    auto heartbeat = bus->create_wall_timer(std::chrono::milliseconds(20), [&] {
      if (!lease) { return; }
      State state; state.header.stamp = bus->now(); state.selected_mode = State::MANUAL;
      state.supervisor_instance = "integration-supervisor"; state.authority_generation = 1;
      state.reset_in_progress = resetting; state.error_active = resetting;
      authority->publish(state);
      Operation operation; operation.header.stamp = bus->now();
      operation.supervisor_instance = state.supervisor_instance; operation.authority_generation = 1;
      operation.enable = enable; operation.reset_id = reset_id;
      operations->publish(operation);
    });
    require(spin_until([&] { return all([](const auto& m) { return m.enabled; }); }, 8), "physical drives must enable through observed states");
    for (unsigned node : {4u, 8u})
    {
      std::vector<int> words;
      for (const auto& frame : transmitted)
      { if (frame.id == 0x600 + node && frame.data[0] != READ_FROM_DEVICE && shortFromFrame(frame, 1) == EC_CONTROLWORD) { words.push_back(shortFromFrame(frame, 4)); } }
      require(std::find(words.begin(), words.end(), 6) != words.end() && std::find(words.begin(), words.end(), 7) != words.end() &&
              std::find(words.begin(), words.end(), 15) != words.end(), "each Elmo receives shutdown, switch-on and enable-operation");
    }
    controllers.at(4).setFault(1, 0x2310);
    require(spin_until([&] { return latest.modules.size() == 2 && latest.modules.front().error_code != 0; }, 4), "drive fault must be observed");
    enable = false; resetting = true; reset_id = 1;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 1 && m.reset_acknowledged && !m.enabled && m.error_code == 0; }); }, 5),
            "B2 management transaction must acknowledge fresh fault clearance while disabled");
    reset_id = 0; resetting = false; enable = true;
    require(spin_until([&] { return all([](const auto& m) { return m.enabled; }); }, 8), "successful B2 reset permits re-enable");
    lease = false;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 3), "lost supervisor still disables physical drives");
    for (const auto& frame : transmitted)
    {
      if (frame.id < 0x601 || frame.id > 0x67F || frame.data[0] == READ_FROM_DEVICE) { continue; }
      const auto index = static_cast<unsigned short>(shortFromFrame(frame, 1));
      require(index == EC_CONTROLWORD || index == EC_TRACTION_VELOCITY_INPUT || index == MICONTROL_CLEAR_ERROR_STATUS ||
              index == MICONTROL_DEVICE_STATE || index == STEERING_PROFILE_VELOCITY_INPUT,
              "startup/reset/enable must never write modes, PDOs, encoder reference, ramps or saved parameters");
    }
    executor.remove_node(driver); driver.reset(); executor.remove_node(bus);
    std::cout << "PASS: actual physical driver read-only startup, controller positions, enable, fault reset/re-enable and lease loss\n";
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1; }
}
