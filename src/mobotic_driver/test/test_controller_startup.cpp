// Runs the actual physical driver against controllers which supply motion
// telemetry only AFTER initialization and enable, matching the EC hardware.
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
    auto commands = bus->create_publisher<mobotic_interfaces::msg::WheelModuleCommandArray>("wheel_modules/command", 1);
    std::map<unsigned, VirtualDriveProtocol> controllers;
    for (unsigned node : {3u, 4u, 7u, 8u})
    {
      const bool traction = node == 4 || node == 8;
      controllers.emplace(node, VirtualDriveProtocol(node, traction));
    }
    controllers.at(4).setFault(1, 0x2310);
    unsigned startup_reset_attempts = 0;
    std::vector<Frame> transmitted;
    bool drop_replies = false, drop_motion = false;
    auto tx = bus->create_subscription<Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()), [&](const Frame& frame) {
      transmitted.push_back(frame);
      if (drop_replies) { return; }
      const bool sdo = frame.id >= 0x601 && frame.id <= 0x67F;
      const unsigned index = sdo ? static_cast<unsigned>(static_cast<uint16_t>(shortFromFrame(frame, 1))) : 0;
      const bool motion_read = sdo && frame.data[0] == READ_FROM_DEVICE &&
          (index == 0x6064 || index == 0x606C || index == 0x6078 || index == 0x3762 || index == 0x3A04 || index == 0x3262);
      for (auto& [node, controller] : controllers)
      {
        if (motion_read && (!controller.enabled() || drop_motion)) { continue; }
        for (auto reply : controller.handle(frame))
        {
          // A transient startup fault persists beyond both entry clears.
          // Recovery requires the EC-style periodic initialization retry.
          if (node == 4 && frame.id == 0x604 && frame.data[0] != READ_FROM_DEVICE &&
              index == EC_CONTROLWORD && shortFromFrame(frame, 4) == 0x0080 &&
              ++startup_reset_attempts < 4)
          { controller.setFault(1, 0x2310); }
          if (reply.id < 0x580 && (!controller.enabled() || drop_motion)) { continue; }
          if (sdo && frame.data[0] == READ_FROM_DEVICE && (index == 0x6064 || index == 0x3762))
          { intToFrame(reply, 4, index == 0x6064 ? 123456 : 2048); }
          if (reply.id == 0x380 + node && (node == 4 || node == 8)) { intToFrame(reply, 0, 123456); }
          if (reply.id == 0x180 + node && (node == 3 || node == 7)) { intToFrame(reply, 0, 2048); }
          reply.header.stamp = bus->now();
          rx->publish(reply);
        }
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
        // A SYNC produces several CAN replies. Drain ready callbacks in a
        // bounded batch before sleeping; spin_some() only collects work once
        // and can leave replies queued until their acquisition stamps expire
        // on a busy build host.
        executor.spin_all(std::chrono::milliseconds(5));
        if (predicate()) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return false;
    };
    const auto all = [&](const std::function<bool(const mobotic_interfaces::msg::WheelModuleStatus&)>& predicate) {
      return latest.modules.size() == 2 && std::all_of(latest.modules.begin(), latest.modules.end(), predicate);
    };
    require(spin_until([&] { return all([](const auto& m) { return !m.startup_pending && !m.startup_complete && !m.enabled; }); }, 2),
            "before supervisor readiness, startup waits without requiring motion feedback");
    require(joints.name.empty(), "disabled controllers deliberately provide no motion feedback");
    for (const auto& frame : transmitted)
    { require(frame.id >= 0x700 || (frame.id >= 0x600 && frame.data[0] == READ_FROM_DEVICE), "startup must be read-only apart from master heartbeat"); }
    bool lease = true, enable = true, resetting = false, adversarial_motion = false;
    uint64_t reset_id = 0;
    auto heartbeat = bus->create_wall_timer(std::chrono::milliseconds(20), [&] {
      if (!lease) { return; }
      State state; state.header.stamp = bus->now(); state.selected_mode = State::MANUAL;
      state.supervisor_instance = "integration-supervisor"; state.authority_generation = 1;
      state.reset_in_progress = resetting; state.reset_id = reset_id;
      state.error_active = resetting && !adversarial_motion;
      state.motion_permitted = adversarial_motion; // Deliberately inconsistent during reset to exercise driver veto.
      authority->publish(state);
      Operation operation; operation.header.stamp = bus->now();
      operation.supervisor_instance = state.supervisor_instance; operation.authority_generation = 1;
      operation.enable = enable; operation.reset_id = reset_id;
      operations->publish(operation);
      if (adversarial_motion)
      {
        mobotic_interfaces::msg::WheelModuleCommandArray command;
        command.header.stamp = bus->now(); command.source_stamp = command.header.stamp;
        command.supervisor_instance = state.supervisor_instance; command.authority_generation = 1;
        command.selected_mode = State::MANUAL; command.purpose = command.MOTION;
        for (const auto& name : {"front_right", "rear_left"})
        {
          mobotic_interfaces::msg::WheelModuleCommand module;
          module.name = name; module.steering_position = 0.5;
          module.steering_profile_velocity = 1.0; module.traction_velocity = 1.0;
          command.modules.push_back(module);
        }
        commands->publish(command);
      }
    });
    const bool startup_ready = spin_until([&] { return all([](const auto& m) { return m.startup_complete && m.enabled && m.feedback_fresh && m.position_valid; }) && joints.name.size() == 4; }, 12);
    if (!startup_ready)
    {
      for (const auto& module : latest.modules)
      {
        std::cerr << module.name << ": startup_complete=" << module.startup_complete
                  << ", enabled=" << module.enabled << ", feedback_fresh=" << module.feedback_fresh
                  << ", position_valid=" << module.position_valid << ", " << module.status_message << '\n';
        for (const auto* drive : {&module.steering, &module.traction})
        {
          std::cerr << "  " << drive->name << ": initialized=" << drive->initialized
                    << ", status_known=" << drive->status_known << ", enabled=" << drive->enabled
                    << ", error_register=" << drive->error_register << ", error_code=" << drive->error_code << '\n';
        }
      }
      for (const auto& [node, controller] : controllers)
      { std::cerr << "  controller " << node << ": enabled=" << controller.enabled() << '\n'; }
    }
    require(startup_ready, "supervised initialization/enable must acquire the first real motion feedback");
    require(startup_reset_attempts >= 4, "startup retries persistent errors beyond the entry fault clears");
    for (std::size_t i = 0; i < joints.name.size(); ++i)
    {
      const bool traction = joints.name[i].find("traction") != std::string::npos;
      const double expected = traction ? 123456.0 * 2 * M_PI / (4096 * 16) : M_PI;
      require(std::abs(joints.position[i] - expected) < 1e-8, "startup preserves the existing controller position");
    }
    for (unsigned node : {4u, 8u})
    {
      std::vector<int> words;
      for (const auto& frame : transmitted)
      {
        if (frame.id == 0x200 + node && frame.dlc == 6) { words.push_back(shortFromFrame(frame, 0)); }
        if (frame.id == 0x600 + node && frame.data[0] != READ_FROM_DEVICE && shortFromFrame(frame, 1) == EC_CONTROLWORD)
        { words.push_back(shortFromFrame(frame, 4)); }
      }
      require(std::find(words.begin(), words.end(), 6) != words.end() && std::find(words.begin(), words.end(), 7) != words.end() &&
              std::find(words.begin(), words.end(), 15) != words.end(), "each Elmo receives shutdown, switch-on and enable-operation");
    }
    // B2 can reset/re-enable from freshly measured standstill even without an
    // existing fault. Disabled motion feedback is withheld during the reset.
    enable = false; resetting = true; reset_id = 1;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 1 && m.reset_acknowledged && !m.enabled && m.error_code == 0; }); }, 5),
            "B2 management transaction must acknowledge fresh fault clearance while disabled");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 8), "acknowledged B2 reset permits zero-target re-enable while reset remains active");
    reset_id = 0; resetting = false;
    enable = false;
    controllers.at(4).setFault(1, 0x2310);
    require(spin_until([&] { return latest.modules.size() == 2 && latest.modules.front().error_code != 0; }, 4), "runtime drive fault must be observed without automatic reset");
    enable = false;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled && m.steering.status_known && m.traction.status_known; }); }, 3),
            "runtime fault disables drives with fresh native status");
    spin_until([] { return false; }, 0.6); // Let all disabled-drive motion samples expire.
    resetting = true; reset_id = 2;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 2 && m.reset_acknowledged && !m.enabled && m.error_code == 0; }); }, 5),
            "fresh disabled native states admit reset after motion telemetry expires");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh && m.position_valid; }); }, 5),
            "faulted disabled controllers reacquire real motion feedback under zero-target recovery");
    adversarial_motion = true;
    spin_until([] { return false; }, 0.3);
    adversarial_motion = false; resetting = false; reset_id = 0;
    lease = false;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 3), "lost supervisor still disables physical drives");
    for (const auto& frame : transmitted)
    {
      if ((frame.id == 0x204 || frame.id == 0x208) && frame.dlc == 6)
      { require(intFromFrame(frame, 2) == 0, "startup and reset recovery never publish nonzero traction targets"); }
      if (frame.id < 0x601 || frame.id > 0x67F || frame.data[0] == READ_FROM_DEVICE) { continue; }
      const auto index = static_cast<unsigned short>(shortFromFrame(frame, 1));
      require(index != EC_TRACTION_POSITION_FEEDBACK && index != ABSOLUTE_POSITION_INPUT && index != 0x1010 && index != 0x607C,
              "startup never zeroes/homes position, sends steering movement or saves controller settings");
    }
    // A second driver attempt with no CAN responses must fail within its own
    // deadline, even while supervisor enable continues to be refreshed.
    executor.remove_node(driver); driver.reset();
    drop_replies = true; lease = true; enable = true;
    auto timeout_options = options;
    timeout_options.append_parameter_override("startup_timeout", 0.4);
    driver = std::make_shared<MoboticDriver>(timeout_options); executor.add_node(driver);
    require(spin_until([&] { return all([](const auto& m) { return m.startup_failed && !m.startup_complete; }); }, 3),
            "missing configuration/feedback responses must fail bounded startup");
    spin_until([] { return false; }, 0.1); // Drain replies already in DDS queues.
    const auto failed_transmissions = transmitted.size();
    spin_until([] { return false; }, 0.3);
    require(std::none_of(transmitted.begin() + failed_transmissions, transmitted.end(), [](const auto& frame) {
      return frame.id == 0x204 || frame.id == 0x208 || frame.id == 0x80;
    }), "failed physical startup must not keep publishing unverified RPDO/SYNC cycles");
    drop_replies = false;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_failed && !m.enabled && m.steering.status_known && m.traction.status_known &&
             m.steering.fault_known && m.traction.fault_known;
    }); }, 4), "failed startup must continue read-only native status and fault polling");
    executor.remove_node(driver); driver.reset();
    // Successful configuration/enable without motion samples is bounded too.
    drop_replies = false; drop_motion = true;
    auto no_feedback_options = options;
    no_feedback_options.append_parameter_override("startup_timeout", 7.0);
    driver = std::make_shared<MoboticDriver>(no_feedback_options); executor.add_node(driver);
    require(spin_until([&] { return std::all_of(controllers.begin(), controllers.end(), [](const auto& item) { return item.second.enabled(); }); }, 6),
            "controllers enable even when every motion sample is suppressed");
    require(spin_until([&] { return all([](const auto& m) { return m.startup_failed && !m.startup_complete; }); }, 8),
            "enabled drives without feedback must time out without granting motion");
    executor.remove_node(driver); driver.reset();
    // Loss of supervisor permission cancels pending initialization as well.
    drop_replies = true;
    driver = std::make_shared<MoboticDriver>(options); executor.add_node(driver);
    require(spin_until([&] { return all([](const auto& m) { return m.startup_pending; }); }, 2), "startup begins under supervisor grant");
    enable = false;
    require(spin_until([&] { return all([](const auto& m) { return m.startup_failed; }); }, 2), "readiness loss cancels startup");
    drop_replies = false; drop_motion = false;
    require(spin_until([&] { return all([](const auto& m) {
      return !m.enabled && m.steering.status_known && m.traction.status_known;
    }); }, 4), "cancelled startup retains fresh disabled-state observations");
    resetting = true; reset_id = 3;
    require(spin_until([&] { return all([](const auto& m) {
      return m.reset_id == 3 && m.reset_acknowledged;
    }); }, 5), "explicit B2 recovery clears errors after failed startup");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && !m.startup_failed && m.enabled && m.feedback_fresh;
    }); }, 12), "B2 recovery reinitializes and enables after failed startup");
    executor.remove_node(driver); driver.reset(); executor.remove_node(bus);
    std::cout << "PASS: supervised EC startup, expired-feedback B2 recovery, reset motion veto, lease loss, timeout and cancellation\n";
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1; }
}
