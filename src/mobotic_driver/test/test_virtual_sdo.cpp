// Production virtual-mode bootstrap over an isolated ROS topic bus.
#include <mobotic_driver/mobotic_driver.h>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <chrono>
#include <iostream>
#include <thread>

void require(bool condition, const char* message)
{ if (!condition) { throw std::runtime_error(message); } }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    using Frame = can_msgs::msg::Frame;
    using State = mobotic_interfaces::msg::VehicleModeState;
    using Operation = mobotic_interfaces::msg::WheelModuleOperationCommand;
    std::vector<std::string> names, modes, ids;
    std::vector<double> gearing;
    for (const std::string module : {"front_left", "front_right", "rear_left", "rear_right"})
    {
      names.insert(names.end(), {module + "_steering", module + "_traction"});
      modes.insert(modes.end(), {"position", "velocity"}); gearing.insert(gearing.end(), {121, 16});
    }
    for (unsigned node = 1; node <= 8; ++node) { ids.push_back(std::to_string(node)); }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"virtual_position_source", true},
      {"watchdog_timeout", .3}, {"drives.names", names},
      {"drives.modes", modes},
      {"drives.can_node_ids", ids},
      {"drives.resolutions", std::vector<int64_t>(8, 4096)},
      {"drives.gear_ratios", gearing},
      {"drives.min_steering_velocities", std::vector<int64_t>(8, -5000)},
      {"drives.max_steering_velocities", std::vector<int64_t>(8, 5000)},
      {"drives.min_traction_velocities", std::vector<int64_t>(8, -1700)},
      {"drives.max_traction_velocities", std::vector<int64_t>(8, 1700)},
      {"drives.min_currents", std::vector<int64_t>(8, -4000)},
      {"drives.max_currents", std::vector<int64_t>(8, 4000)}
    });
    auto bus = std::make_shared<rclcpp::Node>("virtual_sdo_test_bus");
    auto replies = bus->create_publisher<Frame>("can_rx", rclcpp::QoS(rclcpp::KeepAll()));
    auto authority = bus->create_publisher<State>("vehicle/mode", 1);
    auto operations = bus->create_publisher<Operation>("wheel_modules/operation_command", 1);
    std::map<unsigned, VirtualDriveProtocol> models;
    for (unsigned node = 1; node <= 8; ++node)
    { models.emplace(node, VirtualDriveProtocol(node, node % 2 == 0, 4096, node % 2 ? 121 : 16)); }
    const auto all_disabled = [&] {
      return std::none_of(models.begin(), models.end(), [](const auto& item) { return item.second.enabled(); });
    };
    unsigned joint_sets = 0;
    bool enabled = false, lease = false;
    auto commands = bus->create_subscription<Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()), [&](const Frame& frame) {
      require(frame.id >= 0x601 && frame.id <= 0x608, "virtual driver must emit only drive SDOs");
      const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1));
      require(index < 0x1400 || index > 0x1BFF, "virtual startup must not remap PDOs");
      for (auto& [node, model] : models)
      {
        for (auto reply : model.handle(frame))
        { reply.header.stamp = bus->now(); replies->publish(reply); }
      }
    });
    auto joints = bus->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, [&](const sensor_msgs::msg::JointState& msg) {
      require(msg.name.size() == 8 && msg.position.size() == 8 && msg.velocity.size() == 8,
              "virtual SDO scan must publish every configured joint");
      ++joint_sets;
    });
    auto status = bus->create_subscription<mobotic_interfaces::msg::WheelModuleStatusArray>("wheel_modules/status", 10,
        [&](const mobotic_interfaces::msg::WheelModuleStatusArray& msg) { enabled = msg.modules.size() == 4 && std::all_of(msg.modules.begin(), msg.modules.end(), [](const auto& item) { return item.enabled && item.feedback_fresh; }); });
    auto timer = bus->create_wall_timer(std::chrono::milliseconds(20), [&] {
      if (!lease) { return; }
      State state; state.header.stamp = bus->now(); state.supervisor_instance = "virtual-sdo-test";
      state.authority_generation = 1; state.selected_mode = State::MANUAL; state.requested_mode = State::MANUAL;
      state.current_mode = State::MANUAL; state.motion_permitted = true;
      authority->publish(state);
      Operation operation; operation.header.stamp = bus->now(); operation.supervisor_instance = state.supervisor_instance;
      operation.authority_generation = 1; operation.enable = true; operations->publish(operation);
    });
    for (const std::string key : {"traction_pdo_enabled", "telemetry_pdo_enabled"})
    {
      auto invalid = options; invalid.append_parameter_override(key, true);
      bool rejected = false;
      try { auto unsupported = std::make_shared<MoboticDriver>(invalid); }
      catch (const std::invalid_argument&) { rejected = true; }
      require(rejected, "removed PDO transports must be rejected at the driver boundary");
    }
    auto driver = std::make_shared<MoboticDriver>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(bus); executor.add_node(driver);
    const auto wait_for = [&](auto predicate, double seconds) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < deadline)
      {
        executor.spin_all(std::chrono::milliseconds(5));
        if (predicate()) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return false;
    };
    require(wait_for([&] { return joint_sets > 2; }, 5), "virtual SDO startup must acquire stationary feedback before enable");
    require(all_disabled(), "initialization must leave all virtual drives disabled");
    lease = true;
    require(wait_for([&] { return enabled; }, 5), "fresh SDO feedback and authority must permit virtual enable");
    lease = false;
    require(wait_for([&] { return all_disabled(); }, 2), "lost management lease disables virtual drives");
    executor.remove_node(driver); driver.reset(); executor.remove_node(bus);
    std::cout << "PASS: virtual miControl SDO bootstrap, complete feedback, enable and lease loss\n";
  }
  catch (const std::exception& error)
  { std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
}
