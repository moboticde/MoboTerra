#include "vanguard_battery_driver/vanguard_driver.h"
#include "vanguard_battery_driver/communication.h"
#include <can_msgs/msg/frame.hpp>
#include <iomanip>
#include <iostream>

VanguardBatteryDriver::VanguardBatteryDriver(const rclcpp::NodeOptions& options) : Node("vanguard_driver", options)
{

  this->declare_parameter("source_address", "");
  this->declare_parameter("high_voltage_publish_period", 0.01);
  this->declare_parameter("batteries.ids", std::vector<std::string>());
  this->declare_parameter("batteries.status_pgns", std::vector<std::string>());
  this->declare_parameter("batteries.soc_pgns", std::vector<std::string>());
  this->declare_parameter("batteries.voltage_pgns", std::vector<std::string>());
  this->declare_parameter("batteries.source_addresses", std::vector<std::string>());
  this->declare_parameter("batteries.types", std::vector<std::string>());
  this->declare_parameter("op_consent_duration", 0.0);
  this->declare_parameter("mobotic_driver_watchdog_timeout", 0.0);
  this->declare_parameter("mobotic_driver_source_address", "");

  std::string can_device, source_address_str, mobotic_driver_src_addr_str;
  double high_voltage_publish_period, mobotic_driver_watchdog_timeout;
  std::vector<std::string> ids, status_pgns, source_addresses, charge_pgns, voltage_pgns, types;
  double op_consent_duration;

  bool params{true};
  params &= this->get_parameter("source_address", source_address_str);
  params &= this->get_parameter("high_voltage_publish_period", high_voltage_publish_period);
  params &= this->get_parameter("batteries.ids", ids);
  params &= this->get_parameter("batteries.status_pgns", status_pgns);
  params &= this->get_parameter("batteries.soc_pgns", charge_pgns);
  params &= this->get_parameter("batteries.voltage_pgns", voltage_pgns);
  params &= this->get_parameter("batteries.source_addresses", source_addresses);
  params &= this->get_parameter("batteries.types", types);
  params &= this->get_parameter("op_consent_duration", op_consent_duration);
  params &= this->get_parameter("mobotic_driver_watchdog_timeout", mobotic_driver_watchdog_timeout);
  params &= this->get_parameter("mobotic_driver_source_address", mobotic_driver_src_addr_str);

  if (!params)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "Parameters not found: source_address, feedback_request_period, high_voltage_publish_period, "
                 "batteries.ids, batteries.status_pgns, batteries.source_addresses");
    exit(1);
  }

  if (op_consent_duration <= 0.0)
  {
    RCLCPP_FATAL(this->get_logger(), "Op consent period must not be less or equal to 0");
    exit(1);
  }

  source_address_ = std::stoi(source_address_str, nullptr, 16);

  bool all_sizes_match{true};
  all_sizes_match &= ids.size() == status_pgns.size();
  all_sizes_match &= ids.size() == source_addresses.size();

  if (!all_sizes_match)
  {
    RCLCPP_FATAL(this->get_logger(), "[driver][vanguard] Mismatch in number of batteries' parameters. All batteries' "
                                     "parameters must be of equal size.");
    exit(1);
  }

  for (unsigned i = 0; i < ids.size(); ++i)
  {
    batteries_.emplace_back(ids[i], std::stoi(source_addresses[i], nullptr, 16), std::stoi(status_pgns[i], nullptr, 16),
                            std::stoi(voltage_pgns[i], nullptr, 16), std::stoi(charge_pgns[i], nullptr, 16), types[i]);
    pubs_battery_state_.emplace(
        ids[i], this->create_publisher<sensor_msgs::msg::BatteryState>(ids[i] + "/state", rclcpp::QoS(1)));
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "[vanguard_battery_driver] Loaded params: "
                                             << std::endl
                                             << "\tcan_device: " << can_device << std::endl
                                             << "\tsource address: " << std::hex << std::setfill('0') << std::setw(2)
                                             << static_cast<unsigned int>(source_address_) << std::endl
                                             << "\thigh_voltage_publish_period: " << high_voltage_publish_period);
  hv_command_ = 3;
  op_consent_ = 3;

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));
  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()), [this](const can_msgs::msg::Frame& frame) { canFrameCallback(frame); });

  timer_high_voltage_ =
      this->create_timer(std::chrono::duration<double>{high_voltage_publish_period}, [this, op_consent_duration]() {
        if (high_voltage_requested_)
        {
          bool all_batteries_operational{true};
          for (const auto& battery : batteries_)
          {
            all_batteries_operational &= battery.getOperationalState() == OperationalStatus::OPERATIONAL;
          }
          const auto& time_since_request = this->get_clock()->now() - high_voltage_start_time_;
          if (time_since_request > std::chrono::duration<double>(op_consent_duration) &&
              time_since_request <= std::chrono::duration<double>(2 * op_consent_duration))
          {
            hv_command_ = 0;
            op_consent_ = 1;
          }
          else if (time_since_request > std::chrono::duration<double>(2 * op_consent_duration) &&
                   all_batteries_operational)
          {
            hv_command_ = 1;
            op_consent_ = 1;
          }
        }

        for (const auto& battery : batteries_)
        {
          if (battery.getParticipantType() == ParticipantType::CONTROLLER && !mobotic_driver_watchdog_timeout_)
          {
            pub_can_frames_->publish(
                stampFrame(battery.createHighVoltageRequest(source_address_, hv_command_, op_consent_,
                                                            (high_voltage_msg_counter_++ % 16)),
                           battery.getId(), this->get_clock()->now()));
          }
        }
      });

  timer_mobotic_driver_ = this->create_timer(
      std::chrono::duration<double>{mobotic_driver_watchdog_timeout},
      [this]() {
        mobotic_driver_watchdog_timeout_ = true;
      });

  timer_mobotic_driver_->cancel();

  mobotic_driver_src_addr_ = std::stoi(mobotic_driver_src_addr_str, nullptr, 16);

  srv_enable_high_voltage_ = this->create_service<std_srvs::srv::Trigger>(
      "enable_high_voltage", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        high_voltage_requested_ = true;
        high_voltage_start_time_ = this->get_clock()->now();
        response->success = true;
      });

  srv_disable_high_voltage_ = this->create_service<std_srvs::srv::Trigger>(
      "disable_high_voltage", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        high_voltage_requested_ = false;
        hv_command_ = 0;
        response->success = true;
      });
}

VanguardBatteryDriver::~VanguardBatteryDriver() = default;

void VanguardBatteryDriver::canFrameCallback(const can_msgs::msg::Frame& frame)
{
  if (!frame.is_extended && (frame.id == 0x700 + mobotic_driver_src_addr_) && frame.dlc == 1 && frame.data[0] == 0x05)
  {
    timer_mobotic_driver_->reset();
    return;
  }

  for (auto& battery : batteries_)
  {
    battery.update(frame);

    sensor_msgs::msg::BatteryState msg;
    msg.header.frame_id = battery.getId();
    msg.header.stamp = this->get_clock()->now();
    msg.location = battery.getId();
    msg.percentage = battery.getPercentage();
    msg.voltage = battery.getVoltage();
    msg.current = battery.getCurrent();
    if (battery.getHighVoltageStatus() == HighVoltageStatus::CONNECTED)
    {
      msg.present = true;
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Battery "
                                                      << battery.getId()
                                                      << " is connected to the High Voltage bus! It's ready to use.");
    }
    pubs_battery_state_.at(battery.getId())->publish(msg);
  }
}

can_msgs::msg::Frame VanguardBatteryDriver::stampFrame(const can_msgs::msg::Frame& frame, const std::string& frame_id,
                                                       const rclcpp::Time& stamp)
{
  auto stamped_frame = frame;
  stamped_frame.header.frame_id = frame_id;
  stamped_frame.header.stamp = stamp;
  return stamped_frame;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  VanguardBatteryDriver driver{rclcpp::NodeOptions()};
  rclcpp::spin(driver.get_node_base_interface());
  return 0;
}
