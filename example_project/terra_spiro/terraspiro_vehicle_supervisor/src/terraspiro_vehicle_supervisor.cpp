#include "terraspiro_vehicle_supervisor/terraspiro_vehicle_supervisor.hpp"
#include <chrono>
#include <future>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

VehicleSupervisor::VehicleSupervisor() : Node("vehicle_supervisor")
{
  this->declare_parameter<double>("max_velocity_safety_disabled");
  this->declare_parameter<double>("max_velocity_warning_breached");
  this->declare_parameter<double>("max_acceleration");
  this->declare_parameter<double>("max_velocity");
  this->declare_parameter<double>("wheelbase");
  this->declare_parameter<double>("max_acceleration_angular");
  this->declare_parameter<double>("battery_status_stale_threshold");
  this->declare_parameter<double>("battery_status_check_period");
  this->declare_parameter<std::vector<std::string>>("batteries.ids");

  std::vector<std::string> batteries_ids;
  double batt_stat_check_period;
  bool params_ok = true;
  params_ok &= this->get_parameter("max_velocity_safety_disabled", max_vel_safety_disabled_);
  params_ok &= this->get_parameter("max_velocity_warning_breached", max_vel_warning_breached_);
  params_ok &= this->get_parameter("max_acceleration", max_acc_linear_);
  params_ok &= this->get_parameter("max_acceleration_angular", max_acc_angular_);
  params_ok &= this->get_parameter("max_velocity", max_vel_vehicle_);
  params_ok &= this->get_parameter("wheelbase", wheelbase_);
  params_ok &= this->get_parameter("battery_status_stale_threshold", batt_stat_stale_thres_);
  params_ok &= this->get_parameter("battery_status_check_period", batt_stat_check_period);
  params_ok &= this->get_parameter("batteries.ids", batteries_ids);

  params_ok &= max_acc_linear_ > 0;
  params_ok &= max_vel_vehicle_ > 0;
  params_ok &= max_acc_angular_ > 0;
  params_ok &= wheelbase_ > 0;
  params_ok &= batt_stat_stale_thres_ > 0;
  params_ok &= batt_stat_check_period > 0;

  if (!params_ok)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "ROS parameters not found. Required parameters: max_acceleration, max_velocity, wheelbase.");
    exit(1);
  }

  // --- Initialize Service Clients ---
  client_enable_hv_ = this->create_client<std_srvs::srv::Trigger>("enable_high_voltage");
  client_enable_drives_ = this->create_client<std_srvs::srv::Trigger>("enable_all_drives");
  client_initialize_drives_ = this->create_client<std_srvs::srv::Trigger>("initialize_all_drives");

  for (auto& id : batteries_ids)
  {
    sensor_msgs::msg::BatteryState empty;
    battery_states_.emplace(id, empty);
    subs_battery_states_.emplace(id, this->create_subscription<sensor_msgs::msg::BatteryState>(
                                         id + "/state", 1, [this](const sensor_msgs::msg::BatteryState& msg) {
                                           battery_states_[msg.location] = msg;
                                         }));
  }

  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 1, [this](const geometry_msgs::msg::TwistStamped& msg) { cmd_vel_callback(msg); });

  sub_agv_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "agv_vel", 1, [this](const geometry_msgs::msg::TwistStamped& msg) { agv_vel_ = msg; });

  sub_sick_info_ = this->create_subscription<terraspiro_sick_interface::msg::SickInfo>(
      "sick_info", 1, [this](const terraspiro_sick_interface::msg::SickInfo& msg) {
        sick_vel_limit_ = (msg.front_warning_zone_breached || msg.back_warning_zone_breached)
                              ? max_vel_warning_breached_
                              : max_vel_vehicle_;
        sick_vel_limit_ = msg.safety_override_active ? max_vel_safety_disabled_ : sick_vel_limit_;
      });

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_supervised", 1);

  // --- Initialize Service Server ---
  srv_enable_drives_ = this->create_service<std_srvs::srv::Trigger>(
      "~/enable_drives", std::bind(&VehicleSupervisor::enable_drives_service_callback, this, _1, _2));

  sick_vel_limit_ = batt_vel_limit_ = max_vel_vehicle_;

  timer_battery_states_ = this->create_timer(std::chrono::duration<double>{batt_stat_check_period}, [this]() {
    batt_vel_limit_ = max_vel_vehicle_;
    if (!batteries_ok())
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Not all batteries are present....Stopping vehicle!");
      batt_vel_limit_ = 0.0;
    }
  });

  RCLCPP_INFO(this->get_logger(), "Vehicle Supervisor node has started and is operational.");
}

void VehicleSupervisor::initialize_high_voltage()
{
  while (!client_enable_hv_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the enable_high_voltage_output service. Exiting.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for 'enable_high_voltage_output' service...");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client_enable_hv_->async_send_request(request);

  // Block until the service call completes
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service enable_high_voltage_output. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  auto result = result_future.get();
  if (!result->success)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Service enable_high_voltage_output returned failure: "
                                                << result->message << ". Shutting down.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "High voltage output enabled successfully.");
}

void VehicleSupervisor::cmd_vel_callback(const geometry_msgs::msg::TwistStamped& msg)
{
  if (agv_vel_.header.stamp == rclcpp::Time(0))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No state feedback from the vehicle received yet.");
    return;
  }

  if (active_cmd_vel_.header.stamp == rclcpp::Time(0))
  {
    active_cmd_vel_ = msg;
    return;
  }

  const auto& dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(active_cmd_vel_.header.stamp)).seconds();

  if (dt < 1e-3)
  {
    return;
  }

  double v_x = msg.twist.linear.x;
  double v_y = msg.twist.linear.y;
  double v_lim = std::min(batt_vel_limit_, sick_vel_limit_);

  if (double v_set = std::hypot(v_x, v_y); v_set > v_lim)
  {
    v_x *= v_lim / v_set;
    v_y *= v_lim / v_set;
  }

  double ang_z_set = std::clamp(msg.twist.angular.z, -v_lim / (wheelbase_ / 2), v_lim / (wheelbase_ / 2));
  active_cmd_vel_.header.stamp = msg.header.stamp;
  active_cmd_vel_.twist.linear.x = std::clamp(v_x, v_x - max_acc_linear_ * dt, v_x + max_acc_linear_ * dt);
  active_cmd_vel_.twist.linear.y = std::clamp(v_y, v_y - max_acc_linear_ * dt, v_y + max_acc_linear_ * dt);
  active_cmd_vel_.twist.angular.z = std::clamp(ang_z_set, active_cmd_vel_.twist.angular.z - max_acc_angular_ * dt,
                                               active_cmd_vel_.twist.angular.z + max_acc_angular_ * dt);
  pub_cmd_vel_->publish(active_cmd_vel_);
}

void VehicleSupervisor::enable_drives_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!batteries_ok())
  {
    response->success = false;
    response->message = "Cannot enable drives: Battery is not present.";
    RCLCPP_WARN_STREAM(this->get_logger(), "Rejected enable_drives request: " << response->message);
    return;
  }

  if (!client_enable_drives_->wait_for_service(std::chrono::seconds(1)))
  {
    response->success = false;
    response->message = "Internal error: Drive controller service 'enable_drives' is not available.";
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error: " << response->message);
    return;
  }

  if (!client_initialize_drives_->wait_for_service(std::chrono::seconds(1)))
  {
    response->success = false;
    response->message = "Internal error: Drive controller service 'initialize_drives' is not available.";
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error: " << response->message);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Battery is present. Forwarding 'enable_drives' request to drive controller.");
  auto drive_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_initialize = client_initialize_drives_->async_send_request(drive_request);
  auto result_enable = client_enable_drives_->async_send_request(drive_request);

  // Block this service callback until the downstream service responds (with a timeout)
  response->success = true;
  response->message = "Request for drive enabling sent.";
}

bool VehicleSupervisor::batteries_ok()
{
  bool not_stale{true}, all_present{true};
  for (auto& k_v : battery_states_)
  {
    all_present &= k_v.second.present;
    not_stale &=
        this->get_clock()->now() - k_v.second.header.stamp <= std::chrono::duration<double>{batt_stat_stale_thres_};
  }

  if (!all_present)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Not all batteries are present...Please, check batteries states");
    return false;
  }

  if (!not_stale)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Battery states are stale...Please, check if all batteries are present on the CAN bus!");
    return false;
  }

  return true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleSupervisor>();
  node->initialize_high_voltage();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
