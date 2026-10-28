#ifndef TERRASPIRO_VEHICLE_SUPERVISOR__VEHICLE_SUPERVISOR_HPP_
#define TERRASPIRO_VEHICLE_SUPERVISOR__VEHICLE_SUPERVISOR_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "terraspiro_sick_interface/msg/sick_info.hpp"
#include <memory>

class VehicleSupervisor : public rclcpp::Node
{
public:
  VehicleSupervisor();
  void initialize_high_voltage();

private:
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped& msg);
  void enable_drives_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool batteries_ok();

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_, sub_agv_vel_;
  rclcpp::Subscription<terraspiro_sick_interface::msg::SickInfo>::SharedPtr sub_sick_info_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_enable_hv_, client_enable_drives_, client_initialize_drives_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_drives_;

  std::map<std::string, sensor_msgs::msg::BatteryState> battery_states_;
  terraspiro_sick_interface::msg::SickInfo sick_info_;

  geometry_msgs::msg::TwistStamped agv_vel_, active_cmd_vel_;

  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> subs_battery_states_;

  double max_acc_linear_,        //
      max_vel_safety_disabled_,  //
      max_vel_vehicle_,          //
      max_vel_warning_breached_, //
      sick_vel_limit_,           //
      batt_vel_limit_,           //
      max_acc_angular_,          //
      batt_stat_stale_thres_,          //
      wheelbase_;

  std::shared_ptr<rclcpp::TimerBase> timer_battery_states_;
};
#endif // TERRASPIRO_VEHICLE_SUPERVISOR__VEHICLE_SUPERVISOR_HPP_
