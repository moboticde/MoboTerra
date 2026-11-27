#include <mobotic_driver/virtual_can_fleet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class VirtualCanFleetNode final : public rclcpp::Node
{
public:
  VirtualCanFleetNode() : Node("virtual_can_fleet"), last_update_(now())
  {
    const auto ids = declare_parameter<std::vector<std::string>>("node_ids", std::vector<std::string>{});
    const auto roles = declare_parameter<std::vector<std::string>>("drive_types", std::vector<std::string>{});
    const auto controllers = declare_parameter<std::vector<std::string>>("controller_types", std::vector<std::string>{});
    const auto resolutions = declare_parameter<std::vector<double>>("resolutions", std::vector<double>{});
    const auto gearing = declare_parameter<std::vector<double>>("gear_ratios", std::vector<double>{});
    if (ids.empty() || roles.size() != ids.size() || controllers.size() != ids.size() ||
        resolutions.size() != ids.size() || gearing.size() != ids.size())
    { throw std::invalid_argument("virtual fleet requires equally sized nonempty model arrays"); }
    for (std::size_t i = 0; i < ids.size(); ++i)
    {
      std::size_t consumed = 0;
      const auto id = std::stoul(ids[i], &consumed, 16);
      if (consumed != ids[i].size() || id == 0 || id > 127 ||
          (roles[i] != "steering" && roles[i] != "traction"))
      { throw std::invalid_argument("invalid virtual fleet CAN node or drive role"); }
      fleet_.add(static_cast<unsigned>(id), roles[i] == "traction", resolutions[i], gearing[i], controllers[i]);
    }
    publisher_ = create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(1000));
    subscription_ = create_subscription<can_msgs::msg::Frame>("can_rx", rclcpp::QoS(1000),
      [this](const can_msgs::msg::Frame& request) {
        advance();
        for (auto reply : fleet_.handle(request))
        { reply.header.frame_id = "can"; reply.header.stamp = now(); publisher_->publish(reply); }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { advance(); });
    RCLCPP_WARN(get_logger(), "Virtual CAN fleet: %zu independent synthetic controller models; no physical CAN", ids.size());
  }
private:
  void advance()
  {
    const auto current = now();
    fleet_.advance(std::clamp((current - last_update_).seconds(), 0.0, 0.1));
    last_update_ = current;
  }
  VirtualCanFleet fleet_;
  rclcpp::Time last_update_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualCanFleetNode>());
  rclcpp::shutdown();
  return 0;
}
