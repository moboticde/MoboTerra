#include <array>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "mobotic_scanner_projection/project.hpp"

class ScannerEvaluationProjection : public rclcpp::Node
{
  using Raw = sick_safetyscanners2_interfaces::msg::RawMicroScanData;
public:
  ScannerEvaluationProjection() : Node("scanner_evaluation_projection")
  {
    const std::array<std::string, 2> names{"front_left", "rear_right"};
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      const auto prefix = "scanner/" + names[i];
      publishers_[i] = create_publisher<Raw>(prefix + "/evaluation_data", qos);
      subscriptions_[i] = create_subscription<Raw>(prefix + "/raw_data", qos,
          [publisher = publishers_[i]](Raw::ConstSharedPtr message) {
            publisher->publish(projectEvaluation(*message));
          });
    }
  }
private:
  std::array<rclcpp::Publisher<Raw>::SharedPtr, 2> publishers_;
  std::array<rclcpp::Subscription<Raw>::SharedPtr, 2> subscriptions_;
};
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScannerEvaluationProjection>());
  rclcpp::shutdown();
}
