#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mobotic_kinematics/joint_feedback_cache.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr double PI = 3.14159265358979323846;

bool finiteTwist(const geometry_msgs::msg::Twist& twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.angular.z);
}

}  // namespace

class MoboticKinematics : public rclcpp::Node
{
public:
  MoboticKinematics() : Node("mobotic_kinematics")
  {
    this->declare_parameter("module_names", std::vector<std::string>{});
    this->declare_parameter("steering_joint_names", std::vector<std::string>{});
    this->declare_parameter("traction_joint_names", std::vector<std::string>{});
    this->declare_parameter("wheels_x", std::vector<double>{});
    this->declare_parameter("wheels_y", std::vector<double>{});
    this->declare_parameter("wheel_mount_angles", std::vector<double>{});
    this->declare_parameter("wheel_radius", 0.0);
    this->declare_parameter("max_steering_velocity", 0.0);
    this->declare_parameter("max_traction_velocity", 0.0);
    this->declare_parameter("max_traction_current", 0.0);
    this->declare_parameter("traction_control_mode", std::string("velocity"));
    this->declare_parameter("cmd_vel_timeout", 0.5);
    this->declare_parameter("joint_feedback_timeout", 0.3);
    this->declare_parameter("minimum_linear_speed", 1.0e-3);
    this->declare_parameter("output_frame_id", std::string("base_link"));

    this->get_parameter("module_names", module_names_);
    this->get_parameter("steering_joint_names", steering_joint_names_);
    this->get_parameter("traction_joint_names", traction_joint_names_);
    this->get_parameter("wheels_x", wheels_x_);
    this->get_parameter("wheels_y", wheels_y_);
    this->get_parameter("wheel_mount_angles", wheel_mount_angles_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("max_steering_velocity", max_steering_velocity_);
    this->get_parameter("max_traction_velocity", max_traction_velocity_);
    this->get_parameter("max_traction_current", max_traction_current_);
    this->get_parameter("traction_control_mode", traction_control_mode_);
    this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
    this->get_parameter("joint_feedback_timeout", joint_feedback_timeout_);
    this->get_parameter("minimum_linear_speed", minimum_linear_speed_);
    this->get_parameter("output_frame_id", output_frame_id_);

    validateParameters();
    buildKinematicsMatrix();
    joint_feedback_ = std::make_unique<JointFeedbackCache>(
        steering_joint_names_, traction_joint_names_, joint_feedback_timeout_);

    command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "kinematics/joint_setpoints", rclcpp::QoS(rclcpp::KeepLast(1)));
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "agv_vel", rclcpp::QoS(rclcpp::KeepLast(1)));

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const geometry_msgs::msg::TwistStamped& msg) { cmdVelCallback(msg); });
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const sensor_msgs::msg::JointState& msg) { jointStateCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Configured %zu wheel modules in %s traction mode.",
                module_names_.size(), traction_control_mode_.c_str());
  }

private:
  void validateParameters()
  {
    const std::size_t count = module_names_.size();
    if (count < 2 || steering_joint_names_.size() != count ||
        traction_joint_names_.size() != count || wheels_x_.size() != count ||
        wheels_y_.size() != count || wheel_mount_angles_.size() != count)
    {
      throw std::invalid_argument(
          "module_names, joint names, wheel coordinates, and mount angles must have the same size (at least two)");
    }

    std::set<std::string> module_names(module_names_.begin(), module_names_.end());
    std::set<std::string> steering_names(steering_joint_names_.begin(), steering_joint_names_.end());
    std::set<std::string> traction_names(traction_joint_names_.begin(), traction_joint_names_.end());
    std::set<std::string> all_joint_names(steering_joint_names_.begin(), steering_joint_names_.end());
    all_joint_names.insert(traction_joint_names_.begin(), traction_joint_names_.end());
    if (module_names.size() != count || steering_names.size() != count ||
        traction_names.size() != count || all_joint_names.size() != 2 * count ||
        module_names.count("") != 0 ||
        steering_names.count("") != 0 || traction_names.count("") != 0)
    {
      throw std::invalid_argument("module and joint names must be non-empty and unique");
    }

    for (std::size_t i = 0; i < count; ++i)
    {
      if (!std::isfinite(wheels_x_[i]) || !std::isfinite(wheels_y_[i]) ||
          !std::isfinite(wheel_mount_angles_[i]))
      {
        throw std::invalid_argument("wheel coordinates and mount angles must be finite");
      }
    }

    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
        !std::isfinite(max_steering_velocity_) || max_steering_velocity_ <= 0.0 ||
        !std::isfinite(max_traction_velocity_) || max_traction_velocity_ <= 0.0 ||
        !std::isfinite(cmd_vel_timeout_) || cmd_vel_timeout_ <= 0.0 ||
        !std::isfinite(joint_feedback_timeout_) || joint_feedback_timeout_ <= 0.0 ||
        !std::isfinite(minimum_linear_speed_) || minimum_linear_speed_ < 0.0)
    {
      throw std::invalid_argument("wheel radius, velocity limits, and command/feedback timeouts must be positive");
    }

    if (traction_control_mode_ != "velocity" && traction_control_mode_ != "current")
    {
      throw std::invalid_argument("traction_control_mode must be 'velocity' or 'current'");
    }
    if (traction_control_mode_ == "current" &&
        (!std::isfinite(max_traction_current_) || max_traction_current_ <= 0.0))
    {
      throw std::invalid_argument("max_traction_current must be positive in current mode");
    }
  }

  void buildKinematicsMatrix()
  {
    kinematics_matrix_ = Eigen::MatrixXd::Zero(module_names_.size() * 2, 3);
    for (std::size_t i = 0; i < module_names_.size(); ++i)
    {
      kinematics_matrix_.row(2 * i) << 1.0, 0.0, -wheels_y_[i];
      kinematics_matrix_.row(2 * i + 1) << 0.0, 1.0, wheels_x_[i];
    }

    if (kinematics_matrix_.fullPivLu().rank() < 3)
    {
      throw std::invalid_argument("wheel geometry does not constrain all planar velocity axes");
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState& msg)
  {
    const auto received_at = JointFeedbackCache::Clock::now();
    const auto now_ns = this->now().nanoseconds();
    // Reject malformed wire timestamps without throwing from rclcpp::Time.
    const bool stamp_well_formed = msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1'000'000'000u;
    const int64_t stamp_ns = stamp_well_formed
        ? static_cast<int64_t>(msg.header.stamp.sec) * 1'000'000'000 + msg.header.stamp.nanosec
        : 0;
    Eigen::Vector3d body_velocity = Eigen::Vector3d::Zero();
    const bool accepted = joint_feedback_->update(
        msg.name, msg.position, msg.velocity, stamp_ns, now_ns,
        [this, &body_velocity](const std::vector<double>& positions, const std::vector<double>& velocities) {
          Eigen::VectorXd wheel_velocity(module_names_.size() * 2);
          for (std::size_t i = 0; i < module_names_.size(); ++i)
          {
            const double physical_angle = positions[i] + wheel_mount_angles_[i];
            const double linear_speed = velocities[i] * wheel_radius_;
            wheel_velocity(2 * i) = linear_speed * std::cos(physical_angle);
            wheel_velocity(2 * i + 1) = linear_speed * std::sin(physical_angle);
          }
          if (!wheel_velocity.allFinite()) { return false; }
          body_velocity = kinematics_matrix_.completeOrthogonalDecomposition().solve(wheel_velocity);
          return body_velocity.allFinite();
        }, received_at);
    if (!accepted)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Rejecting incomplete, invalid, stale or non-newer joint feedback.");
      return;
    }

    geometry_msgs::msg::TwistStamped output;
    output.header = msg.header;
    output.header.frame_id = output_frame_id_;
    output.twist.linear.x = body_velocity.x();
    output.twist.linear.y = body_velocity.y();
    output.twist.angular.z = body_velocity.z();
    velocity_publisher_->publish(output);
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped& msg)
  {
    if (!joint_feedback_->fresh(this->now().nanoseconds()))
    {
      joint_feedback_->invalidate();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Ignoring cmd_vel until fresh, complete joint feedback is available.");
      return;
    }
    if (!finiteTwist(msg.twist))
    {
      RCLCPP_WARN(this->get_logger(), "Rejecting cmd_vel containing non-finite values.");
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Rejecting cmd_vel without a timestamp.");
      return;
    }
    const double age = (this->now() - stamp).seconds();
    if (age > cmd_vel_timeout_ || age < -0.1)
    {
      RCLCPP_WARN(this->get_logger(), "Rejecting cmd_vel with invalid age %.3f s.", age);
      return;
    }

    const auto& steering_positions = joint_feedback_->steeringPositions();
    std::vector<double> steering_targets(steering_positions);
    std::vector<double> steering_velocities(module_names_.size(), 0.0);
    std::vector<double> traction_velocities(module_names_.size(), 0.0);

    for (std::size_t i = 0; i < module_names_.size(); ++i)
    {
      const double velocity_x = msg.twist.linear.x - msg.twist.angular.z * wheels_y_[i];
      const double velocity_y = msg.twist.linear.y + msg.twist.angular.z * wheels_x_[i];
      double linear_speed = std::hypot(velocity_x, velocity_y);
      if (linear_speed <= minimum_linear_speed_)
      {
        continue;
      }

      const double reference_angle = steering_positions[i] + wheel_mount_angles_[i];
      double angle_delta = std::remainder(std::atan2(velocity_y, velocity_x) - reference_angle, 2.0 * PI);
      if (angle_delta > PI / 2.0)
      {
        angle_delta -= PI;
        linear_speed = -linear_speed;
      }
      else if (angle_delta < -PI / 2.0)
      {
        angle_delta += PI;
        linear_speed = -linear_speed;
      }

      steering_targets[i] = reference_angle + angle_delta - wheel_mount_angles_[i];
      steering_velocities[i] = max_steering_velocity_;
      traction_velocities[i] = linear_speed / wheel_radius_;
    }

    double maximum_requested_velocity = 0.0;
    for (const double velocity : traction_velocities)
    {
      maximum_requested_velocity = std::max(maximum_requested_velocity, std::abs(velocity));
    }
    if (maximum_requested_velocity > max_traction_velocity_)
    {
      const double scale = max_traction_velocity_ / maximum_requested_velocity;
      for (double& velocity : traction_velocities)
      {
        velocity *= scale;
      }
      maximum_requested_velocity = max_traction_velocity_;
    }

    sensor_msgs::msg::JointState output;
    output.header = msg.header;
    const bool current_mode = traction_control_mode_ == "current";
    const double current_magnitude = current_mode
                                         ? max_traction_current_ * maximum_requested_velocity /
                                               max_traction_velocity_
                                         : 0.0;

    for (std::size_t i = 0; i < module_names_.size(); ++i)
    {
      output.name.push_back(steering_joint_names_[i]);
      output.position.push_back(steering_targets[i]);
      output.velocity.push_back(steering_velocities[i]);
      output.effort.push_back(0.0);

      output.name.push_back(traction_joint_names_[i]);
      output.position.push_back(0.0);
      output.velocity.push_back(current_mode ? 0.0 : traction_velocities[i]);
      output.effort.push_back(
          current_mode && std::abs(traction_velocities[i]) > 0.0
              ? std::copysign(current_magnitude, traction_velocities[i])
              : 0.0);
    }

    command_publisher_->publish(output);
  }

  std::vector<std::string> module_names_;
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> traction_joint_names_;
  std::vector<double> wheels_x_;
  std::vector<double> wheels_y_;
  std::vector<double> wheel_mount_angles_;
  std::unique_ptr<JointFeedbackCache> joint_feedback_;
  Eigen::MatrixXd kinematics_matrix_;

  double wheel_radius_{0.0};
  double max_steering_velocity_{0.0};
  double max_traction_velocity_{0.0};
  double max_traction_current_{0.0};
  double cmd_vel_timeout_{0.5};
  double joint_feedback_timeout_{0.3};
  double minimum_linear_speed_{1.0e-3};
  std::string traction_control_mode_;
  std::string output_frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoboticKinematics>());
  rclcpp::shutdown();
  return 0;
}
