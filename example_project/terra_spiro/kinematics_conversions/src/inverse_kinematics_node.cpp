#include <Eigen/Dense>
#include <array>
#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

//! Unwrap angle to closest value within range of target
inline double unwrapToTarget(double angle, double target, double range = M_PI)
{
  // Naive solution assumes that the difference between the angle and
  // the target will never be huge.

  while (angle - target > range / 2)
    angle -= range;
  while (angle - target < -range / 2)
    angle += range;

  return angle;
}

class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics() : Node("kinematics")
  {
    this->declare_parameter("wheelbase", 0.0);
    this->declare_parameter("track", 0.0);
    this->declare_parameter("wheel_radius", 0.0);
    this->declare_parameter("max_velocity_steering", 0.0);
    this->declare_parameter("max_velocity_traction", 0.0);
    this->declare_parameter("max_current_traction", 0.0);
    this->declare_parameter("wheel_mount_angle_reordered", std::vector<double>{});

    bool params_ok{true};
    // wheelbase is the distance between the front and rear wheels
    // track is the distance between the left and right wheels
    // A symmetric configuration is assumed.
    params_ok &= this->get_parameter("wheelbase", wheelbase_);
    params_ok &= this->get_parameter("track", track_);
    params_ok &= this->get_parameter("wheel_radius", wheel_radius_);
    params_ok &= this->get_parameter("max_velocity_steering", max_vel_steer_);
    params_ok &= this->get_parameter("max_velocity_traction", limit_vel_traction_);
    params_ok &= this->get_parameter("max_current_traction", max_cur_traction_);
    params_ok &= this->get_parameter("wheel_mount_angle_reordered", mount_angles_);

    if (!params_ok)
    {
      RCLCPP_FATAL(this->get_logger(), "Parameter(s) 'wheelbase', 'track', 'wheel_radius', 'max_velocity_steering' "
                                       "or 'max_velocity_traction' or 'max_current_traction' or 'wheel_mount_angles' "
                                       "required but not set. Aborting.");
      rclcpp::shutdown();
    }

    if (wheelbase_ == 0.0 or track_ == 0.0 or wheel_radius_ == 0.0)
    {
      RCLCPP_FATAL(this->get_logger(), "Read default parameter value of 0.0 for wheelbase, track or wheel_radius. "
                                       "Please set the parameters explicitly to correct values!");
      rclcpp::shutdown();
    }

    if (mount_angles_.size() != NUM_WHEELS)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Please provide mount angles for exactly " << NUM_WHEELS << " wheels.");
      rclcpp::shutdown();
    }

    // Later code assumes the sizes of steering joint names and traction joint names are equal.
    static_assert(STEERING_JOINT_NAMES.size() == TRACTION_JOINT_NAMES.size(),
                  "Steering joints and traction joints must have the same size");

    // Set up initial joint states
    auto N = NUM_WHEELS * 2;
    joint_setpoints_.name.reserve(N);
    joint_setpoints_.name.insert(joint_setpoints_.name.end(), STEERING_JOINT_NAMES.begin(), STEERING_JOINT_NAMES.end());
    joint_setpoints_.name.insert(joint_setpoints_.name.end(), TRACTION_JOINT_NAMES.begin(), TRACTION_JOINT_NAMES.end());
    joint_setpoints_.position.resize(N, 0.);
    joint_setpoints_.velocity.resize(N, 0.);
    joint_setpoints_.effort.resize(N, 0.);

    // Initialize the inverse kinematics matrix with wheel positions.
    // A fully symmetric configuration is assumed.
    H_ = Eigen::MatrixXd(NUM_WHEELS * 2, 3);
    H_ << 1, 0, track_ / 2,      // Front-right y
          0, 1, wheelbase_ / 2,  // Front-right x
          1, 0, -track_ / 2,     // Rear-left y
          0, 1, -wheelbase_ / 2; // Rear-left x

    srv_enable_ = this->create_service<std_srvs::srv::SetBool>(
        "enable_joystick",
        std::bind(&InverseKinematics::handle_enable_disable, this, std::placeholders::_1, std::placeholders::_2));
    srv_disable_ = this->create_service<std_srvs::srv::Trigger>(
        "disable_joystick",
        std::bind(&InverseKinematics::handle_disable, this, std::placeholders::_1, std::placeholders::_2));

    joint_setpoint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_setpoints", rclcpp::QoS(rclcpp::KeepLast(1)));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](sensor_msgs::msg::JointState msg) { jointStateCallback(msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](geometry_msgs::msg::TwistStamped msg) { cmdVelCallback(msg); });
  }

  void jointStateCallback(const sensor_msgs::msg::JointState& msg)
  {
    // Make sure joint ordering corresponds to our internal vehicle representation.
    joint_states_ = reorderJointStates(msg);
    // Mounting angles are added to measured values to reflect the actual orientation of the wheel on the vehicle.
    std::transform(joint_states_.position.begin(), joint_states_.position.begin() + NUM_WHEELS,
                   mount_angles_.begin(), joint_states_.position.begin(), std::plus());

    // We are publishing here, to stay synchronized with the feedback.
    // It does not make sense to publish more often than feedback is received.
    if (computed_joint_setpoints_ && is_enabled_)
      joint_setpoint_pub_->publish(joint_setpoints_);
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped& msg)
  {
    // We will not process commands until we have received the first feedback.
    // This is necessary to obtain initial steering values, around which
    // we will generate setpoints.
    if (joint_states_.name.empty())
      return;

    // The default action is to keep all joints at their current positions
    // and set velocities and efforts to 0.
    std::vector<double> positions(joint_states_.position);
    // Mounting angles are subtracted from setpoints to compensate for inaccurate mounting.
    std::transform(positions.begin(), positions.begin() + NUM_WHEELS, mount_angles_.begin(),
                   positions.begin(), std::minus());
    std::vector<double> velocities(NUM_WHEELS * 2, 0.0);

    Eigen::Vector3d body_velocity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z);
    if (body_velocity.norm() > MIN_VELOCITY)
    {
      // We are computing kinematics only if the body velocity is above a minimal threshold,
      // to avoid numeric ambiguities close to 0.

      // Compute wheel kinematics

      Eigen::VectorXd wheel_velocities = H_ * body_velocity;
      for (unsigned i = 0; i < NUM_WHEELS; ++i)
      {
        Eigen::Vector2d v_i(wheel_velocities(2 * i), wheel_velocities(2 * i + 1));
        positions[i] = unwrapToTarget(std::atan2(v_i(1), v_i(0)), joint_states_.position[i], 2 * M_PI);
        velocities[i] = max_vel_steer_;
        velocities[i + NUM_WHEELS] = v_i.norm() / wheel_radius_;
      }
      // Mounting angles are subtracted from setpoints to compensate for inaccurate mounting.
      std::transform(positions.begin(), positions.begin() + NUM_WHEELS, mount_angles_.begin(),
                     positions.begin(), std::minus());
      // Flip velocity signs on reverse-mounted wheels.
      std::transform(velocities.begin(), velocities.begin() + NUM_WHEELS, mount_angles_.begin(),
                     velocities.begin(),
                     [](double velocity, double angle) { return velocity * std::copysign(1.0, cos(angle)); });
      enforceContinuity(positions, velocities, joint_setpoints_);
    }
    joint_setpoints_.position = positions;
    joint_setpoints_.velocity = velocities;

    // Also provide traction current setpoints (same value for all traction motors),
    // if the driver is configured to control motors in torque mode.
    setCurrentsFromVelocities();

    // Prepare the message with setpoints for sending
    joint_setpoints_.header = msg.header;
    computed_joint_setpoints_ = true;
  }

  // Enforces continuity of positions and velocities wrt. joint_states.position and joint_states.velocity.
  // joint_states may be the previous setpoint values or the current feedback values.
  void enforceContinuity(std::vector<double>& positions,  //
                         std::vector<double>& velocities, //
                         const sensor_msgs::msg::JointState& joint_states)
  {

    // If the inverse kinematics solution returns large steering changes
    // prefer keeping the steering direction and just flipping the velocity.
    for (unsigned i = 0; i < NUM_WHEELS; ++i)
    {
      while (positions[i] - joint_states.position[i] > M_PI / 2.)
      {
        positions[i] -= M_PI;
        velocities[NUM_WHEELS + i] *= -1;
      }
      while (positions[i] - joint_states.position[i] < -M_PI / 2.)
      {
        positions[i] += M_PI;
        velocities[NUM_WHEELS + i] *= -1;
      }
    }
  }

  // Make sure that the joint ordering corresponds to our internal representation
  // ROS Control reorders joint states alphabetically, see
  // https://github.com/ros-controls/ros2_controllers/issues/159
  sensor_msgs::msg::JointState reorderJointStates(const sensor_msgs::msg::JointState& joint_states)
  {
    sensor_msgs::msg::JointState reordered(joint_states);
    if (joint_states.name.size() != 2 * NUM_WHEELS)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Number of joint states ("
                                                  << joint_states.name.size()
                                                  << ") does not match expected number of joints (" << 2 * NUM_WHEELS
                                                  << ")!");
      rclcpp::shutdown();
    }
    auto assign_joint_data = [&](const std::string& joint_name, size_t idx) {
      size_t joint_idx = std::distance(joint_states.name.begin(),
                                       std::find(joint_states.name.begin(), joint_states.name.end(), joint_name));
      if (joint_idx >= joint_states.name.size())
      {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Joint " << joint_name << " not found in feedback message!");
        rclcpp::shutdown();
      }
      reordered.name[idx] = joint_name;
      reordered.position[idx] = joint_states.position[joint_idx];
      reordered.velocity[idx] = joint_states.velocity[joint_idx];
      reordered.effort[idx] = joint_states.effort[joint_idx];
    };

    for (unsigned i = 0; i < NUM_WHEELS; ++i)
    {
      assign_joint_data(STEERING_JOINT_NAMES[i], i);
      assign_joint_data(TRACTION_JOINT_NAMES[i], NUM_WHEELS + i);
    }
    return reordered;
  }

  // Compute and set currents for torque mode control.
  // We are using a trivial approach, which sets the same current setpoints
  // for all traction wheels. The setpoint is (arbitratily) chosen so as to be
  // proportional to the velocity of the fastest wheel. Velocity signs are respected.
  void setCurrentsFromVelocities(void)
  {
    // Find maximum velocity (by absolute value)
    double max_vel = 0.0;
    for (unsigned i = 0; i < NUM_WHEELS; i++)
    {
      if (fabs(joint_setpoints_.velocity[NUM_WHEELS + i]) > max_vel)
        max_vel = fabs(joint_setpoints_.velocity[NUM_WHEELS + i]);
    }

    // Apply current proportional to maximum velocity to all wheels
    for (unsigned i = 0; i < NUM_WHEELS; i++)
    {
      double vel = max_cur_traction_ * max_vel / limit_vel_traction_;
      joint_setpoints_.effort[NUM_WHEELS + i] = std::copysign(vel, joint_setpoints_.velocity[NUM_WHEELS + i]);
    }
  }

private:
  void setJoystickEnabled(const bool enabled, std::string& message)
  {
    if (enabled)
    {
      if (is_enabled_)
      {
        message = "Node is already enabled.";
      }
      else
      {
        is_enabled_ = true;
        message = "Node has been enabled successfully.";
      }
      return;
    }

    if (!is_enabled_)
    {
      message = "Node is already disabled.";
    }
    else
    {
      is_enabled_ = false;
      message = "Node has been disabled successfully.";
    }
  }

  void handle_enable_disable(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    setJoystickEnabled(request->data, response->message);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void handle_disable(
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    setJoystickEnabled(false, response->message);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_setpoint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disable_;
  sensor_msgs::msg::JointState joint_setpoints_, joint_states_;
  Eigen::MatrixXd H_; // Inverse kinematics Matrix H (from Kelly, 2010.)
  inline static constexpr std::array STEERING_JOINT_NAMES = {"front_steering", "rear_steering"};
  inline static constexpr std::array TRACTION_JOINT_NAMES = {"front_traction", "rear_traction"};
  inline static constexpr unsigned NUM_WHEELS{STEERING_JOINT_NAMES.size()};
  inline static constexpr double MIN_VELOCITY{1.e-3};
  double wheelbase_{}; // Distance between front and rear wheels
  double track_{};     // Distance between left and right wheels
  double wheel_radius_{}, max_vel_steer_{}, limit_vel_traction_{}, max_cur_traction_{};
  std::vector<double> mount_angles_{0.0, 0.0};
  bool computed_joint_setpoints_{false}, is_enabled_{true};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InverseKinematics>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
