#include <mobotic_driver/can_scheduling.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace
{
int64_t writeQueueAge(double age)
{
  if (!std::isfinite(age) || age <= 0 || age > .05)
  { throw std::invalid_argument("SDO sender write queue age must be positive and at most 50 ms"); }
  return static_cast<int64_t>(age * 1e9);
}
}

PacedSocketCanSender::PacedSocketCanSender(const rclcpp::NodeOptions& options, Transport transport)
  : rclcpp::Node("socket_can_sender", options), transport_(std::move(transport)),
    queue_(writeQueueAge(declare_parameter<double>("max_queue_age_sec", .05)))
{
  const auto interface = declare_parameter<std::string>("interface", "can0");
  if (!transport_) { openSocket(interface); }
  // Observation must not make a slow recorder backpressure the control path.
  accepted_ = create_publisher<can_msgs::msg::Frame>("can_tx_sent", rclcpp::QoS(128).best_effort());
  status_ = create_publisher<std_msgs::msg::String>("can_tx_status", rclcpp::QoS(10).best_effort());
  subscription_ = create_subscription<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(64),
      [this](const can_msgs::msg::Frame& frame) {
        if (!queue_.push(frame, now().nanoseconds()))
        { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN Tx request rejected: invalid, expired or queue full"); }
        drain();
      });
  timer_ = create_wall_timer(CanTxQueue::minimum_gap, [this]() { drain(); });
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    std::ostringstream out;
    out << "{\"sent\":" << queue_.sent() << ",\"dropped\":" << queue_.dropped()
        << ",\"invalid\":" << queue_.invalid() << ",\"expired\":" << queue_.expired()
        << ",\"capacity_drops\":" << queue_.capacityDrops()
        << ",\"coalesced\":" << queue_.coalesced() << ",\"revoked\":" << queue_.revoked()
        << ",\"send_failures\":" << queue_.failures()
        << ",\"configured_gap_ms\":" << can_scheduling::minimum_gap.count()
        << ",\"queued\":" << queue_.size() << ",\"minimum_socket_gap_ms\":" << min_gap_ms_
        << ",\"maximum_source_age_ms\":" << max_queue_age_ms_ << '}';
    std_msgs::msg::String message; message.data = out.str(); status_->publish(message);
  });
  RCLCPP_INFO(get_logger(), "SDO-only CAN sender on %s: shared %.1f ms socket-attempt gap, bounded queue, no catch-up; Tx observations on can_tx_sent/status", interface.c_str(),
      std::chrono::duration<double, std::milli>(can_scheduling::minimum_gap).count());
}

void PacedSocketCanSender::openSocket(const std::string& interface)
{
  if (interface.empty() || interface.size() >= IFNAMSIZ) { throw std::invalid_argument("Invalid CAN interface name"); }
  socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (socket_ < 0) { throw std::system_error(errno, std::generic_category(), "CAN socket"); }
  try
  {
    struct ifreq request{}; std::memcpy(request.ifr_name, interface.c_str(), interface.size());
    if (::ioctl(socket_, SIOCGIFINDEX, &request) < 0)
    { throw std::system_error(errno, std::generic_category(), "CAN interface lookup"); }
    const int loopback = 0;
    if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0 ||
        ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0)
    { throw std::system_error(errno, std::generic_category(), "CAN sender options"); }
    struct sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0)
    { throw std::system_error(errno, std::generic_category(), "CAN bind"); }
    transport_ = [this](const can_msgs::msg::Frame& frame) {
      struct can_frame raw{}; raw.can_id = frame.id; raw.can_dlc = frame.dlc;
      std::copy_n(frame.data.begin(), frame.dlc, raw.data);
      if (::write(socket_, &raw, CAN_MTU) != CAN_MTU)
      { throw std::system_error(errno, std::generic_category(), "CAN nonblocking send"); }
    };
  }
  catch (...) { ::close(socket_); socket_ = -1; throw; }
}

PacedSocketCanSender::~PacedSocketCanSender()
{ if (socket_ >= 0) { ::close(socket_); } }

void PacedSocketCanSender::drain()
{
  const auto ros_now = now();
  const auto frame = queue_.next(ros_now.nanoseconds());
  if (!frame) { return; }
  try
  {
    const auto before_send = CanTxQueue::Clock::now();
    transport_(*frame);
    const auto after_send = CanTxQueue::Clock::now();
    if (last_success_)
    {
      const double gap = std::chrono::duration<double, std::milli>(before_send - *last_success_).count();
      min_gap_ms_ = min_gap_ms_ == 0 ? gap : std::min(min_gap_ms_, gap);
    }
    last_success_ = after_send;
    queue_.complete(true, after_send);
    max_queue_age_ms_ = std::max(max_queue_age_ms_, (ros_now.nanoseconds() - CanTxQueue::stamp(*frame)) / 1e6);
    // This stamp observes socket acceptance only; it never renews the input
    // request or claims a physical wire/hardware transmission timestamp.
    auto observed = *frame; observed.header.stamp = now(); accepted_->publish(observed);
  }
  catch (const std::exception& error)
  {
    queue_.complete(false);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN send failed; paced retry with expiry: %s", error.what());
  }
  // Anchor after the attempt and all local bookkeeping. No late timer can
  // release multiple queued requests as a catch-up batch.
  if (timer_) { timer_->reset(); }
}

#ifndef MOBOTIC_PACED_CAN_SENDER_NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<PacedSocketCanSender>()); }
  catch (const std::exception& error)
  { std::cerr << "Paced CAN sender failed: " << error.what() << '\n'; result = 1; }
  rclcpp::shutdown(); return result;
}
#endif
