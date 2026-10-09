#ifndef FRAME_OPERATIONS_H
#define FRAME_OPERATIONS_H
#include <can_msgs/msg/frame.hpp>
#include <cstdint>

// Preserve exact acquisition time for ordering. Zero retains transport fallback;
// negative/unnormalized wire timestamps must not throw or update any cache.
inline int64_t sourceStampNanoseconds(const can_msgs::msg::Frame& frame)
{
  const auto& stamp = frame.header.stamp;
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000u) { return -1; }
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

enum FrameOperationsTypes
{
  READ_FROM_DEVICE = 0x40,
  WRITE_TO_DEVICE = 0x23,
  WRITE_TO_DEVICE_4B = 0x23,
  WRITE_TO_DEVICE_2B = 0x2B,
  WRITE_TO_DEVICE_1B = 0x2F,
  REPLY_FROM_DEVICE = 0x43,
  READ_REPLY_4B = 0x43,
  READ_REPLY_3B = 0x47,
  READ_REPLY_2B = 0x4B,
  READ_REPLY_1B = 0x4F,
  WRITE_REPLY = 0x60,
  ABORT_REPLY = 0x80
};

inline bool isReadReplyFromDevice(const can_msgs::msg::Frame& frame)
{
  return frame.data[0] == READ_REPLY_4B || frame.data[0] == READ_REPLY_3B || frame.data[0] == READ_REPLY_2B ||
         frame.data[0] == READ_REPLY_1B;
}

// Write 2 byte message to frame at index as: [0xabcd -> | index: 0xcd | index + 1: 0xab |]
inline void shortToFrame(can_msgs::msg::Frame& frame, unsigned index, short message, bool reverse = true)
{
  if (reverse)
  {
    frame.data[index] = message & 0x00ff;
    frame.data[index + 1] = (message & 0xff00) >> 8;
  }
  else
  {
    frame.data[index] = (message & 0xff00) >> 8;
    frame.data[index + 1] = (message & 0x00ff);
  }
}

// Get 2 byte message from frame at index as: [|index: 0xcd | index:0xab | -> 0xabcd]
inline short shortFromFrame(const can_msgs::msg::Frame& frame, const unsigned& index, const bool& reverse = true)
{
  if (reverse)
    return frame.data[index + 1] << 8 | frame.data[index];
  else
    return frame.data[index] << 8 | frame.data[index + 1];
}

// Get 4 byte message from frame at index as: [|index: 0xgh | index:0xef | index: 0xcd | index:0xab | -> 0xabcdefgh]
inline int intFromFrame(const can_msgs::msg::Frame& frame, const unsigned& index, const bool& reverse = true)
{
  if (reverse)
    return frame.data[index + 3] << 24 | frame.data[index + 2] << 16 | frame.data[index + 1] << 8 | frame.data[index];
  else
    return frame.data[index] << 24 | frame.data[index + 1] << 16 | frame.data[index + 2] << 8 | frame.data[index + 3];
}

inline void intToFrame(can_msgs::msg::Frame& frame, const unsigned& index, const int& message, const bool& reverse = true)
{
  if (reverse)
  {
    frame.data[index + 0] = message & 0xFF;
    frame.data[index + 1] = (message >> 8) & 0xFF;
    frame.data[index + 2] = (message >> 16) & 0xFF;
    frame.data[index + 3] = (message >> 24) & 0xFF;
  }
  else
  {
    frame.data[index + 3] = message & 0xFF;
    frame.data[index + 2] = (message >> 8) & 0xFF;
    frame.data[index + 1] = (message >> 16) & 0xFF;
    frame.data[index + 0] = (message >> 24) & 0xFF;
  }
}

#endif // FRAME_OPERATIONS_H
