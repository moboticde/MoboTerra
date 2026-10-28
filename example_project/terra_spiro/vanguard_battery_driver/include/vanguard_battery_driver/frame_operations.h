#ifndef FRAME_OPERATIONS_H
#define FRAME_OPERATIONS_H
#include <can_msgs/msg/frame.hpp>

enum FrameOperationsTypes
{
  READ_FROM_DEVICE = 0x40,
  WRITE_TO_DEVICE = 0x23,
  REPLY_FROM_DEVICE = 0x43
};

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