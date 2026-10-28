#ifndef VANGUARD_DRIVER_COMMUNICATION_H
#define VANGUARD_DRIVER_COMMUNICATION_H
#include "vanguard_battery_driver/frame_operations.h"

#include <iostream>
#include <string>
#include <vector>

const static std::vector<uint8_t> CRC_LOOKUP_TABLE = {
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0,
    0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0,
    0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9, 0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A,
    0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0,
    0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD,
    0xD2, 0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14,
    0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92, 0x5F,
    0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79, 0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80,
    0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99,
    0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA,
    0x8B, 0xA4, 0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F, 0xAB,
    0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
    0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC, 0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A,
    0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42,
};

constexpr uint8_t SOURCE_ADDRESS = 0xD0; // VCM Source ID
constexpr uint32_t PGN = 0x1BF3;
constexpr uint8_t PRIORITY = 3;
constexpr float SOC_SCALE = 0.001563f;
constexpr float VOLTAGE_SCALE = 0.05f;
constexpr float CURR_SCALE = 0.05f;

constexpr float SOC_OFFSET = 0.0f;
constexpr float VOLTAGE_OFFSET = 0.0f;
constexpr float CURR_OFFSET = -1600.0f;


/**
 * @brief Computes the CRC for the J1939 message.
 * @param data The 8-byte data payload of the CAN message.
 * @param source_address The source address of the transmitting ECU.
 * @param pgn The PGN of the message sent
 * @return The calculated CRC byte.
 */
inline uint8_t compute_crc(const std::array<uint8_t, 8>& data, const uint8_t source_address, const uint32_t pgn)
{
  std::vector<uint8_t> crc_input;
  crc_input.reserve(11);

  // Copy first 7 bytes of data
  for (size_t i = 0; i < 7; ++i)
  {
    crc_input.push_back(data[i]);
  }

  // Append source address and PGN bytes
  crc_input.push_back(source_address);
  crc_input.push_back(pgn & 0x000000FF);
  crc_input.push_back((pgn & 0x0000FF00) >> 8);
  crc_input.push_back((pgn & 0x00FF0000) >> 16);

  uint8_t crc = 0xFF;
  for (const uint8_t byte : crc_input)
  {
    crc = CRC_LOOKUP_TABLE[crc ^ byte];
  }

  return crc ^ 0xFF;
}

/**
 * @brief Computes the 29-bit arbitration ID.
 * * @return The 29-bit arbitration ID.
 */
inline uint32_t compute_arb_id(const uint8_t priority, const uint32_t pgn, const uint8_t source_address)
{
  return (static_cast<uint32_t>(priority) << 26) | (pgn << 8) | source_address;
}

/**
 * @brief Creates and fills a can_msgs::msg::CanFrame for a J1939 high voltage request.
 * @param source_address The address of the source
 * @param priority The priority of the message
 * @param pgn The parameter group number of parameter group
 * @return A populated can_msgs::msg::CanFrame message.
 */
inline can_msgs::msg::Frame createFrameAndFillHeader(const uint8_t source_address, const uint8_t priority,
                                                     const uint32_t pgn)
{
  can_msgs::msg::Frame frame;
  const uint32_t arbitration_id = compute_arb_id(priority, pgn, source_address);
  frame.is_extended = true;
  frame.id = arbitration_id;
  frame.dlc = 8;
  return frame;
}

/**
 * @brief Extracts the priority, PGN, and source address from a 29-bit arbitration ID.
 * @param[in] arb_id The 29-bit arbitration ID.
 * @param[out] priority The extracted 3-bit priority.
 * @param[out] pgn The extracted 18-bit Parameter Group Number (PGN).
 * @param[out] source_address The extracted 8-bit source address.
 */
inline void extract_arb_id_elements(const uint32_t arb_id, uint8_t& priority, uint32_t& pgn, uint8_t& source_address)
{
  source_address = static_cast<uint8_t>(arb_id & 0xFF);
  pgn = (arb_id >> 8) & 0x3FFFF;
  priority = static_cast<uint8_t>((arb_id >> 26) & 0x07);
}

enum class HighVoltageStatus
{
  DISCONNECTED = 0,
  CONNECTED = 1,
  CONNECTING = 2,
  DISCONNECTING = 3,
  DISCONNECT = 4,
  RESERVED = 13,
  ERROR = 14,
  NOT_AVAILABLE = 15
};

enum class OperationalStatus
{
  INITIALIZATION_IN_PROGRESS = 0,
  INITIALIZATION_COMPLETE = 1,
  OPERATIONAL = 2,
  NORMAL_POWER_DOWN = 3,
  EMERGENCY_POWER_DOWN = 4,
  DISABLED_EXTERNAL = 5,
  DISABLED_INTERNAL = 6,
  RESERVED = 13,
  ERROR = 14,
  NOT_AVAILABLE = 15
};

enum class ParticipantType
{
  UNKNOWN = 0,
  CONTROLLER = 1,
  BATTERY = 2
};

class VanguardBatteryCanCommunication
{
public:
  VanguardBatteryCanCommunication(const std::string& id, const uint8_t source_address, const uint32_t status_pgn,
                                  const uint32_t voltage_pgn, const uint32_t soc_pgn, const std::string type)
      : id_(id), source_address_(source_address), status_pgn_(status_pgn), voltage_pgn_(voltage_pgn), soc_pgn_(soc_pgn)
  {
    type_ = ParticipantType::UNKNOWN;
    if (type == "controller")
    {
      type_ = ParticipantType::CONTROLLER;
    }
    else if (type == "battery")
    {
      type_ = ParticipantType::BATTERY;
    }
  };
  ~VanguardBatteryCanCommunication() = default;

  can_msgs::msg::Frame createHighVoltageRequest(const uint8_t source_address, const int hv_command,
                                                const int op_consent, const int counter) const
  {
    // TODO: highVoltageRequest header 0xC1BF3D0, but by using the doc: 0xC1B00D0, figure out where the difference
    // comes from.
    // TODO: remove hard-coded values after figuring out the difference
    can_msgs::msg::Frame frame = createFrameAndFillHeader(source_address, 3, 7155);

    frame.data[0] = (hv_command & 0x03) | (0x03 << 2) | (0x03 << 4) | (0x03 << 6);
    frame.data[1] = (op_consent == 3) ? 0xFF : 0xDF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = (counter & 0x0F) << 4;

    // Temporarily calculate CRC after filling data
    frame.data[7] = compute_crc(frame.data, source_address, 6912);

    return frame;
  };

  void update(const can_msgs::msg::Frame& frame)
  {
    if (!frame.is_extended)
    {
      return;
    }

    uint8_t source_address, priority;
    uint32_t pgn;

    extract_arb_id_elements(frame.id, priority, pgn, source_address);

    if (source_address != source_address_)
    {
      return;
    }

    if (pgn == status_pgn_)
    {
      high_voltage_status_ = static_cast<HighVoltageStatus>(static_cast<uint8_t>(frame.data[5] & 0x0F));
      operational_status_ = static_cast<OperationalStatus>(static_cast<uint8_t>((frame.data[5] & 0xF0) >> 4));
    }
    else if (pgn == soc_pgn_)
    {
      percentage_ = (frame.data[1] << 8 | frame.data[0]) * SOC_SCALE + SOC_OFFSET;
    }
    else if (pgn == voltage_pgn_)
    {
      current_ = (frame.data[7] << 8 | frame.data[6]) * CURR_SCALE  + CURR_OFFSET;
      voltage_ = (frame.data[5] << 8 | frame.data[4]) * VOLTAGE_SCALE + VOLTAGE_OFFSET;
    }
  }

  OperationalStatus getOperationalState() const { return operational_status_; }

  HighVoltageStatus getHighVoltageStatus() const { return high_voltage_status_; }

  float getPercentage() { return percentage_; }

  float getCurrent() { return current_; }

  float getVoltage() { return voltage_; }

  ParticipantType getParticipantType() const { return type_; }

  std::string getId() const { return id_; }

private:
  std::string id_;
  uint8_t source_address_;
  bool enabled_{false}, error_{false}, initialized_{false};
  uint32_t status_pgn_{0}, voltage_pgn_{0}, soc_pgn_{0};
  OperationalStatus operational_status_{};
  HighVoltageStatus high_voltage_status_{};
  ParticipantType type_{};
  float voltage_{}, current_{}, percentage_{};
};

#endif // VANGUARD_DRIVER_COMMUNICATION_H
