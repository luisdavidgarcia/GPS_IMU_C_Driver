#include <cstdint>
#include <iostream>
#include <map>
#include <stdio.h>
#include <string>
#include <vector>
#include "ubx_msg.h"

/*
 * NAV 0x01 Navigation Results Messages: Position, Speed, Time, Acceleration,
 * Heading, DOP, SVs used RXM 0x02 Receiver Manager Messages: Satellite Status,
 * RTC Status INF 0x04 Information Messages: Printf-Style Messages, with IDs
 * such as Error, Warning, Notice ACK 0x05 Ack/Nak Messages: Acknowledge or
 * Reject messages to UBX-CFG input messages CFG 0x06 Configuration Input
 * Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. UPD 0x09
 * Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash
 * identification, etc. MON 0x0A Monitoring Messages: Communication Status, CPU
 * Load, Stack Usage, Task Status AID 0x0B AssistNow Aiding Messages: Ephemeris,
 * Almanac, other A-GPS data input TIM 0x0D Timing Messages: Time Pulse Output,
 * Time Mark Results ESF 0x10 External Sensor Fusion Messages: External Sensor
 * Measurements and Status Information MGA 0x13 Multiple GNSS Assistance
 * Messages: Assistance data for various GNSS LOG 0x21 Logging Messages: Log
 * creation, deletion, info and retrieval SEC 0x27 Security Feature Messages HNR
 * 0x28 High Rate Navigation Results Messages: High rate time, position, speed,
 * heading
 */

/*
const uint8_t NAV_CLASS = 0x01;
const uint8_t RXM_CLASS = 0x02;
const uint8_t INF_CLASS = 0x04;
const uint8_t ACK_CLASS = 0x05;
const uint8_t CFG_CLASS = 0x06;
const uint8_t UPD_CLASS = 0x09;
const uint8_t MON_CLASS = 0x0A;
const uint8_t AID_CLASS = 0x0B;
const uint8_t TIM_CLASS = 0x0D;
const uint8_t ESF_CLASS = 0x10;
const uint8_t MGA_CLASS = 0x13;
const uint8_t LOG_CLASS = 0x21;
const uint8_t SEC_CLASS = 0x27;
const uint8_t HNR_CLASS = 0x28;

const uint8_t SYNC_CHAR_1 = 0xB5;
const uint8_t SYNC_CHAR_2 = 0x62;

// NAV MESSAGE SECTION
const uint8_t NAV_PVT = 0x07;

// ACK MESSAGE SECTION
const uint8_t ACK_ACK = 0x01;
const uint8_t ACK_NAK = 0x00;

// CFG MESSAGE SECTION
const uint8_t CFG_PRT = 0x00;
const uint8_t CFG_MSG = 0x01;
const uint8_t CFG_RATE = 0x08;

// DEBUG/HELPING CONSTANTS
const int MAX_MESSAGE_LENGTH = 1000;
*/

// GNSS FIX TYPE
const std::map<uint8_t, std::string> GNSS_FIX_TYPE = {
    {0, "no fix"},
    {1, "dead reckoning only"},
    {2, "2D-fix"},
    {3, "3D-fix"},
    {4, "GNSS + dead reckoning combined"},
    {5, "time only fix"}};

/* Primary Functions */
std::string ping(std::string s) {
	std::string temp = "Echo message is: " + s + "\n";
  return temp;
}

/**
 * @brief   Should index into current message and return the payload of the UBX
            message
 * @param   msg      ubx message stored as vector (dynmaic list)
 * @return  Returns the payload of the message as new vector
 */
std::vector<uint8_t> payloadFromMessage(const std::vector<uint8_t> &msg) {
  // Create a vector to hold the payload.
  std::vector<uint8_t> payload;

  // Check if the message is at least 8 bytes long (the minimum size to contain
  // a payload).
  if (msg.size() >= 8) {
    // Iterate from the 7th element (index 6) to the second-to-last element.
    for (size_t i = 6; i < msg.size() - 2; ++i) {
      payload.push_back(msg[i]);
    }
  }
  return payload;
}

/**
 * @brief   Creates a ubx message given message class and id. If a length is
            specified the message will be populated with 0x00. If a payload is
 specified the message will be populated with the payloads content.
 * @param   msg_class   1 byte
 * @param   msg_id      1 byte
 * @param   length      default to 0
 * @param   payload     default to empty vector
 * @return  Returns ubx message as vector
 */

std::vector<uint8_t> composeMessage(uint8_t msg_class, uint8_t msg_id,
                                    uint16_t length,
                                    const std::vector<uint8_t> &payload) {
  // Calculate the payload length as two little-endian bytes
  uint8_t byte_length[2] = {static_cast<uint8_t>(length),
                            static_cast<uint8_t>(length >> 8)};

  // Create the UBX message vector
  std::vector<uint8_t> msg = {SYNC_CHAR_1, SYNC_CHAR_2,    msg_class,
                              msg_id,      byte_length[0], byte_length[1]};

  if (payload.empty() && length > 0) {
    // Append an empty payload filled with zeros
    std::vector<uint8_t> zeros(length, 0);
    msg.insert(msg.end(), zeros.begin(), zeros.end());
  } else {
    // Append the provided payload
    msg.insert(msg.end(), payload.begin(), payload.end());
  }

  /*
  // Compute and append the checksum
  auto checksum = computeChecksum({msg.begin() + 2, msg.end()});
  msg.push_back(checksum.first);
  msg.push_back(checksum.second);
  */

  return msg;
}

/**
 * @brief   Calculates and returns checksum based on ubs message input
 * @param   msg   ubx message stored as vector (dynmaic list)
 * @return  Returns two checksum values as tuple
 */
std::pair<uint8_t, uint8_t> computeChecksum(const std::vector<uint8_t> &msg) {
  uint8_t ck_a = 0x00;
  uint8_t ck_b = 0x00;

  for (const uint8_t byte : msg) {
    ck_a += byte;
    ck_b += ck_a;
    ck_a &= 0xFF;
    ck_b &= 0xFF;
  }

  return {ck_a, ck_b};
}

/**
 * @brief   Converts msg class into user readable string
 * @param   msg_cls   msg class is in byte or numerical form
 * @return  Returns string form of msg class
 */
std::string msgClassToString(uint8_t msg_cls) {
  if (msg_cls == NAV_CLASS) {
    return "Navigation";
  } else if (msg_cls == RXM_CLASS) {
    return "Receiver Manager";
  } else if (msg_cls == INF_CLASS) {
    return "Information";
  } else if (msg_cls == ACK_CLASS) {
    return "ACK/NAK";
  } else if (msg_cls == CFG_CLASS) {
    return "Configuration";
  } else if (msg_cls == UPD_CLASS) {
    return "Firmware update";
  } else if (msg_cls == MON_CLASS) {
    return "Monitoring";
  } else if (msg_cls == AID_CLASS) {
    return "AssistNow messages";
  } else if (msg_cls == TIM_CLASS) {
    return "Timing";
  } else if (msg_cls == ESF_CLASS) {
    return "External Sensor Fusion Messages";
  } else if (msg_cls == MGA_CLASS) {
    return "Multiple GNSS Assistance Messages";
  } else if (msg_cls == LOG_CLASS) {
    return "Logging";
  } else if (msg_cls == SEC_CLASS) {
    return "Security";
  } else if (msg_cls == HNR_CLASS) {
    return "High rate navigation results";
  } else {
    return std::to_string(msg_cls);
  }
}

/**
 * @brief   This function simply converts the msg_id (message identifier) to a
 string. It returns the string representation of the msg_id as-is.
 */
std::string msg_id_to_string(uint8_t msg_id) { return std::to_string(msg_id); }

/**
 * @brief   This function converts a 2-byte sequence represented in
 little-endian byte order (least significant byte first) into an integer. It
 performs bitwise operations to combine the bytes into a single integer value.
 */
uint16_t u2_to_int(const std::vector<uint8_t> &little_endian_bytes) {
  if (little_endian_bytes.size() < 2) {
    // Handle error or return a default value
    return 0;
  }
  return (static_cast<uint16_t>(little_endian_bytes[1]) << 8) |
         little_endian_bytes[0];
}

/**
 * @brief   Similar to the previous function, this one converts a 4-byte
 sequence in little-endian order into an integer by performing bitwise
 operations to combine the bytes into a single 32-bit integer.
 */
uint32_t u4_to_int(const std::vector<uint8_t> &little_endian_bytes) {
  if (little_endian_bytes.size() < 4) {
    // Handle error or return a default value
    return 0;
  }
  return (static_cast<uint32_t>(little_endian_bytes[3]) << 24) |
         (static_cast<uint32_t>(little_endian_bytes[2]) << 16) |
         (static_cast<uint32_t>(little_endian_bytes[1]) << 8) |
         little_endian_bytes[0];
}

/**
 * @brief   This function converts a 2-byte sequence in little-endian order into
 * a signed integer.
 */
int16_t i2_to_int(const std::vector<uint8_t> &little_endian_bytes) {
  if (little_endian_bytes.size() < 2) {
    // Handle error or return a default value
    return 0;
  }
  return static_cast<int16_t>(
      (static_cast<uint16_t>(little_endian_bytes[1]) << 8) |
      little_endian_bytes[0]);
}

/**
 * @brief   Similar to the previous function, this one converts a 4-byte
 sequence in little-endian order into a signed 32-bit integer
 */
int32_t i4_to_int(const std::vector<uint8_t> &little_endian_bytes) {
  if (little_endian_bytes.size() < 4) {
    // Handle error or return a default value
    return 0;
  }
  return static_cast<int32_t>(
      (static_cast<uint32_t>(little_endian_bytes[3]) << 24) |
      (static_cast<uint32_t>(little_endian_bytes[2]) << 16) |
      (static_cast<uint32_t>(little_endian_bytes[1]) << 8) |
      little_endian_bytes[0]);
}

/**
 * @brief   This function converts an integer into a 2-byte sequence represented
            in little-endian byte order.
 */
std::vector<uint8_t> int_to_u2(uint16_t integer) {
  return {static_cast<uint8_t>(integer), static_cast<uint8_t>(integer >> 8)};
}

/**
 * @brief   Similar to the previous function, this one converts an integer into
 a 4-byte sequence in little-endian byte order
 */
std::vector<uint8_t> int_to_u4(uint32_t integer) {
  return {static_cast<uint8_t>(integer), static_cast<uint8_t>(integer >> 8),
          static_cast<uint8_t>(integer >> 16),
          static_cast<uint8_t>(integer >> 24)};
}

/**
 * @brief   This function converts an integer into a 2-byte sequence in
 little-endian order, treating it as a signed integer.
 */
std::vector<uint8_t> int_to_i2(int16_t integer) {
  return int_to_u2(static_cast<uint16_t>(integer));
}

/**
 * @brief   Similar to the previous function, this one converts an integer into
 a 4-byte sequence in little-endian order, treating it as a signed 32-bit
 integer.
 */
std::vector<uint8_t> int_to_i4(int32_t integer) {
  return int_to_u4(static_cast<uint32_t>(integer));
}

/**
 * @brief   This function looks up a GNSS fix type and if a match is found, it
 returns the corresponding string representation of the fix type. If no match is
 found, it returns the string "reserved / no fix."
 */
std::string get_gnss_fix_type(uint8_t fix_flag) {
  auto it = GNSS_FIX_TYPE.find(fix_flag);
  if (it != GNSS_FIX_TYPE.end()) {
    return it->second;
  }
  return "reserved / no fix";
}

