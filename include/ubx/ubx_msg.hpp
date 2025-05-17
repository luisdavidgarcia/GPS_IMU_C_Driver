/*
UBX MESSAGE STRUCTURE:
    each part of the message consist of 1 byte, the length of the payload
    (only the payload) is an unsigned 16 bit integer (little endian)
    sync char 1 | sync char 2 | class | id | length of payload (2 byte little endian) | payload | checksum A | checksum B

NUMBER FORMATS:
    All multi-byte values are ordered in Little Endian format, unless
    otherwise indicated. All floating point values are transmitted in
    IEEE754 single or double precision.

UBX CHECKSUM:
    The checksum is calculated over the Message, starting and including
    the CLASS field, up until, but excluding, the Checksum Field.
    The checksum algorithm used is the 8-Bit Fletcher Algorithm:

NAV 0x01 Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
RXM 0x02 Receiver Manager Messages: Satellite Status, RTC Status
INF 0x04 Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
ACK 0x05 Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
UPD 0x09 Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
MON 0x0A Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
AID 0x0B AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
TIM 0x0D Timing Messages: Time Pulse Output, Time Mark Results
ESF 0x10 External Sensor Fusion Messages: External Sensor Measurements and Status Information
MGA 0x13 Multiple GNSS Assistance Messages: Assistance data for various GNSS
LOG 0x21 Logging Messages: Log creation, deletion, info and retrieval
SEC 0x27 Security Feature Messages
HNR 0x28 High Rate Navigation Results Messages: High rate time, position, speed, heading

CREDIT/CODE MODIFIED FROM: https://github.com/melopero/Melopero_UBX/tree/master
*/

#ifndef UBX_MSG_H
#define UBX_MSG_H

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <algorithm>

//********* MESSAGE CLASS SECTION **********
constexpr int NAV_CLASS = 0x01;
constexpr int RXM_CLASS = 0x02;
constexpr int INF_CLASS = 0x04;
constexpr int ACK_CLASS = 0x05;
constexpr int CFG_CLASS = 0x06;
constexpr int UPD_CLASS = 0x09;
constexpr int MON_CLASS = 0x0A;
constexpr int AID_CLASS = 0x0B;
constexpr int TIM_CLASS = 0x0D;
constexpr int ESF_CLASS = 0x10;
constexpr int MGA_CLASS = 0x13;
constexpr int LOG_CLASS = 0x21;
constexpr int SEC_CLASS = 0x27;
constexpr int HNR_CLASS = 0x28;

//********* SYNC CHAR SECTION **********
constexpr int SYNC_CHAR_1 = 0xB5;
constexpr int SYNC_CHAR_2 = 0x62;

//********* NAV MESSAGE SECTION **********
constexpr int NAV_PVT = 0x07;

//********* ACK MESSAGE SECTION **********
constexpr int ACK_ACK = 0x01;
constexpr int ACK_NAK = 0x00;

//********* CFG MESSAGE SECTION **********
constexpr int CFG_PRT = 0x00;
constexpr int CFG_MSG = 0x01;
constexpr int CFG_RATE = 0x08;

//********* FixTypes SECTION **********
constexpr int NO_FIX = 0;
constexpr int DEAD_RECKONING_ONLY = 1;
constexpr int TWO_D_FIX = 2;
constexpr int THREE_D_FIX = 3;
constexpr int GNSS_DEAD_RECKONING_COMBINED = 4;
constexpr int TIME_ONLY_FIX = 5;

//******* DEBUG/HELPING CONSTANTS ********
constexpr int MAX_MESSAGE_LENGTH = 1000;
constexpr int MAX_PAYLOAD_LENGTH = 92;

struct alignas(128) UbxMessage {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t msgClass;
  uint8_t msgId;
  uint16_t payloadLength;
  std::array<uint8_t, MAX_PAYLOAD_LENGTH> payload;
  uint8_t checksumA;
  uint8_t checksumB;
};

struct alignas(2) MessageInfo {
  uint8_t msgClass;
  uint8_t msgId;
};

class UBX
{
private:
  UbxMessage message;

public:
  UBX();
  ~UBX();
  void ComposeMessage(
    const MessageInfo& messageInfo,
    const std::vector<uint8_t>& payload
  );
  const UbxMessage& GetUBXMessage() const {return message;} 
  void ComputeChecksum();
  void ResetPayload();
  std::string MsgClassToString(const uint8_t& msgClass);
  std::string GetGNSSFixType(const uint8_t& fixFlag);
  std::string UbxMessageToString() const;
};

#endif // UBX_MSG_H

