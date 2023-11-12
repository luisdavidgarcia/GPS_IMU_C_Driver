#ifndef UBX_MSG_H
#define UBX_MSG_H

#include <stdint.h>
#include <string>

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


#define NAV_CLASS 0x01
#define RXM_CLASS 0x02
#define INF_CLASS 0x04
#define ACK_CLASS 0x05
#define CFG_CLASS 0x06
#define UPD_CLASS 0x09
#define MON_CLASS 0x0A
#define AID_CLASS 0x0B
#define TIM_CLASS 0x0D
#define ESF_CLASS 0x10
#define MGA_CLASS 0x13
#define LOG_CLASS 0x21
#define SEC_CLASS 0x27
#define HNR_CLASS 0x28

#define SYNC_CHAR_1 0xB5
#define SYNC_CHAR_2 0x62

//********* NAV MESSAGE SECTION **********
#define NAV_PVT 0x07

//********* ACK MESSAGE SECTION **********
#define ACK_ACK 0x01
#define ACK_NAK 0x00

//********* CFG MESSAGE SECTION **********
#define CFG_PRT 0x00
#define CFG_MSG 0x01
#define CFG_RATE 0x08

//********* FixTypes SECTION **********
#define NO_FIX 0
#define DEAD_RECKONING_ONLY 1
#define TWO_D_FIX 2
#define THREE_D_FIX 3
#define GNSS_DEAD_RECKONING_COMBINED 4
#define TIME_ONLY_FIX 5

//******* DEBUG/HELPING CONSTANTS ********
#define MAX_MESSAGE_LENGTH 1008 
#define MAX_PAYLOAD_LENGTH 1000 

typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t payloadLength;
    uint8_t payload[MAX_MESSAGE_LENGTH];
    uint8_t checksumA;
    uint8_t checksumB;
} UbxMessage;

UbxMessage ComposeMessage(uint8_t msg_class, uint8_t msg_id, uint16_t payloadLength, const uint8_t* payload);
void ComputeChecksum(UbxMessage &msg);
void ResetPayload(UbxMessage &msg);
std::string MsgClassToString(uint8_t msgClass);
std::string GetGNSSFixType(uint8_t fixFlag);
std::string UbxMessageToString(UbxMessage &msg);

#endif // UBX_MSG_H

