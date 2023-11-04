#ifndef UBX_MSG_H
#define UBX_MSG_H

#include <cstdint>
#include <map>
#include <vector>
#include <string>

#define  NAV_CLASS 0x01
#define  RXM_CLASS 0x02
#define  INF_CLASS 0x04
#define  ACK_CLASS 0x05
#define  CFG_CLASS 0x06
#define  UPD_CLASS 0x09
#define  MON_CLASS 0x0A
#define  AID_CLASS 0x0B
#define  TIM_CLASS 0x0D
#define  ESF_CLASS 0x10
#define  MGA_CLASS 0x13
#define  LOG_CLASS 0x21
#define  SEC_CLASS 0x27
#define  HNR_CLASS 0x28

#define  SYNC_CHAR_1 0xB5
#define  SYNC_CHAR_2 0x62

#define NAV_PVT 0x07

#define ACK_ACK 0x01
#define ACK_NAK 0x00

#define CFG_PRT 0x00
#define CFG_MSG 0x01
#define CFG_RATE 0x08

#define MAX_MESSAGE_LENGTH 1000

std::string ping(std::string);
std::vector<uint8_t> payloadFromMessage(const std::vector<uint8_t> &msg);
std::vector<uint8_t> composeMessage(uint8_t msg_class, uint8_t msg_id, uint16_t length, const std::vector<uint8_t> &payload);
std::pair<uint8_t, uint8_t> computeChecksum(const std::vector<uint8_t> &msg);
std::string msgClassToString(uint8_t msg_cls);
std::string msg_id_to_string(uint8_t msg_id);
uint16_t u2_to_int(const std::vector<uint8_t> &little_endian_bytes);
uint32_t u4_to_int(const std::vector<uint8_t> &little_endian_bytes);
int16_t i2_to_int(const std::vector<uint8_t> &little_endian_bytes);
int32_t i4_to_int(const std::vector<uint8_t> &little_endian_bytes);
std::vector<uint8_t> int_to_u2(uint16_t integer);
std::vector<uint8_t> int_to_u4(uint32_t integer);
std::vector<uint8_t> int_to_i2(int16_t integer);
std::vector<uint8_t> int_to_i4(int32_t integer);
std::string get_gnss_fix_type(uint8_t fix_flag);

#endif // UBX_MSG_H

