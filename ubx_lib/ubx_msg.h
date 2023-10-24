fndef UBX_MSG_H
#define UBX_MSG_H

#include <cstdint>
#include <map>
#include <vector>
#include <string>

const uint8_t NAV_CLASS;
const uint8_t RXM_CLASS;
const uint8_t INF_CLASS;
const uint8_t ACK_CLASS;
const uint8_t CFG_CLASS;
const uint8_t UPD_CLASS;
const uint8_t MON_CLASS;
const uint8_t AID_CLASS;
const uint8_t TIM_CLASS;
const uint8_t ESF_CLASS;
const uint8_t MGA_CLASS;
const uint8_t LOG_CLASS;
const uint8_t SEC_CLASS;
const uint8_t HNR_CLASS;

const uint8_t SYNC_CHAR_1;
const uint8_t SYNC_CHAR_2;

const int MAX_MESSAGE_LENGTH;

std::vector<uint8_t> payloadFromMessage(const std::vector<uint8_t> &msg);
std::vector<uint8_t> composeMessage(uint8_t msg_class, uint8_t msg_id, uint16_t length = 0, const std::vector<uint8_t> &payload = {});
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

