#include "UBX/UBX_MSG.hpp"

/**
 * @brief   Compose a UBX message.
 * @param   messageInfo   The message class and ID of the UBX message.
 * @param   payload       The payload data (default is nullptr).
 * @return  The composed UBX message.
 */
void UBX::ComposeMessage(
  const MessageInfo& messageInfo,
  const std::vector<uint8_t>& payload) 
{
  ResetPayload();

  message_.sync1 = SYNC_CHAR_1;
  message_.sync2 = SYNC_CHAR_2;
  message_.msgClass = messageInfo.msgClass;
  message_.msgId = messageInfo.msgId;
  message_.payloadLength = std::min(
    static_cast<uint16_t>(payload.size()), 
    static_cast<uint16_t>(MAX_PAYLOAD_LENGTH)
  );

  std::copy(
    payload.begin(), 
    payload.begin() + message_.payloadLength, 
    std::begin(message_.payload));

  ComputeChecksum();
}

/**
 * @brief   Convert a message class (msgClass) to its corresponding string representation.
 * @param   msgClass    The message class to convert.
 * @return  The string representation of the message class.
 */
std::string MsgClassToString(const uint8_t& msgClass) 
{
  if (msgClass == NAV_CLASS) {
    return "Navigation"s;
  }
  if (msgClass == RXM_CLASS) {
    return "Receiver Manager"s;
  }
  if (msgClass == INF_CLASS) {
    return "Information"s;
  }
  if (msgClass == ACK_CLASS) {
    return "ACK/NAK"s;
  }
  if (msgClass == CFG_CLASS) {
    return "Configuration"s;
  }
  if (msgClass == UPD_CLASS) {
    return "Firmware update"s;
  }
  if (msgClass == MON_CLASS) {
    return "Monitoring"s;
  }
  if (msgClass == AID_CLASS) {
    return "AssistNow messages"s;
  } 
  if (msgClass == TIM_CLASS) { 
    return "Timing"s;
  }
  if (msgClass == ESF_CLASS) {
    return "External Sensor Fusion Messages"s;
  }
  if (msgClass == MGA_CLASS) {
    return "Multiple GNSS Assistance Messages"s;
  }
  if (msgClass == LOG_CLASS) {
    return "Logging"s;
  }
  if (msgClass == SEC_CLASS) {
    return "Security"s;
  }
  if (msgClass == HNR_CLASS) {
    return "High rate navigation results"s;
  }

  return "Couldn't find class"s;
}

/**
 * @brief   Convert a GNSS fix flag (fixFlag) to its corresponding string representation.
 * @param   fixFlag The GNSS fix flag to convert.
 * @return  The string representation of the GNSS fix type.
 */
std::string GetGNSSFixType(const uint8_t& fixFlag) 
{
  if (fixFlag == NO_FIX) {
    return "no fix";
  }
  if (fixFlag == DEAD_RECKONING_ONLY) {
    return "dead reckoning only";
  }
  if (fixFlag == TWO_D_FIX) {
    return "2D-fix";
  }
  if (fixFlag == THREE_D_FIX) {
    return "3D-fix";
  }
  if (fixFlag == GNSS_DEAD_RECKONING_COMBINED) {
    return "GNSS + dead reckoning combined";
  }
  if (fixFlag == TIME_ONLY_FIX) {
    return "time only fix";
  }
  
  return "reserved / no fix";
}

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message.
 */
void UBX::ComputeChecksum() 
{
  message_.checksumA = message_.msgClass;
  message_.checksumB = message_.checksumA;

  message_.checksumA += message_.msgId;
  message_.checksumB += message_.checksumA;

  message_.checksumA += message_.payloadLength % (1 << 8);
  message_.checksumB += message_.checksumA;

  message_.checksumA += message_.payloadLength >> 8;
  message_.checksumB += message_.checksumA;

  for (int i = 0; i < message_.payloadLength; i++) {
    message_.checksumA += message_.payload.at(i);
    message_.checksumB += message_.checksumA;
  }
}

/**
 * @brief   Reset the payload of a UBX message to all zeros.
 */
void UBX::ResetPayload() 
{
  std::fill(message_.payload.begin(), message_.payload.end(), 0);
}

/**
 * @brief   Convert a UBX message to its string representation.
 * @return  The string representation of the UBX message.
 */
std::string UBX::UbxMessageToString() const {
  std::string result = "Class : " + std::to_string(message_.msgClass) + "\n";
  result += "ID : " + std::to_string(message_.msgId) + "\n";
  result += "Length : " + std::to_string(message_.payloadLength) + "\n";
  result += "Payload : ";
  for (int i = 0; i < message_.payloadLength; i++) {
    result += std::to_string(message_.payload.at(i)) + " ";
  }
  result += "\n";
  result += "checksum : " + std::to_string(message_.checksumA) + " " + std::to_string(message_.checksumB) + "\n";
  return result;
}

