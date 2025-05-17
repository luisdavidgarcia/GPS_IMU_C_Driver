#include "ubx/ubx_msg.hpp"

/**
 * @brief   Compose a UBX message with the given message class, message ID, optional payloadLength, and payload.
 * @param   msg_class   The message class of the UBX message.
 * @param   msg_id      The message ID of the UBX message.
 * @param   payloadLength      The payloadLength of the payload (default is 0).
 * @param   payload     The payload data (default is nullptr).
 * @return  The composed UBX message.
 */
void UBX::ComposeMessage(
  const MessageInfo& messageInfo,
  const std::vector<uint8_t>& payload) 
{
  ResetPayload();

  message.sync1 = SYNC_CHAR_1;
  message.sync2 = SYNC_CHAR_2;
  message.msgClass = messageInfo.msgClass;
  message.msgId = messageInfo.msgId;
  message.payloadLength = std::min(
    static_cast<uint16_t>(payload.size()), 
    static_cast<uint16_t>(MAX_PAYLOAD_LENGTH)
  );

  std::copy(
    payload.begin(), 
    payload.begin() + message.payloadLength, 
    std::begin(message.payload));

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
    return "Navigation";
  }
  if (msgClass == RXM_CLASS) {
    return "Receiver Manager";
  }
  if (msgClass == INF_CLASS) {
    return "Information";
  }
  if (msgClass == ACK_CLASS) {
    return "ACK/NAK";
  }
  if (msgClass == CFG_CLASS) {
    return "Configuration";
  }
  if (msgClass == UPD_CLASS) {
    return "Firmware update";
  }
  if (msgClass == MON_CLASS) {
    return "Monitoring";
  }
  if (msgClass == AID_CLASS) {
    return "AssistNow messages";
  } 
  if (msgClass == TIM_CLASS) { 
    return "Timing";
  }
  if (msgClass == ESF_CLASS) {
    return "External Sensor Fusion Messages";
  }
  if (msgClass == MGA_CLASS) {
    return "Multiple GNSS Assistance Messages";
  }
  if (msgClass == LOG_CLASS) {
    return "Logging";
  }
  if (msgClass == SEC_CLASS) {
    return "Security";
  }
  if (msgClass == HNR_CLASS) {
    return "High rate navigation results";
  }

  return "Couldn't find class";
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
 * @param   msg The UbxMessage struct for which to calculate the checksum.
 */
void UBX::ComputeChecksum() 
{
  message.checksumA = message.msgClass;
  message.checksumB = message.checksumA;

  message.checksumA += message.msgId;
  message.checksumB += message.checksumA;

  message.checksumA += message.payloadLength % (1 << 8);
  message.checksumB += message.checksumA;

  message.checksumA += message.payloadLength >> 8;
  message.checksumB += message.checksumA;

  for (int i = 0; i < message.payloadLength; i++) {
    message.checksumA += message.payload.at(i);
    message.checksumB += message.checksumA;
  }
}

/**
 * @brief   Reset the payload of a UBX message to all zeros.
 * @param   msg The UbxMessage struct for which to reset the payload.
 */
void UBX::ResetPayload() 
{
  std::fill(message.payload.begin(), message.payload.end(), 0);
}

/**
 * @brief   Convert a UBX message to its string representation.
 * @param   msg The UbxMessage struct to convert.
 * @return  The string representation of the UBX message.
 */
std::string UBX::UbxMessageToString() const {
  std::string result = "Class : " + std::to_string(message.msgClass) + "\n";
  result += "ID : " + std::to_string(message.msgId) + "\n";
  result += "Length : " + std::to_string(message.payloadLength) + "\n";
  result += "Payload : ";
  for (int i = 0; i < message.payloadLength; i++) {
    result += std::to_string(message.payload.at(i)) + " ";
  }
  result += "\n";
  result += "checksum : " + std::to_string(message.checksumA) + " " + std::to_string(message.checksumB) + "\n";
  return result;
}

