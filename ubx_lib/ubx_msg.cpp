#include "UBX_MSG.h" 

/**
 * @brief   Convert a message class (msgClass) to its corresponding string representation.
 * @param   msgClass    The message class to convert.
 * @return  The string representation of the message class.
 */
std::string MsgClassToString(uint8_t msgClass) {
    if (msgClass == NAV_CLASS)
        return "Navigation";
    else if (msgClass == RXM_CLASS)
        return "Receiver Manager";
    else if (msgClass == INF_CLASS)
        return "Information";
    else if (msgClass == ACK_CLASS)
        return "ACK/NAK";
    else if (msgClass == CFG_CLASS)
        return "Configuration";
    else if (msgClass == UPD_CLASS)
        return "Firmware update";
    else if (msgClass == MON_CLASS)
        return "Monitoring";
    else if (msgClass == AID_CLASS)
        return "AssistNow messages";
    else if (msgClass == TIM_CLASS)
        return "Timing";
    else if (msgClass == ESF_CLASS)
        return "External Sensor Fusion Messages";
    else if (msgClass == MGA_CLASS)
        return "Multiple GNSS Assistance Messages";
    else if (msgClass == LOG_CLASS)
        return "Logging";
    else if (msgClass == SEC_CLASS)
        return "Security";
    else if (msgClass == HNR_CLASS)
        return "High rate navigation results";

    return "Couldn't find class";
}

/**
 * @brief   Convert a GNSS fix flag (fixFlag) to its corresponding string representation.
 * @param   fixFlag The GNSS fix flag to convert.
 * @return  The string representation of the GNSS fix type.
 */
std::string GetGNSSFixType(uint8_t fixFlag) {
    if (fixFlag == NO_FIX)
        return "no fix";
    else if (fixFlag == DEAD_RECKONING_ONLY)
        return "dead reckoning only";
    else if (fixFlag == TWO_D_FIX)
        return "2D-fix";
    else if (fixFlag == THREE_D_FIX)
        return "3D-fix";
    else if (fixFlag == GNSS_DEAD_RECKONING_COMBINED)
        return "GNSS + dead reckoning combined";
    else if (fixFlag == TIME_ONLY_FIX)
        return "time only fix";
    else
        return "reserved / no fix";
}

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message.
 * @param   msg The UbxMessage struct for which to calculate the checksum.
 */
void ComputeChecksum(UbxMessage &msg) {
    msg.checksumA = msg.msgClass;
    msg.checksumB = msg.checksumA;

    msg.checksumA += msg.msgId;
    msg.checksumB += msg.checksumA;

    msg.checksumA += msg.length % (1 << 8);
    msg.checksumB += msg.checksumA;

    msg.checksumA += msg.length >> 8;
    msg.checksumB += msg.checksumA;

    for (int i = 0; i < msg.length; i++) {
        msg.checksumA += msg.payload[i];
        msg.checksumB += msg.checksumA;
    }
}

/**
 * @brief   Reset the payload of a UBX message to all zeros.
 * @param   msg The UbxMessage struct for which to reset the payload.
 */
void ResetPayload(UbxMessage &msg) {
    for (int i = 0; i < msg.length; i++)
        msg.payload[i] = 0;
}

/**
 * @brief   Convert a UBX message to its string representation.
 * @param   msg The UbxMessage struct to convert.
 * @return  The string representation of the UBX message.
 */
std::string UbxMessageToString(UbxMessage &msg) {
    std::string result = "Class : " + std::to_string(msg.msgClass) + "\n";
    result += "ID : " + std::to_string(msg.msgId) + "\n";
    result += "Length : " + std::to_string(msg.length) + "\n";
    result += "Payload : ";
    for (int i = 0; i < msg.length; i++) {
        result += std::to_string(msg.payload[i]) + " ";
    }
    result += "\n";
    result += "checksum : " + std::to_string(msg.checksumA) + " " + std::to_string(msg.checksumB) + "\n";
    return result;
}

