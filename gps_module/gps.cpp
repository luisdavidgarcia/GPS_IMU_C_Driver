#include "gps.h"
#include <unistd.h>
#include <stdio.h>
#include <vector>

Gps::Gps() {
  // Initiliaze I2C
  const char *deviceName = GPS_I2C_BUS;
  i2c_fd = open(deviceName, O_RDWR);
  if (i2c_fd < 0) {
    perror("Unable to open I2C GPS device");
  }

  if (ioctl(i2c_fd, I2C_SLAVE, GPS_I2C_ADDRESS) < 0) {
    perror("Failed to acquire I2C GPS address");
  }

  this->ubxOnly();

  UbxMessage result_msg;

  while (1) {
    result_msg = this->readUbxMessage();
    if (result_msg.sync1 != -1) {
      break;
    }
  }

  /*
  bool result = this->waitForAcknowledge(CFG_CLASS, CFG_PRT);
  if (!result) {
      printf("Error: Acknowledgment not received for setting communication to UBX only.\n");
      exit(-1);
  }
  */

  /*
  result = this->setMessageSendRate(NAV_CLASS, NAV_PVT, 1);
  if (!result) {
    printf("Error: Failed to set message send rate for NAV_PVT.\n");
    exit(-1); 
  }

  result = this->waitForAcknowledge(CFG_CLASS, CFG_MSG);
  if (!result) {
    printf("Error: Acknowledgment not received for setting message frequency.\n");
    exit(-1);
  }

  result = this->setMeasurementFrequency(500, 1, 0);
  if (!result) {
      printf("Error: Failed to set measurement frequency.\n");
      exit(-1); 
  }

  result = this->waitForAcknowledge(CFG_CLASS, CFG_RATE);
  if (!result) {
      printf("Error: Acknowledgment not received for setting measurement frequency.\n");
      exit(-1); 
  }
  */
}

Gps::~Gps() { close(i2c_fd); }

void Gps::ubxOnly(void) {
    uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_PRT, sizeof(payload), payload);

    bool result = this->writeUbxMessage(message);
    if (!result) {
      printf("Error: Could not write to GPS.\n");
      exit(-1);
    }
}

/*
bool Gps::setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate = DEFAULT_SEND_RATE) {
  uint8_t payload[] = {msgClass, msgId, sendRate, 0x00, 0x00, 0x00, 0x00, 0x00};

  UbxMessage message = ComposeMessage(CFG_CLASS, CFG_PRT, sizeof(payload), payload);

    bool result = this->writeUbxMessage(message);

    return result;
}

bool Gps::setMeasurementFrequency(uint16_t measurementPeriodMillis = DEFAULT_UPDATE_MILLS, uint8_t navigationRate = 1, uint8_t timeref = 0) {
    uint8_t payload[6];

    // Convert measurement_period_ms to little-endian bytes
    payload[0] = static_cast<uint8_t>(measurementPeriodMillis & 0xFF);
    payload[1] = static_cast<uint8_t>((measurementPeriodMillis >> 8) & 0xFF);
    // Convert navigation_rate to little-endian bytes
    payload[2] = navigationRate;
    // Convert timeref to little-endian bytes
    payload[3] = timeref;
    payload[4] = 0x00;
    payload[5] = 0x00;

    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_RATE, sizeof(payload), payload);

    bool result = writeUbxMessage(message);

    return result;
}
*/

uint16_t Gps::getAvailableBytes() {
  i2c_smbus_write_byte(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t msb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t lsb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_LSB);

  if (msb == 0xFF || lsb == 0xFF) {
    printf("No Bytes were available\n");
    return 0;
  }

  // Combine MSB and LSB to form a 16-bit value
  return static_cast<uint16_t>(msb << 8) | lsb;
}

bool Gps::writeUbxMessage(UbxMessage &msg) {
  std::vector<uint8_t> tempBuf; 
  tempBuf.push_back(msg.sync1);
  tempBuf.push_back(msg.sync2);
  tempBuf.push_back(msg.msgClass);
  tempBuf.push_back(msg.msgId);
  tempBuf.push_back(msg.payloadLength >> 8);
  tempBuf.push_back(msg.payloadLength & 0xFF);
  for (int i = 0; i < msg.payloadLength; i++) {
    tempBuf.push_back(msg.payload[i]);
  }
  tempBuf.push_back(msg.checksumA);
  tempBuf.push_back(msg.checksumB);

  uint8_t buf[tempBuf.size()];
  for (int i = 0; i < tempBuf.size(); i++) {
    buf[i] = tempBuf[i];
  }

  if (i2c_smbus_write_block_data(i2c_fd, 0x00, tempBuf.size(), buf) < 0) {
    return false;
  }

  return true;
}

UbxMessage Gps::readUbxMessage() {
  uint16_t messageLength = getAvailableBytes();
  std::vector<uint8_t> message;

  if (messageLength > 0 && messageLength < MAX_MESSAGE_LENGTH) {
      for (int i = 0; i < messageLength; i++) {
          uint8_t byte_data = i2c_smbus_read_byte_data(i2c_fd, DATA_STREAM_REGISTER);
          if (byte_data == static_cast<uint8_t>(-1)) {
              perror("Failed to read byte from I2C device");
              return UbxMessage();  // Return an empty message on error
          }
          message.push_back(byte_data);
      }

      UbxMessage ubxMsg;
      ubxMsg.sync1 = message[0];
      ubxMsg.sync2 = message[1];
      ubxMsg.msgClass = message[2];
      ubxMsg.msgId = message[3];
      ubxMsg.payloadLength = (message[4] << 8 | message[5]);
      memcpy(&ubxMsg.payload, &message[6], ubxMsg.payloadLength);
      ubxMsg.checksumA = message[messageLength - 2];
      ubxMsg.checksumB = message[messageLength - 1];

      return ubxMsg;
  }

  printf("Failed to read message.\n");
  UbxMessage badMsg;
  badMsg.sync1 = -1;

  return badMsg;  // Return an empty message
}

/*
UbxMessage Gps::pollUbxMessage(uint8_t msg_class, uint8_t msg_id) {
    UbxMessage ubxMsg;
    ubxMsg.msgClass = msg_class;
    ubxMsg.msgId = msg_id;
    ubxMsg.payloadLength = 0;

    bool result = writeUbxMessage(ubxMsg);

    if (!result) {
        printf("Failed to write poll message to GPS.\n");
        return UbxMessage();
    }

    return this->waitForUbxMessage(1, 1, CFG_CLASS, CFG_PRT);
}
*/
/*
UbxMessage Gps::waitForUbxMessage(uint32_t timeoutMillis=1, uint32_t intervalMillis=1, uint8_t msg_cls=CFG_CLASS, uint8_t msg_id=CFG_PRT) {
    uint32_t startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    UbxMessage response, msg;

    while (true) {
        uint32_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        if (currentTime - startTime >= timeoutMillis) {
            printf("Timeout while waiting for UBX message.\n");
            return UbxMessage();  
        }

        response = readUbxMessage(msg);  

        // Check if the received message matches the expected class and ID
        if (response.sync1 == msg.msgClass && response.sync2 == msg.msgId) {
            return response;
        }

        // Sleep for the specified interval before checking again
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMillis));
    }
}

bool Gps::waitForAcknowledge(uint8_t msgClass, uint8_t msgId) {
    bool ack = false;
    UbxMessage msg = waitForUbxMessage(msgClass=ACK_ACK, msgId);  

    if (msg.payloadLength == 0) {
        return ack;
    }

    if (msg.payload[3] == ACK_ACK && msgClass == msg.payload[6] && msgId == msg.msgId) {
        ack = true;
    }

    return ack;
}
*/
/*
PVTData Gps::GetPvt(bool polling = DEFAULT_POLLING_STATE, 
    uint16_t timeOutMillis = DEFAULT_UPDATE_MILLS) {
  return this->pvtData;
}

uint32_t Gps::extractU4FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 3 >= msg.payloadLength)
    return 0;

  uint32_t value = (uint32_t) this->extractU2FromUbxMessage(msg, startIndex);
  value |= ((uint32_t) this->extractU2FromUbxMessage(msg, startIndex + 2)) << 16;
  return value;
}

uint16_t Gps::extractU2FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 1 >= msg.payloadLength)
    return 0;

  uint16_t value = (uint16_t) msg.payload[startIndex];
  value |= ((uint16_t) msg.payload[startIndex + 1]) << 8;
  return value;
}

*/
