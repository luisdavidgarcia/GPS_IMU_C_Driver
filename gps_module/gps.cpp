#include "gps.h"
#include <unistd.h>
#include <stdio.h>
#include <vector>

Gps::Gps() {
  const char *deviceName = GPS_I2C_BUS;
  i2c_fd = open(deviceName, O_RDWR);
  if (i2c_fd < 0) {
    perror("Unable to open I2C GPS device");
  }

  if (ioctl(i2c_fd, I2C_SLAVE, GPS_I2C_ADDRESS) < 0) {
    perror("Failed to acquire I2C GPS address");
  }

  this->ubxOnly();

  bool result = this->setMessageSendRate(NAV_CLASS, NAV_PVT, 1);
  if (!result) {
    printf("Error: Failed to set message send rate for NAV_PVT.\n");
    exit(-1); 
  }

  // result = this->setMeasurementFrequency(500, 1, 0);
  // if (!result) {
  //     printf("Error: Failed to set measurement frequency.\n");
  //     exit(-1); 
  // }
}

Gps::~Gps() { close(i2c_fd); }

void Gps::ubxOnly(void) {
    uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_PRT, 20, payload);

    bool result = this->writeUbxMessage(message);
    if (!result) {
      exit(-1);
    }
}

bool Gps::setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate = DEFAULT_SEND_RATE) {
    uint8_t payload[] = {msgClass, msgId, sendRate, 0x00, 0x00, 0x00, 0x00, 0x00};

    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_MSG, 8, payload);

    bool result = this->writeUbxMessage(message);

    return result;
}

bool Gps::setMeasurementFrequency(uint16_t measurementPeriodMillis = DEFAULT_UPDATE_MILLS, uint8_t navigationRate = 1, uint8_t timeref = 0) {
    uint8_t payload[6];

    // Convert measurement_period_ms to little-endian bytes
    payload[0] = static_cast<uint8_t>(measurementPeriodMillis & 0xFF);
    payload[1] = static_cast<uint8_t>((measurementPeriodMillis >> 8) & 0xFF);
    // Convert navigation_rate to little-endian bytes will always be 1
    payload[2] = navigationRate;
    payload[3] = 0x00;
    // Convert timeref to little-endian bytes will always be 0 for now
    payload[4] = timeref;
    payload[5] = 0x00;

  for (int i = 0; i < 6; i++) { 
    printf("payload[%d]: 0x%02X\n", i, payload[i]);
  }

    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_RATE, 6, payload);

    bool result = writeUbxMessage(message);

    return result;
}

uint16_t Gps::getAvailableBytes() {
  uint8_t msb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t lsb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_LSB);

  uint16_t availableBytes = (msb << 8) | lsb;
  // Combine MSB and LSB to form a 16-bit value
  return availableBytes;
}

bool Gps::writeUbxMessage(UbxMessage &msg) {
  std::vector<uint8_t> tempBuf; 
  tempBuf.push_back(msg.sync1);
  tempBuf.push_back(msg.sync2);
  tempBuf.push_back(msg.msgClass);
  tempBuf.push_back(msg.msgId);
  tempBuf.push_back(msg.payloadLength & 0xFF);
  tempBuf.push_back(msg.payloadLength >> 8);
  //tempBuf.push_back(msg.payloadLength & 0xFF);
  for (int i = 0; i < msg.payloadLength; i++) {
    tempBuf.push_back(msg.payload[i]);
  }
  tempBuf.push_back(msg.checksumA);
  tempBuf.push_back(msg.checksumB);

  uint8_t buf[tempBuf.size()];
  for (int i = 0; i < tempBuf.size(); i++) {
    buf[i] = tempBuf[i];
  }

  for (int i = 0; i < tempBuf.size(); i++) {
    int8_t reg = i2c_smbus_write_byte_data(i2c_fd, 0x02, buf[i]);
    if (reg < 0) {
      perror("Failed to write to I2C device");
      return false;
    }
  }

  // if (i2c_smbus_write_block_data(i2c_fd, 0xFF, tempBuf.size(), buf) < 0) {
  //   perror("Failed to write to I2C device");
  //   return false;
  // }

  return true;
}

UbxMessage Gps::readUbxMessage() {
  uint16_t messageLength = getAvailableBytes();
  printf("Message Length: %d\n", messageLength);
  std::vector<uint8_t> message;

  if (messageLength > 2 && messageLength < MAX_MESSAGE_LENGTH) {
        for (int i = 0; i < messageLength; i++) {
            uint8_t byte_data = i2c_smbus_read_byte_data(i2c_fd, DATA_STREAM_REGISTER);
            if (byte_data < -1) {
                perror("Failed to read byte from I2C device");
                UbxMessage badMsg;
                badMsg.sync1 = 255;
                return badMsg;  // Return an empty message on error
            }
            message.push_back(static_cast<uint8_t>(byte_data)); // Cast to uint8_t
        }

        if (message[0] == SYNC_CHAR_1 && message[1] == SYNC_CHAR_2) {
          printf("Message Size: %lu\n", message.size());
          for (size_t i = 0; i < message.size(); ++i) {
              printf("i: %d |", i);
              printf("0x%02X", message[i]); // %02X formats the number in hexadecimal, 2 digits with leading zeros
              if (i < message.size() - 1) {
                  printf(", "); // Separate elements with a comma
              }
          }
          printf("\n");

          UbxMessage ubxMsg;
          ubxMsg.sync1 = message[0];
          ubxMsg.sync2 = message[1];
          ubxMsg.msgClass = message[2];
          ubxMsg.msgId = message[3];
          ubxMsg.payloadLength = (message[5] << 8 | message[4]);
          memcpy(&ubxMsg.payload, &message[6], ubxMsg.payloadLength);
          ubxMsg.checksumA = message[message.size() - 2];
          ubxMsg.checksumB = message[message.size() - 1];

          return ubxMsg;
        }
    }

  UbxMessage badMsg;
  badMsg.sync1 = 255;

  return badMsg;  // Return an empty message
}

PVTData Gps::GetPvt(bool polling = DEFAULT_POLLING_STATE, 
    uint16_t timeOutMillis = DEFAULT_UPDATE_MILLS) {

  if (polling) {
    UbxMessage message = ComposeMessage(NAV_CLASS, NAV_PVT, 0, nullptr);
    this->writeUbxMessage(message);
  }
  // Works here
  UbxMessage message = this->readUbxMessage();
  
     if (message.sync1 != 255) {
        pvtData.year = u2_to_int(&message.payload[4]);
        pvtData.month = message.payload[6];
        pvtData.day = message.payload[7];
        pvtData.hour = message.payload[8];
        pvtData.min = message.payload[9];
        pvtData.sec = message.payload[10];

        uint8_t valid_flag = message.payload[11];

        // Extract and clarify flags
        pvtData.validDateFlag = (valid_flag & 0x01) == 0x01 ? 1 : 0;
        pvtData.validTimeFlag = (valid_flag & 0x02) == 0x02 ? 1 : 0;
        pvtData.fullyResolved = (valid_flag & 0x04) == 0x04 ? 1 : 0;
        pvtData.validMagFlag = (valid_flag & 0x08) == 0x08 ? 1 : 0;

        // Extract GNSS fix and related data
        pvtData.gnssFix = message.payload[20];
        memcpy(&pvtData.fixStatusFlags, &message.payload[21], 2);
        pvtData.numberOfSatellites = message.payload[23];

        // Extract longitude and latitude in degrees
        pvtData.longitude = i4_to_int(&message.payload[24]) * 1e-07;
        pvtData.latitude = i4_to_int(&message.payload[28]) * 1e-07;

        // Extract height data
        pvtData.height = i4_to_int(&message.payload[32]);
        pvtData.heightMSL = i4_to_int(&message.payload[36]);

        // Extract horizontal and vertical accuracy in millimeters
        pvtData.horizontalAccuracy = u4_to_int(&message.payload[40]);
        pvtData.verticalAccuracy = u4_to_int(&message.payload[44]);

        // Extract North East Down velocity in mm/s
        pvtData.velocityNorth = i4_to_int(&message.payload[48]);
        pvtData.velocityEast = i4_to_int(&message.payload[52]);
        pvtData.velocityDown = i4_to_int(&message.payload[56]);

        // Extract ground speed in mm/s and motion heading in degrees
        pvtData.groundSpeed = i4_to_int(&message.payload[60]);
        pvtData.motionHeading = i4_to_int(&message.payload[64]) * 1e-05;

        // Extract speed accuracy in mm/s and heading accuracy in degrees
        pvtData.speedAccuracy = u4_to_int(&message.payload[68]);
        pvtData.motionHeadingAccuracy = u4_to_int(&message.payload[72]) * 1e-05;

        // Extract vehicle heading in degrees
        pvtData.vehicalHeading = i4_to_int(&message.payload[84]) * 1e-05;

        // Extract magnetic declination and accuracy in degrees
        pvtData.magneticDeclination = i2_to_int(&message.payload[88]) * 1e-02;
        pvtData.magnetDeclinationAccuracy = u2_to_int(&message.payload[90]) * 1e-02;

      return this->pvtData;
    }

  pvtData.year = 33920;
  return this->pvtData;
}

// Function to extract an integer from a little-endian byte array
int16_t Gps::i2_to_int(const uint8_t *little_endian_bytes) {
    return (int16_t)(((uint16_t)little_endian_bytes[1] << 8) | (uint16_t)little_endian_bytes[0]);
}

// Function to extract an unsigned integer from a little-endian byte array
uint16_t Gps::u2_to_int(const uint8_t *little_endian_bytes) {
    return (uint16_t)(((uint16_t)little_endian_bytes[1] << 8) | (uint16_t)little_endian_bytes[0]);
}

// Function to extract a signed 32-bit integer from a little-endian byte array
int32_t Gps::i4_to_int(const uint8_t *little_endian_bytes) {
    return (int32_t)(((uint32_t)little_endian_bytes[3] << 24) | ((uint32_t)little_endian_bytes[2] << 16) | ((uint32_t)little_endian_bytes[1] << 8) | (uint32_t)little_endian_bytes[0]);
}

// Function to extract an unsigned 32-bit integer from a little-endian byte array
uint32_t Gps::u4_to_int(const uint8_t *little_endian_bytes) {
    return (uint32_t)(((uint32_t)little_endian_bytes[3] << 24) | ((uint32_t)little_endian_bytes[2] << 16) | ((uint32_t)little_endian_bytes[1] << 8) | (uint32_t)little_endian_bytes[0]);
}

