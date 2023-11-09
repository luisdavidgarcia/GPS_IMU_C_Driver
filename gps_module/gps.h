/*
 * CREDIT/CODE MODIFIED FROM:https://github.com/melopero/Melopero_SAM-M8Q_Arduino_Library/tree/master
 */
#ifndef GPS_H_INCLUDED
#define GPS_H_INCLUDED

#include "../ubx_lib/ubx_msg.h"
#include <fcntl.h>
#include <unistd.h>
extern "C" {
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h> 
}
#include <cstdint>
#include <sys/ioctl.h>
#include <string>

#define GPS_I2C_ADDRESS 0x42
#define GPS_I2C_BUS "/dev/i2c-1"

#define DATA_STREAM_REGISTER 0xFF

#define AVAILABLE_BYTES_MSB 0xFD
#define AVAILABLE_BYTES_LSB 0xFE

#define DEFAULT_TIMEOUT_MILLS 2000
#define DEFAULT_UPDATE_MILLS 1000
#define DEFAULT_SEND_RATE 0x01
#define DEFAULT_INTERVAL_MILLS 50
#define DEFAULT_POLLING_STATE false


enum class Status : int8_t {
    NoError = 0,
    ArgumentError = -2,
    ErrorSending = -3,
    ErrorReceiving = -4,
    OperationTimeout = -5
};

enum class TimeRef : uint8_t {
    UTC = 0,
    GPS = 1,
    GLONASS = 2,
    BeiDou = 3,
    Galileo = 4
};

class Gps {

  private:
      UbxMessage ubxmsg;
      PVTData pvtData;
      int i2c_fd;
      uint32_t ExtractU4FromUbxMessage(UbxMessage& msg, uint16_t startIndex);
      uint16_t ExtractU2FromUbxMessage(UbxMessage& msg, uint16_t startIndex);
      Status WriteUbxMessage(UbxMessage& msg);
      Status ReadUbxMessage(UbxMessage& msg);

  public:
      Gps();
      ~Gps();
      void UbxOnly(void);
      uint16_t GetAvailableBytes();
      Status PollUbxMessage(UbxMessage& msg);
      Status WaitForUbxMessage(UbxMessage& msg, 
          uint32_t timeoutMillis = DEFAULT_TIMEOUT_MILLS, 
          uint32_t intervalMillis = DEFAULT_INTERVAL_MILLS);
      bool WaitForAcknowledge(uint8_t msgClass, uint8_t msgId);
      Status SetCommunicationToUbxOnly();
      Status SetMessageSendRate(uint8_t msgClass, uint8_t msgId, 
          uint8_t sendRate = DEFAULT_SEND_RATE);
      Status SetMeasurementFrequency(uint16_t measurementPeriodMillis = 
          DEFAULT_UPDATE_MILLS, uint8_t navigationRate = 1, 
          TimeRef timeref = TimeRef::UTC);
      Status UpdatePVT(bool polling = DEFAULT_POLLING_STATE, 
          uint16_t timeOutMillis = DEFAULT_UPDATE_MILLS);
      std::string GetStatusDescription(Status status);
};

#endif // GPS_H

