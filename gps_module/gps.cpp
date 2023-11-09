#include "gps.h"
#include <unistd.h>

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
}

Gps::~Gps() { close(i2c_fd); }

void UbxOnly() {
    // Define the payload data
    uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // Compose the UBX message
    UbxMessage message = ComposeMessage(CFG_CLASS, CFG_PRT, 20, payload);

    // Assuming you have a function named writeMessage to send the message
    WriteUbxMessage(message);
}

// TODO: Integrate SMBUS I2C for Get Available Bytes
/* returns the number of bytes available to read*/
uint16_t Gps::GetAvailableBytes() {
  i2c_smbus_write_byte(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t msb = i2c_smbus_read_byte(i2c_fd);
  uint8_t lsb = i2c_smbus_read_byte(i2c_fd);
  usleep(100000);


  //i2cBus->beginTransmission(this->i2cAddress);
  //i2cBus->write(AVAILABLE_BYTES_MSB);
  //if (i2cBus->endTransmission(false) != 0)
  //  return 0;

// i2cBus->requestFrom(this->i2cAddress, 2);
//  uint8_t msb = i2cBus->read();
//  uint8_t lsb = i2cBus->read();
  if (msb == 0xFF || lsb == 0xFF) {
    printf("No Bytes were available\n");
    return 0;
  }
  msb &= 0x7F; //check if this is correct
  return ((uint16_t) msb << 8 | lsb);
}

// TODO: Integrate SMBUS I2C for Read
/* Reads a UBX message and populates the given UbxMessage*/
Status Gps::ReadUbxMessage(UbxMessage &msg) {
  uint16_t bytes = this->GetAvailableBytes();

  /*
  //if (bytes > MAX_MESSAGE_LENGTH || bytes <= 0)
  //  return Status::ErrorReceiving;
  i2cBus->beginTransmission(this->i2cAddress);
  i2cBus->write(DATA_STREAM_REGISTER);
  if (i2cBus->endTransmission(false) != 0)
    return Status::ErrorReceiving;

  if (bytes > 32)
    i2cBus->requestFrom(this->i2cAddress, 32, 0);
  else
    i2cBus->requestFrom(this->i2cAddress, (uint8_t) bytes);
  //Arduino's i2c buffer has 32 byte limit. We have to read 32 bytes at a time
  uint8_t bufferSize = 0;

  if (i2cBus->available()){

    uint8_t syncChA = i2cBus->read(); // sync char a
    uint8_t syncChB = i2cBus->read(); // sync char b

    if (!(syncChA == SYNC_CHAR_1 && syncChB == SYNC_CHAR_2))
      return Status::ErrorReceiving;

    msg.msgClass = i2cBus->read();
    msg.msgId = i2cBus->read();
    uint8_t lsb_length = i2cBus->read();
    uint8_t msb_length = i2cBus->read();
    msg.length = msb_length << 8 | lsb_length;

    bufferSize += 6;
    for (uint16_t i = 0; i < msg.length; i++){
      msg.payload[i] = i2cBus->read();
      bufferSize ++;
      if (bufferSize >= 32){
        bytes -= bufferSize;
        bufferSize = 0;

        if (bytes > 32)
          i2cBus->requestFrom(this->i2cAddress, 32, 0);
        else
          i2cBus->requestFrom(this->i2cAddress, (uint8_t) bytes);
      }
    }

    msg.checksumA = i2cBus->read();
    msg.checksumB = i2cBus->read();

    return Status::NoError;
  }
  else {
    return Status::ErrorReceiving;
  }
  */
  // TEMP
  return Status::ErrorReceiving;
}

// TODO: Integrate SMBUS I2C for Write
Status Gps::WriteUbxMessage(UbxMessage &msg) {
  if (i2c_smbus_write_block_data(i2c_fd, 0x00, msg.length, msg.payload) < 0) {
    return Status::ErrorReceiving;
  }

  return Status::NoError;
}

// TODO: Get Rid of Old delay and Millis() Arduino calls
/* waits for a given message type (class and id).
  timeoutSeconds: the maximum amount of time to wait for the message to arrive in milliseconds.
  intervalSeconds: the interval in milliseconds between two readings.
*/
Status Gps::WaitForUbxMessage(UbxMessage &msg, uint32_t timeoutMillis, uint32_t intervalMillis) {
  //int startTime = millis();
  int startTime = 0;
  uint8_t desiredClass = msg.msgClass;
  uint8_t desiredId = msg.msgId;

  int currTime = startTime;
  while (currTime - startTime < timeoutMillis){
      Status status = this->ReadUbxMessage(msg);
      if (status == Status::NoError) {
        if (msg.msgClass == desiredClass && msg.msgId == desiredId)
          return Status::NoError;
      }
      //delay(intervalMillis);
      //currTime = millis();
  }
  return Status::OperationTimeout;
}

/* An acknowledge message (or a Not Acknowledge message) is sent everytime
a configuration message is sent.*/
bool Gps::WaitForAcknowledge(uint8_t msgClass, uint8_t msgId) {
  this->ubxmsg.msgClass = ACK_CLASS;
  this->ubxmsg.msgId = ACK_ACK;
  Status status = this->WaitForUbxMessage(this->ubxmsg, 1000, 50);
  //see if a message is received
  if (status == Status::OperationTimeout)
    return false;

  //see if the received message is an acknowledge message
  if (this->ubxmsg.msgClass == ACK_CLASS && this->ubxmsg.msgId == ACK_ACK)
    if (this->ubxmsg.length >= 2)
      if (this->ubxmsg.payload[0] == msgClass && this->ubxmsg.payload[1] == msgId)
        return true;

  return false;
}

/*Sets the communication protocol to UBX (only) both for input and output*/
Status Gps::SetCommunicationToUbxOnly() {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_PRT;
  this->ubxmsg.length = 20;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[4] = 0x84;
  this->ubxmsg.payload[12] = 0x01;
  this->ubxmsg.payload[14] = 0x01;

  return this->WriteUbxMessage(this->ubxmsg);
}

/* Send rate is relative to the event a message is registered on.
For example, if the rate of a navigation message is set to 2,
the message is sent every second navigation solution */
Status Gps::SetMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate) {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_MSG;
  this->ubxmsg.length = 8;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[0] = msgClass;
  this->ubxmsg.payload[1] = msgId;
  this->ubxmsg.payload[2] = sendRate;
  return this->WriteUbxMessage(this->ubxmsg);
}

/*measurementPeriodMillis:
    elapsed time between GNSS measurements, which defines the rate,
    e.g. 100ms => 10Hz, Measurement rate should be greater than or
    equal to 25 ms.
navigationRate :
    The ratio between the number of measurements and the number of
    navigation solutions, e.g. 5 means five measurements for
    every navigation solution. Maximum value is 127. \n
timeref :
    The time system to which measurements are aligned:
    UTC | GPS | GLONASS | BeiDou | Galileo */
Status Gps::SetMeasurementFrequency(uint16_t measurementPeriodMillis, uint8_t navigationRate, TimeRef timeref) {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_RATE;
  this->ubxmsg.length = 6;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[0] = (uint8_t) (measurementPeriodMillis & 0xFF );
  this->ubxmsg.payload[1] = measurementPeriodMillis >> 8;
  this->ubxmsg.payload[2] = navigationRate;
  this->ubxmsg.payload[4] = (uint8_t) timeref;
  return this->WriteUbxMessage(this->ubxmsg);
}

/*Updates the pvt data contained in the struct pvtData.
polling : if true the pvt message is polled, else waits for the next navigation solution
timeOutMillis : the maximum time to wait for the message
To reduce the time between pvt messages the frequency of the message can be
increased with setMessageSendRate and setMeasurementFrequency. */
Status Gps::UpdatePVT(bool polling, uint16_t timeOutMillis) {
  this->ubxmsg.msgClass = NAV_CLASS;
  this->ubxmsg.msgId = NAV_PVT;
  if (polling){ //send message without payload
    this->ubxmsg.length = 0;
    Status status = this->WriteUbxMessage(this->ubxmsg);
    if (status != Status::NoError)
      return status;
  }
  //read response / wait for the next pvt message
  Status status = this->WaitForUbxMessage(this->ubxmsg, timeOutMillis); 
  //TODO: check status
  if (status == Status::NoError){
    this->pvtData.itow = this->ExtractU4FromUbxMessage(this->ubxmsg, 0);
    this->pvtData.year = this->ExtractU2FromUbxMessage(this->ubxmsg, 4);
    this->pvtData.month = this->ubxmsg.payload[6];
    this->pvtData.day = this->ubxmsg.payload[7];
    this->pvtData.hour = this->ubxmsg.payload[8];
    this->pvtData.min = this->ubxmsg.payload[9];
    this->pvtData.sec = this->ubxmsg.payload[10];
    this->pvtData.validTimeFlag = this->ubxmsg.payload[11];
    this->pvtData.timeAccuracy = this->ExtractU4FromUbxMessage(this->ubxmsg, 12); //nanoseconds
    this->pvtData.nano = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 16); //nanoseconds
    this->pvtData.fixType = this->ubxmsg.payload[20];
    this->pvtData.fixStatusFlags = this->ubxmsg.payload[21];
    this->pvtData.additionalFlags = this->ubxmsg.payload[22];
    this->pvtData.numberOfSatellites = this->ubxmsg.payload[23];
    this->pvtData.longitude = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 24); //degrees
    this->pvtData.latitude = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 28); //degrees
    this->pvtData.height = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 32); //mm
    this->pvtData.hMSL = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 36); //mm
    this->pvtData.horizontalAccuracy = this->ExtractU4FromUbxMessage(this->ubxmsg, 40);
    this->pvtData.verticalAccuracy = this->ExtractU4FromUbxMessage(this->ubxmsg, 44);
    this->pvtData.velocityNorth = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 48);//mm/s
    this->pvtData.velocityEast = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 52); //mm/s
    this->pvtData.velocityDown = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 56); //mm/s
    this->pvtData.groundSpeed = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 60); //mm/s
    this->pvtData.headingOfMotion = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 64); //degrees
    this->pvtData.speedAccuracy = this->ExtractU4FromUbxMessage(this->ubxmsg, 68); //mm/s
    this->pvtData.headingAccuracy = this->ExtractU4FromUbxMessage(this->ubxmsg, 72); //mm/s
    this->pvtData.positionDOP = this->ExtractU2FromUbxMessage(this->ubxmsg, 76);
    this->pvtData.reserved = this->ubxmsg.payload[78];
    this->pvtData.headingOfVehicle = (int32_t) this->ExtractU4FromUbxMessage(this->ubxmsg, 84);
    this->pvtData.magneticDeclination = (int16_t) this->ExtractU2FromUbxMessage(this->ubxmsg, 88);
    this->pvtData.declinationAccuracy = this->ExtractU2FromUbxMessage(this->ubxmsg, 90);
  }

  return status;
}


std::string Gps::GetStatusDescription(Status status){
  if (status == Status::NoError)
    return "No Errors";
  else if (status == Status::ArgumentError)
    return "Argument Error";
  else if (status == Status::ErrorSending)
    return "Error occurred while sending a ubx message to the device(writing data)";
  else if (status == Status::ErrorReceiving)
    return "Error occurred while receiving a ubx message from the device(reading data)";
  else if (status == Status::OperationTimeout)
    return "Operation time out : the operation is taking too long to complete";
  else
    return "Unknown error / Status code";

  return "Unknown error / Status code";
}

uint32_t Gps::ExtractU4FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 3 >= msg.length)
    return 0;

  uint32_t value = (uint32_t) this->ExtractU2FromUbxMessage(msg, startIndex);
  value |= ((uint32_t) this->ExtractU2FromUbxMessage(msg, startIndex + 2)) << 16;
  return value;
}

uint16_t Gps::ExtractU2FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 1 >= msg.length)
    return 0;

  uint16_t value = (uint16_t) msg.payload[startIndex];
  value |= ((uint16_t) msg.payload[startIndex + 1]) << 8;
  return value;
}

