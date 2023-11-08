#include "gps.h"

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

// TODO: Integrate SMBUS I2C for Get Available Bytes
/* returns the number of bytes available to read*/
uint16_t Gps::getAvailableBytes() {
  i2c_smbus_write_byte(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t msb = i2c_smbus_read_byte(i2c_fd);
  uint8_t lsb = i2c_smbus_read_byte(i2c_fd);


  //i2cBus->beginTransmission(this->i2cAddress);
  //i2cBus->write(AVAILABLE_BYTES_MSB);
  //if (i2cBus->endTransmission(false) != 0)
  //  return 0;

// i2cBus->requestFrom(this->i2cAddress, 2);
//  uint8_t msb = i2cBus->read();
//  uint8_t lsb = i2cBus->read();
  if (msb == 0xFF || lsb == 0xFF) {
    printf("No Bytes were available");
    return 0;
  }
  msb &= 0x7F; //check if this is correct
  return ((uint16_t) msb << 8 | lsb);
}

// TODO: Integrate SMBUS I2C for Read
/* Reads a UBX message and populates the given UbxMessage*/
Status Gps::readUbxMessage(UbxMessage &msg) {
  uint16_t bytes = this->getAvailableBytes();

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
Status Gps::writeUbxMessage(UbxMessage &msg) {
  /*
  ComputeChecksum(msg);
  i2cBus->beginTransmission(this->i2cAddress);
  if (i2cBus->endTransmission(false) != 0)
    return Status::ErrorSending;

  i2cBus->beginTransmission(this->i2cAddress);
  i2cBus->write(SYNC_CHAR_1);
  i2cBus->write(SYNC_CHAR_2);
  i2cBus->write(msg.msgClass);
  i2cBus->write(msg.msgId);
  i2cBus->write((uint8_t) (msg.length & 0xFF)); // length lsb
  i2cBus->write((uint8_t) (msg.length >> 8)); // length msb
  for (int i = 0; i < msg.length; i++)
    i2cBus->write(msg.payload[i]);

  i2cBus->write(msg.checksumA);
  i2cBus->write(msg.checksumB);

  i2cBus->endTransmission();

  return Status::NoError;
}

Status Gps::pollUbxMessage(UbxMessage &msg) {
  Status status = Status::NoError;
  msg.length = 0;
  status = this->writeUbxMessage(msg);
  if (status != Status::NoError) return status;
  status = this->waitForUbxMessage(msg);
  return status;
  */
  // TEMP
  return Status::ErrorReceiving;

}

// TODO: Get Rid of Old delay and Millis() Arduino calls
/* waits for a given message type (class and id).
  timeoutSeconds: the maximum amount of time to wait for the message to arrive in milliseconds.
  intervalSeconds: the interval in milliseconds between two readings.
*/
Status Gps::waitForUbxMessage(UbxMessage &msg, uint32_t timeoutMillis, uint32_t intervalMillis) {
  //int startTime = millis();
  int startTime = 0;
  uint8_t desiredClass = msg.msgClass;
  uint8_t desiredId = msg.msgId;

  int currTime = startTime;
  while (currTime - startTime < timeoutMillis){
      Status status = this->readUbxMessage(msg);
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
bool Gps::waitForAcknowledge(uint8_t msgClass, uint8_t msgId) {
  this->ubxmsg.msgClass = ACK_CLASS;
  this->ubxmsg.msgId = ACK_ACK;
  Status status = this->waitForUbxMessage(this->ubxmsg, 1000, 50);
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
Status Gps::setCommunicationToUbxOnly() {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_PRT;
  this->ubxmsg.length = 20;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[4] = 0x84;
  this->ubxmsg.payload[12] = 0x01;
  this->ubxmsg.payload[14] = 0x01;

  return this->writeUbxMessage(this->ubxmsg);
}

/* Send rate is relative to the event a message is registered on.
For example, if the rate of a navigation message is set to 2,
the message is sent every second navigation solution */
Status Gps::setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate) {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_MSG;
  this->ubxmsg.length = 8;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[0] = msgClass;
  this->ubxmsg.payload[1] = msgId;
  this->ubxmsg.payload[2] = sendRate;
  return this->writeUbxMessage(this->ubxmsg);
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
Status Gps::setMeasurementFrequency(uint16_t measurementPeriodMillis, uint8_t navigationRate, TimeRef timeref) {
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_RATE;
  this->ubxmsg.length = 6;
  ResetPayload(this->ubxmsg);
  this->ubxmsg.payload[0] = (uint8_t) (measurementPeriodMillis & 0xFF );
  this->ubxmsg.payload[1] = measurementPeriodMillis >> 8;
  this->ubxmsg.payload[2] = navigationRate;
  this->ubxmsg.payload[4] = (uint8_t) timeref;
  return this->writeUbxMessage(this->ubxmsg);
}

/*Updates the pvt data contained in the struct pvtData.
polling : if true the pvt message is polled, else waits for the next navigation solution
timeOutMillis : the maximum time to wait for the message
To reduce the time between pvt messages the frequency of the message can be
increased with setMessageSendRate and setMeasurementFrequency. */
Status Gps::updatePVT(bool polling, uint16_t timeOutMillis) {
  this->ubxmsg.msgClass = NAV_CLASS;
  this->ubxmsg.msgId = NAV_PVT;
  if (polling){ //send message without payload
    this->ubxmsg.length = 0;
    Status status = this->writeUbxMessage(this->ubxmsg);
    if (status != Status::NoError)
      return status;
  }
  //read response / wait for the next pvt message
  Status status = this->waitForUbxMessage(this->ubxmsg, timeOutMillis); 
  //TODO: check status
  if (status == Status::NoError){
    this->pvtData.itow = this->extractU4FromUbxMessage(this->ubxmsg, 0);
    this->pvtData.year = this->extractU2FromUbxMessage(this->ubxmsg, 4);
    this->pvtData.month = this->ubxmsg.payload[6];
    this->pvtData.day = this->ubxmsg.payload[7];
    this->pvtData.hour = this->ubxmsg.payload[8];
    this->pvtData.min = this->ubxmsg.payload[9];
    this->pvtData.sec = this->ubxmsg.payload[10];
    this->pvtData.validTimeFlag = this->ubxmsg.payload[11];
    this->pvtData.timeAccuracy = this->extractU4FromUbxMessage(this->ubxmsg, 12); //nanoseconds
    this->pvtData.nano = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 16); //nanoseconds
    this->pvtData.fixType = this->ubxmsg.payload[20];
    this->pvtData.fixStatusFlags = this->ubxmsg.payload[21];
    this->pvtData.additionalFlags = this->ubxmsg.payload[22];
    this->pvtData.numberOfSatellites = this->ubxmsg.payload[23];
    this->pvtData.longitude = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 24); //degrees
    this->pvtData.latitude = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 28); //degrees
    this->pvtData.height = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 32); //mm
    this->pvtData.hMSL = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 36); //mm
    this->pvtData.horizontalAccuracy = this->extractU4FromUbxMessage(this->ubxmsg, 40);
    this->pvtData.verticalAccuracy = this->extractU4FromUbxMessage(this->ubxmsg, 44);
    this->pvtData.velocityNorth = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 48);//mm/s
    this->pvtData.velocityEast = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 52); //mm/s
    this->pvtData.velocityDown = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 56); //mm/s
    this->pvtData.groundSpeed = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 60); //mm/s
    this->pvtData.headingOfMotion = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 64); //degrees
    this->pvtData.speedAccuracy = this->extractU4FromUbxMessage(this->ubxmsg, 68); //mm/s
    this->pvtData.headingAccuracy = this->extractU4FromUbxMessage(this->ubxmsg, 72); //mm/s
    this->pvtData.positionDOP = this->extractU2FromUbxMessage(this->ubxmsg, 76);
    this->pvtData.reserved = this->ubxmsg.payload[78];
    this->pvtData.headingOfVehicle = (int32_t) this->extractU4FromUbxMessage(this->ubxmsg, 84);
    this->pvtData.magneticDeclination = (int16_t) this->extractU2FromUbxMessage(this->ubxmsg, 88);
    this->pvtData.declinationAccuracy = this->extractU2FromUbxMessage(this->ubxmsg, 90);
  }

  return status;
}


std::string Gps::getStatusDescription(Status status){
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

uint32_t Gps::extractU4FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 3 >= msg.length)
    return 0;

  uint32_t value = (uint32_t) this->extractU2FromUbxMessage(msg, startIndex);
  value |= ((uint32_t) this->extractU2FromUbxMessage(msg, startIndex + 2)) << 16;
  return value;
}

uint16_t Gps::extractU2FromUbxMessage(UbxMessage &msg, uint16_t startIndex){
  if (startIndex + 1 >= msg.length)
    return 0;

  uint16_t value = (uint16_t) msg.payload[startIndex];
  value |= ((uint16_t) msg.payload[startIndex + 1]) << 8;
  return value;
}

//SAM_M8Q::SAM_M8Q(int32_t i2c_addr, int32_t i2c_bus) : curr_i2c_addr(i2c_addr), curr_i2c_bus(i2c_bus) {
//    // Allows the device to setup, avoiding i2c error 5
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//}
//
///**
// * @brief   Sets the communication protocol to UBX (only) both for input and output
// */
//void SAM_M8Q::ubx_only() {
//  std::vector<uint8_t> payload = {0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
//  std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_PRT, 20, payload);
//  write_message(message);
//}
//
///**
// * @brief   Send rate is relative to the event a message is registered on.
// *          For example, if the rate of a navigation message is set to 2,
// *          the message is sent every second navigation solution
// * @param   msg_class   UBX message class
// * @param   msg_id      UBX message ID
// * @param   freq        Frequency value
// */
//void SAM_M8Q::set_message_frequency(int32_t msg_class, int32_t msg_id, int32_t freq) {
//    std::vector<uint8_t> payload = {static_cast<uint8_t>(msg_class), static_cast<uint8_t>(msg_id), static_cast<uint8_t>(freq), 0x00, 0x00, 0x00, 0x00, 0x00};
//    std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_MSG, 8, payload);
//    write_message(message);
//}
//
///**
// * @brief   Sets the measurement frequency and aligns it to a specified time system.
// * @param   measurement_period_ms    Elapsed time between GNSS measurements
// * @param   navigation_rate         The ratio between the number of measurements and the number of navigation solutions
// * @param   timeref                 The time system to which measurements are aligned
// */
//void SAM_M8Q::set_measurement_frequency(int32_t measurement_period_ms, int32_t navigation_rate, int32_t timeref) {
//    std::vector<uint8_t> payload = UBX_MSG::int_to_u2(measurement_period_ms);
//    payload.insert(payload.end(), UBX_MSG::int_to_u2(navigation_rate).begin(), UBX_MSG::int_to_u2(navigation_rate).end());
//    payload.insert(payload.end(), UBX_MSG::int_to_u2(timeref).begin(), UBX_MSG::int_to_u2(timeref).end());
//
//    std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_RATE, 6, payload);
//    write_message(message);
//}
//
///**
// * @brief   Returns the number of bytes available if a timeout is specified it
// *          tries to read the number of bytes for the given amount of millis
// * @return  Number of available bytes
// */
//int32_t SAM_M8Q::available_bytes() {
//  uint8_t msb, lsb;
//  {
//	  SMBus bus(curr_i2c_bus);
//	  msb = bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER);
//	  lsb = bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER + 1);
//  }
//  return (msb << 8) | lsb;
//}
//
///**
// * @brief   Writes a message to the device
// * @param   buffer  Vector containing the message to be written
// */
//void SAM_M8Q::write_message(const std::vector<uint8_t>& buffer) {
//  SMBus bus(curr_i2c_bus);
//  i2c_msg msg_out(curr_i2c_addr, I2C_MSG_WRITE, buffer.size(), const_cast<uint8_t*>(buffer.data()));
//  bus.i2c_rdwr({msg_out});
//}
//
///**
// * @brief   Reads a message from the device
// * @return  Vector containing the read message
// */
//std::vector<uint8_t> SAM_M8Q::read_message() {
//  int32_t msg_length = available_bytes();
//  std::vector<uint8_t> msg;
//
//  if (msg_length > UBX_MSG::MAX_MESSAGE_LENGTH) {
//	  return msg;
//  }
//
//  {
//	  SMBus bus(curr_i2c_bus);
//	  for (int32_t i = 0; i < msg_length; ++i) {
//		  msg.push_back(bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER));
//	  }
//  }
//
//  return msg;
//}
//
///**
// * @brief   Sends a polling message to request a specific UBX message
// * @param   msg_class   UBX message class
// * @param   msg_id      UBX message ID
// * @return  Vector containing the received message
// */
//std::vector<uint8_t> SAM_M8Q::poll_message(int32_t msg_class, int32_t msg_id) {
//  std::vector<uint8_t> msg = UBX_MSG::compose_message(msg_class, msg_id, 0);
//  write_message(msg);
//  return wait_for_message(1, 0.01, msg_class, msg_id);
//}
//
///**
// * @brief   Waits for a specific UBX message to arrive
// * @param   time_out_s   Maximum amount of time to wait for the message to arrive in seconds
// * @param   interval_s   Interval in seconds between two readings
// * @param   msg_cls      The class of the message to wait for
// * @param   msg_id       The ID of the message to wait for
// * @return  Vector containing the received message, or an empty vector if timed out
// */
//std::vector<uint8_t> SAM_M8Q::wait_for_message(int32_t time_out_s, double interval_s, int32_t msg_cls, int32_t msg_id) {
//  auto start_time = std::chrono::high_resolution_clock::now();
//  std::vector<uint8_t> to_compare = {UBX_MSG::SYNC_CHAR_1, UBX_MSG::SYNC_CHAR_2, static_cast<uint8_t>(msg_cls), static_cast<uint8_t>(msg_id)};
//  std::vector<uint8_t> msg;
//
//  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() < time_out_s) {
//	  msg = read_message();
//	  if (msg.size() >= to_compare.size() && std::equal(to_compare.begin(), to_compare.end(), msg.begin())) {
//		  return msg;
//	  }
//	  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int32_t>(interval_s * 1000)));
//  }
//
//  return msg;
//}
//
///**
// * @brief   Waits for an acknowledge message to be received
// * @param   msg_class   UBX message class
// * @param   msg_id      UBX message ID
// * @param   verbose     Whether to print verbose information
// * @return  True if the message was acknowledged, false otherwise
// */
//bool SAM_M8Q::wait_for_acknowledge(int32_t msg_class, int32_t msg_id, bool verbose) {
//    bool ack = false;
//    std::vector<uint8_t> msg = wait_for_message(1, 0.01, UBX_MSG::ACK_CLASS);
//    if (!msg.empty()) {
//        if (msg[3] == UBX_MSG::ACK_ACK && msg[6] == static_cast<uint8_t>(msg_class) && msg[7] == static_cast<uint8_t>(msg_id)) {
//            ack = true;
//        }
//    }
//
//    if (verbose) {
//        std::cout << " A message of class : " << UBX_MSG::msg_class_to_string(msg[6]) << " and id : " << static_cast<int32_t>(msg[7])
//                  << " was " << (ack ? "" : "not ") << "acknowledged" << std::endl;
//    }
//
//    return ack;
//}
//
///**
// * @brief   Updates and returns the pvt_data dictionary that contains the last received pvt data
// * @param   polling         If true, the pvt message is polled, else waits for the next navigation solution
// * @param   time_out_s      The maximum time to wait for the message
// * @return  A map containing the PVT data
// */
//std::map<std::string, int32_t> SAM_M8Q::get_pvt(bool polling, int32_t time_out_s) {
//    if (polling) {
//        std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::NAV_CLASS, UBX_MSG::NAV_PVT);
//        write_message(message);
//    }
//
//    std::vector<uint8_t> read = wait_for_message(time_out_s, 0.01, UBX_MSG::NAV_CLASS, UBX_MSG::NAV_PVT);
//    std::map<std::string, int32_t> pvt_data;
//
//    if (!read.empty()) {
//        int32_t start_payload = 6;
//
//        // WARNING: POSITION_DOP AND ITOW ARE MISSING (NOT RETRIEVED)
//
//        // Time solution
//        int32_t year = UBX_MSG::u2_to_int({read[start_payload + 4], read[start_payload + 5]});
//        int32_t month = read[start_payload + 6];
//        int32_t day = read[start_payload + 7];
//        int32_t hour = read[start_payload + 8];
//        int32_t minutes = read[start_payload + 9];
//        int32_t sec = read[start_payload + 10];
//        int32_t valid_flag = read[start_payload + 11];
//
//        // clarifying flags
//        bool valid_date = valid_flag & 0x01;
//        bool valid_time = valid_flag & 0x02;
//        bool fully_resolved = valid_flag & 0x04;
//        bool valid_mag = valid_flag & 0x08;
//
//        // GNSS fix and flags
//        int32_t gnss_fix = UBX_MSG::get_gnss_fix_type(read[start_payload + 20]);
//        std::vector<uint8_t> fix_status_flags(read.begin() + start_payload + 21, read.begin() + start_payload + 23);
//        int32_t num_satellites = read[start_payload + 23];
//
//        // longitude and latitude are in Degrees
//        double longitude = UBX_MSG::i4_to_int({read[start_payload + 24], read[start_payload + 25], read[start_payload + 26], read[start_payload + 27]}) * 1e-07;
//        double latitude = UBX_MSG::i4_to_int({read[start_payload + 28], read[start_payload + 29], read[start_payload + 30], read[start_payload + 31]}) * 1e-07;
//
//        // height, mean sea level height in millimeters
//        int32_t height = UBX_MSG::i4_to_int({read[start_payload + 32], read[start_payload + 33], read[start_payload + 34], read[start_payload + 35]});
//        int32_t height_MSL = UBX_MSG::i4_to_int({read[start_payload + 36], read[start_payload + 37], read[start_payload + 38], read[start_payload + 39]});
//
//        // horizontal and vertical accuracy estimates in millimeters
//        int32_t h_acc = UBX_MSG::u4_to_int({read[start_payload + 40], read[start_payload + 41], read[start_payload + 42], read[start_payload + 43]});
//        int32_t v_acc = UBX_MSG::u4_to_int({read[start_payload + 44], read[start_payload + 45], read[start_payload + 46], read[start_payload + 47]});
//
//        // North East Down velocity in mm / s
//        int32_t n_vel = UBX_MSG::i4_to_int({read[start_payload + 48], read[start_payload + 49], read[start_payload + 50], read[start_payload + 51]});
//        int32_t e_vel = UBX_MSG::i4_to_int({read[start_payload + 52], read[start_payload + 53], read[start_payload + 54], read[start_payload + 55]});
//        int32_t d_vel = UBX_MSG::i4_to_int({read[start_payload + 56], read[start_payload + 57], read[start_payload + 58], read[start_payload + 59]});
//
//        // Ground speed in mm / s and heading of motion in degrees + speed and heading accuracy estimates
//        int32_t g_speed = UBX_MSG::i4_to_int({read[start_payload + 60], read[start_payload + 61], read[start_payload + 62], read[start_payload + 63]});
//        double motion_heading = UBX_MSG::i4_to_int({read[start_payload + 64], read[start_payload + 65], read[start_payload + 66], read[start_payload + 67]}) * 1e-05;
//        int32_t s_acc = UBX_MSG::u4_to_int({read[start_payload + 68], read[start_payload + 69], read[start_payload + 70], read[start_payload + 71]});
//        double m_acc = UBX_MSG::u4_to_int({read[start_payload + 72], read[start_payload + 73], read[start_payload + 74], read[start_payload + 75]}) * 1e-05;
//
//        // Heading of vehicle in degrees
//        double vehicle_heading = UBX_MSG::i4_to_int({read[start_payload + 84], read[start_payload + 85], read[start_payload + 86], read[start_payload + 87]}) * 1e-05;
//
//        // Magnetic declination and magnetic declination accuracy both in degrees
//        double mag_deg = UBX_MSG::i2_to_int({read[start_payload + 88], read[start_payload + 89]}) * 1e-02;
//        double mag_deg_acc = UBX_MSG::u2_to_int({read[start_payload + 90], read[start_payload + 91]}) * 1e-02;
//
//        // time
//        pvt_data[YEAR_TAG] = year;
//        pvt_data[MONTH_TAG] = month;
//        pvt_data[DAY_TAG] = day;
//        pvt_data[HOUR_TAG] = hour;
//        pvt_data[MINUTE_TAG] = minutes;
//        pvt_data[SECOND_TAG] = sec;
//
//        // flags
//        pvt_data[VALID_TIME_TAG] = valid_time;
//        pvt_data[VALID_DATE_TAG] = valid_date;
//        pvt_data[FULLY_RESOLVED_TAG] = fully_resolved;
//        pvt_data[VALID_MAG_DEC_TAG] = valid_mag;
//
//        // GNSS
//        pvt_data[GNSS_FIX_TAG] = gnss_fix;
//        pvt_data[FIX_STATUS_FLAGS_TAG] = fix_status_flags;
//        pvt_data[NUM_SATELLITES_TAG] = num_satellites;
//
//        // Coordinates
//        pvt_data[LONGITUDE_TAG] = longitude;
//        pvt_data[LATITUDE_TAG] = latitude;
//        pvt_data[ELLIPSOID_HEIGHT_TAG] = height;
//        pvt_data[MSL_HEIGHT_TAG] = height_MSL;
//        pvt_data[HORIZONTAL_ACCURACY_TAG] = h_acc;
//        pvt_data[VERTICAL_ACCURACY_TAG] = v_acc;
//
//        // Velocity and heading
//        pvt_data[NED_VELOCITY_TAG] = std::make_tuple(n_vel, e_vel, d_vel);
//        pvt_data[GROUND_SPEED_TAG] = g_speed;
//        pvt_data[VEHICLE_HEADING_TAG] = vehicle_heading;
//        pvt_data[MOTION_HEADING_TAG] = motion_heading;
//        pvt_data[SPEED_ACCURACY_TAG] = s_acc;
//        pvt_data[HEADING_ACCURACY_TAG] = m_acc;
//
//        // Magnetic declination
//        pvt_data[MAGNETIC_DECLINATION_TAG] = mag_deg;
//        pvt_data[MAG_DEC_ACCURACY_TAG] = mag_deg_acc;
//    }
//
//    return pvt_data;
//}
//
