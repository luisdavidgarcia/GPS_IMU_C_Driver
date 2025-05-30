#include "GPS/GPS.hpp"
#include "UBX/UBX_MSG.hpp"

/**
 * @brief   Constructor for the GPS class.
 *
 * Initializes the GPS module communication, sets message send rates, and measurement frequencies.
 */
GPS::GPS(int16_t currentYear) : currentYear_(currentYear), pvtData_{} 
{
	const char *deviceName = GPS_I2C_BUS;
	i2c_fd_ = open(deviceName, O_RDWR | O_CLOEXEC);
	if (i2c_fd_ < 0) {
		perror("Unable to open I2C GPS device");
	}

	if (ioctl(i2c_fd_, I2C_SLAVE, GPS_I2C_ADDRESS) < 0) {
		perror("Failed to acquire I2C GPS address");
	}

	this->ubxSetup();

	bool result = this->setMessageSendRate(DEFAULT_SEND_RATE);
	if (!result) {
		printf("Error: Failed to set message send rate for NAV_PVT.\n");
		exit(-1);
	}
	MeasurementParams params = {
		MEASUREMENT_PERIOD_MILLIS_100_MS,
		DEFAULT_SEND_RATE,
		DEFAULT_TIME_REF
	};
	result = this->setMeasurementFrequency(params);
	if (!result) {
		printf("Error: Failed to set measurement frequency.\n");
		exit(-1);
	}

	if (currentYear == DEFAULT_YEAR) {
		printf("Error: Current year is not set.\n");
		exit(-1);
	}
}

/**
 * @brief   Destructor for the GPS class.
 *
 * Closes the communication with the GPS module.
 */
GPS::~GPS() { close(i2c_fd_); }

/**
 * @brief   Configure the GPS module to use UBX protocol exclusively.
 *
 * Sends a UBX configuration message to the GPS module to use UBX protocol only.
 */
void GPS::ubxSetup() 
{
	std::vector<uint8_t> payload = {
			0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	MessageInfo classANDFlag = { CFG_CLASS, CFG_PRT };

	ubx.ComposeMessage(classANDFlag, payload);

	bool result = this->writeUbxMessage();
	if (!result) {
		exit(-1);
	}
}

/**
 * @brief   Set the message send rate for a specific UBX message.
 *
 * @param   sendRate    The desired message send rate (default is DEFAULT_SEND_RATE).
 * @return  true if the message send rate was successfully set, false otherwise.
 */
bool GPS::setMessageSendRate(uint8_t sendRate) {
	std::vector<uint8_t> payload = { sendRate, 0x00, 0x00, 0x00, 0x00, 0x00 };

	MessageInfo classANDFlag = { CFG_CLASS, CFG_MSG };
	ubx_.ComposeMessage(classANDFlag, payload);

	return this->writeUbxMessage();
}

/**
 * @brief   Set the measurement frequency of the GPS module.
 *
 * @param   params  Contains the measurementPeriodMillis (default is DEFAULT_UPDATE_MILLS),
 *    							navigationRate the navigation rate (default is 1), and
 *    							timeref the time reference (default is 0).
 * @return  true if the measurement frequency was successfully set, false otherwise.
 */
bool GPS::setMeasurementFrequency(const MeasurementParams& params)
{
	std::vector<uint8_t> payload(6,0);

	payload[0] = static_cast<uint8_t>(params.measurementPeriodMillis & BYTE_MASK);
	payload[1] = static_cast<uint8_t>(
		(params.measurementPeriodMillis >> BYTE_SHIFT_AMOUNT) & BYTE_MASK
	);
	payload[2] = params.navigationRate;
	payload[3] = 0x00;
	payload[4] = params.timeref;
	payload[5] = 0x00;

	MessageInfo classANDFlag = { CFG_CLASS, CFG_RATE };
	ubx_.ComposeMessage(classANDFlag, payload);

	return this->writeUbxMessage();
}

/**
 * @brief   Retrieve the number of available bytes for reading from the GPS module.
 *
 * @return  The number of available bytes.
 */
uint16_t GPS::getAvailableBytes() const
{
  uint8_t msb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_MSB);
  uint8_t lsb = i2c_smbus_read_byte_data(i2c_fd, AVAILABLE_BYTES_LSB);

  uint16_t availableBytes = (msb << BYTE_SHIFT_AMOUNT) | lsb;
  return availableBytes;
}

/**
 * @brief   Write a UBX message to the GPS module.
 *
 * @return  true if the message was successfully written, false otherwise.
 */
bool GPS::writeUbxMessage() const
{
	UbxMessage ubxMessage = ubx_.GetUBXMessage();
	std::vector<uint8_t> tempBuf;
	tempBuf.push_back(ubxMessage.sync1);
	tempBuf.push_back(ubxMessage.sync2);
	tempBuf.push_back(ubxMessage.msgClass);
	tempBuf.push_back(ubxMessage.msgId);
	tempBuf.push_back(ubxMessage.payloadLength & BYTE_MASK);
	tempBuf.push_back(ubxMessage.payloadLength >> BYTE_SHIFT_AMOUNT);
	for (int i = 0; i < ubxMessage.payloadLength; i++) {
		tempBuf.push_back(ubxMessage.payload.at(i));
	}
	tempBuf.push_back(ubxMessage.checksumA);
	tempBuf.push_back(ubxMessage.checksumB);

	if (write(i2c_fd_, tempBuf.data(), tempBuf.size()) != tempBuf.size()) {
		perror("Failed to write to I2C device");
		close(i2c_fd_);
		return false;
	}

	return true;
}

/**
 * @brief   Read a UBX message from the GPS module.
 *
 * @return  The read UBX message.
 */
UbxMessage GPS::readUbxMessage() 
{
	uint16_t messageLength = getAvailableBytes();
  std::vector<uint8_t> message;

  if (messageLength > 2 && messageLength < MAX_MESSAGE_LENGTH) {
		for (int i = 0; i < messageLength; i++) {
			int32_t byte_data = i2c_smbus_read_byte_data(i2c_fd_, DATA_STREAM_REGISTER);
			if (byte_data < -1) {
				perror("Failed to read byte from I2C device");
				UbxMessage badMsg = {
					INVALID_SYNC_FLAG
				};
				return badMsg;
			}
			message.push_back(static_cast<uint8_t>(byte_data));
		}

		if (message[0] == SYNC_CHAR_1 && message[1] == SYNC_CHAR_2) {
			UbxMessage ubxMsg = {};
			ubxMsg.sync1 = message[0];
			ubxMsg.sync2 = message[1];
			ubxMsg.msgClass = message[2];
			ubxMsg.msgId = message[3];
			ubxMsg.payloadLength = (message[5] << BYTE_SHIFT_AMOUNT | message[4]);
			if (ubxMsg.payloadLength > messageLength) {
				UbxMessage badMsg = {
					INVALID_SYNC_FLAG
				};
				return badMsg;
			}
			memcpy(&ubxMsg.payload, &message[6], ubxMsg.payloadLength);
			ubxMsg.checksumA = message[message.size() - 2];
			ubxMsg.checksumB = message[message.size() - 1];

			return ubxMsg;
		}
	}

	UbxMessage badMsg = {
		INVALID_SYNC_FLAG
	};
	return badMsg;
}

/**
 * @brief   Retrieve Position-Velocity-Time (PVT) data from the GPS module.
 *
 * @param   polling         Whether to poll the GPS module for new data (default is DEFAULT_POLLING_STATE).
 * @return  The PVTData structure containing GPS-related information.
 */
PVTData GPS::GetPvt(bool polling)
{
	if (polling) {
		MessageInfo classANDFlag = { NAV_CLASS, NAV_PVT };
		ubx_.ComposeMessage(classANDFlag, {});
		this->writeUbxMessage();
	}

	UbxMessage message = this->readUbxMessage();

	if (message.sync1 != INVALID_SYNC1_FLAG) {
		pvtData_.year = u2_to_int(
			std::span<const uint8_t, 2>(&message.payload[4],2)
		);
		if (pvtData_.year != currentYear) {
			pvtData_.year = INVALID_YEAR_FLAG;
			return this->pvtData_;
		}
		pvtData_.month = message.payload[6];
		pvtData_.day = message.payload[7];
		pvtData_.hour = message.payload[8];
		pvtData_.min = message.payload[9];
		pvtData_.sec = message.payload[10];
		uint8_t valid_flag = message.payload[11];

		// Extract and clarify flags
		pvtData_.validDateFlag = (valid_flag & VALID_DATE_FLAG) == VALID_DATE_FLAG ? 1 : 0;
		pvtData_.validTimeFlag = (valid_flag & VALID_TIME_FLAG) == VALID_TIME_FLAG ? 1 : 0;
		pvtData_.fullyResolved = (valid_flag & FULLY_RESOLVED_FLAG) == FULLY_RESOLVED_FLAG ? 1 : 0;
		pvtData_.validMagFlag = (valid_flag & VALID_MAG_FLAG) == VALID_MAG_FLAG ? 1 : 0;

		// Extract GNSS fix and related data
		pvtData_.gnssFix = message.payload[20];
		memcpy(&pvtData_.fixStatusFlags, &message.payload[21], 2);
		pvtData_.numberOfSatellites = message.payload[23];

		// Extract longitude and latitude in degrees
		pvtData_.longitude = static_cast<float>(
			i4_to_int(std::span<const uint8_t, 4>(&message.payload[24],4)
		)) * LONGTITUDE_SCALE;
		pvtData_.latitude = static_cast<float>(
			i4_to_int(std::span<const uint8_t, 4>(&message.payload[28],4)
		)) * LATTITUDE_SCALE;
		pvtData_.height = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[32],4)
		);
		pvtData_.heightMSL = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[36],4)
		);
		// Extract horizontal and vertical accuracy in millimeters
		pvtData_.horizontalAccuracy = u4_to_int(
			std::span<const uint8_t, 4>(&message.payload[40],4)
		);
		pvtData_.verticalAccuracy = u4_to_int(
			std::span<const uint8_t, 4>(&message.payload[44],4)
		);
		// Extract North East Down velocity in mm/s
		pvtData_.velocityNorth = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[48],4)
		);
		pvtData_.velocityEast = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[52],4)
		);
		pvtData_.velocityDown = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[56],4)
		);
		// Extract ground speed in mm/s and motion heading in degrees
		pvtData_.groundSpeed = i4_to_int(
			std::span<const uint8_t, 4>(&message.payload[60],4)
		);
		pvtData_.motionHeading = static_cast<float>(
			i4_to_int(std::span<const uint8_t, 4>(&message.payload[64],4)
		)) * MOTION_HEADING_SCALE;
		// Extract speed accuracy in mm/s and heading accuracy in degrees
		pvtData_.speedAccuracy = u4_to_int(
			std::span<const uint8_t, 4>(&message.payload[68],4)
		);
		pvtData_.motionHeadingAccuracy = static_cast<float>(
			u4_to_int(std::span<const uint8_t, 4>(&message.payload[72],4)
		)) * MOTION_HEADING_ACCURACY_SCALE;
		// Extract vehicle heading in degrees
		pvtData_.vehicalHeading = static_cast<float>(
			i4_to_int(std::span<const uint8_t, 4>(&message.payload[84],4)
		)) * VEHICLE_HEADING_SCALE;
		// Extract magnetic declination and accuracy in degrees
		pvtData_.magneticDeclination = static_cast<float>(
			i2_to_int(std::span<const uint8_t, 2>(&message.payload[88],4)
		)) * MAGNETIC_DECLINATION_SCALE;
		pvtData_.magnetDeclinationAccuracy = static_cast<float>(
			u2_to_int(std::span<const uint8_t, 2>(&message.payload[90],4)
		)) * MAGNETIC_DECLINATION_ACCURACY_SCALE;

		return this->pvtData_;
	}

	pvtData_.year = INVALID_YEAR_FLAG;
	return this->pvtData_;
}

/**
 * @brief   Convert a little-endian byte array to a signed 16-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted signed 16-bit integer.
 */
int16_t GPS::i2_to_int(std::span<const uint8_t, 2> bytes) 
{
	auto byte0 = static_cast<uint16_t>(bytes[0]);
	auto byte1 = static_cast<uint16_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;
	
	return static_cast<int16_t>(byte1 | byte0);
}

/**
 * @brief   Convert a little-endian byte array to an unsigned 16-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted unsigned 16-bit integer.
 */
uint16_t GPS::u2_to_int(std::span<const uint8_t, 2> bytes) 
{
	auto byte0 = static_cast<uint16_t>(bytes[0]);
	auto byte1 = static_cast<uint16_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;

	return byte1 | byte0;
}

double GPS::bytes_to_double(const uint8_t *little_endian_bytes) {
	// Assuming little_endian_bytes is a representation of a double
	// If it's not, you'll need to adjust the conversion logic accordingly
	double result = 0;
	memcpy(&result, little_endian_bytes, sizeof(result));
	printf("Result: %f\n", result);
	return result * 1e-07;
}

/**
 * @brief   Convert a little-endian byte array to a signed 32-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted signed 32-bit integer.
 */
int32_t GPS::i4_to_int(std::span<const uint8_t, 4> bytes) 
{
  auto byte0 = static_cast<uint32_t>(bytes[0]);
	auto byte1 = static_cast<uint32_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;
	auto byte2 = static_cast<uint32_t>(bytes[2]) << HALF_WORD_SHIFT_AMOUNT;
	auto byte3 = static_cast<uint32_t>(bytes[3]) << THREE_BYTE_SHIFT_AMOUNT;

	return static_cast<int32_t>(byte3 | byte2 | byte1 | byte0);
}

/**
 * @brief   Convert a little-endian byte array to an unsigned 32-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted unsigned 32-bit integer.
 */
uint32_t GPS::u4_to_int(std::span<const uint8_t, 4> bytes)
{
  auto byte0 = static_cast<uint32_t>(bytes[0]);
	auto byte1 = static_cast<uint32_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;
	auto byte2 = static_cast<uint32_t>(bytes[2]) << HALF_WORD_SHIFT_AMOUNT;
	auto byte3 = static_cast<uint32_t>(bytes[3]) << THREE_BYTE_SHIFT_AMOUNT;

	return (byte3 | byte2 | byte1 | byte0);
}
