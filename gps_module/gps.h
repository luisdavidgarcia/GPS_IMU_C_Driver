/*
 * GPS.h - Header file for GPS module integration
 *
 * This header file defines the interface for interacting with a GPS module
 * over the I2C communication protocol. It provides functions and data structures
 * to retrieve GPS-related information, including location, time, velocity, and more.
 * The code in this header is designed to work with a specific GPS module and may
 * require configuration and adaptation for different hardware or use cases.
 *
 * CREDIT/CODE MODIFIED FROM:
 * https://github.com/melopero/Melopero_SAM-M8Q_Arduino_Library/tree/master
 *
 * Dependencies:
 * - ubx_lib/ubx_msg.h: Defines UBX message structures for communication with the GPS module.
 * - fcntl.h, unistd.h, i2c/smbus.h, linux/i2c-dev.h, linux/i2c.h: Required for I2C communication.
 * - cstdint, sys/ioctl.h, string, chrono, thread, vector, cstring: Standard C++ libraries.
 *
 * Constants and Definitions:
 * - Various constants for I2C addresses, register addresses, timeouts, and default values.
 *
 * Data Structures:
 * - PVTData: A structure to hold GPS-related data, including time, coordinates, accuracy,
 *   velocity, and more.
 *
 * Class:
 * - Gps: A class that encapsulates GPS functionality, including I2C communication,
 *   message handling, and data retrieval.
 *
 * Usage:
 * - Include this header file in your C++ project to interact with a GPS module.
 * - Instantiate the Gps class to communicate with the GPS module and retrieve data.
 *
 * Note: This code is designed for a specific GPS module and may require adaptation for
 *       other GPS modules or hardware configurations. Refer to the provided credit and
 *       documentation for further details on usage and customization.
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
#include <chrono>
#include <thread>
#include <vector>
#include <cstring>

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

#define INVALID_YEAR_FLAG 0xBEEF
#define INVALID_SYNC_FLAG 255

typedef struct {
    // Time Information
    uint16_t year;               // Year (UTC)
    uint8_t month;               // Month (UTC)
    uint8_t day;                 // Day of the month (UTC)
    uint8_t hour;                // Hour of the day (UTC)
    uint8_t min;                 // Minute of the hour (UTC)
    uint8_t sec;                 // Second of the minute (UTC)

    // Validity Flags
    uint8_t validTimeFlag;       // Validity flags for time
    uint8_t validDateFlag;       // Validity flags for time
    uint8_t fullyResolved;       // Validity flags for time
    uint8_t validMagFlag;       // Validity flags for time

    // GNSS
    uint8_t gnssFix;
    uint8_t fixStatusFlags;
    uint8_t numberOfSatellites;

    // Coordinates
    int32_t longitude;           // Longitude (degrees * 1e7)
    int32_t latitude;            // Latitude (degrees * 1e7)
    int32_t height;              // Height above ellipsoid (millimeters)
    int32_t heightMSL;                // Height above mean sea level (millimeters)

    // Accuracy Information
    uint32_t horizontalAccuracy; // Horizontal accuracy estimate (millimeters)
    uint32_t verticalAccuracy;   // Vertical accuracy estimate (millimeters)

    // Velocity and Heading
    int32_t velocityNorth; // Velocity in the north direction (millimeters/second)
    int32_t velocityEast;   // Velocity in the east direction (millimeters/second)
    int32_t velocityDown;   // Velocity in the down direction (millimeters/second)
    int32_t groundSpeed;      // Ground speed (millimeters/second)
    int32_t vehicalHeading;
    int32_t motionHeading;     // Heading of motion (degrees * 1e5)
    uint32_t speedAccuracy;      // Speed accuracy estimate (millimeters/second)
    int32_t motionHeadingAccuracy;

    // Vehicle Heading and Magnetic Declination
    int16_t magneticDeclination; // Magnetic declination (degrees * 1e2)
    uint16_t magnetDeclinationAccuracy; // Declination accuracy (degrees * 1e2)
} PVTData;

class Gps {

	private:
		UbxMessage ubxmsg;
		PVTData pvtData;
		int i2c_fd;

		void ubxOnly(void);
		bool writeUbxMessage(UbxMessage& msg);
		uint16_t getAvailableBytes(void);
		UbxMessage readUbxMessage(void);
		bool setMessageSendRate(uint8_t msgClass, uint8_t msgId,
		uint8_t sendRate);
		bool setMeasurementFrequency(uint16_t measurementPeriodMillis,
		uint8_t navigationRate, uint8_t timeref);

		int16_t i2_to_int(const uint8_t *little_endian_bytes);
		uint16_t u2_to_int(const uint8_t *little_endian_bytes);
		int32_t i4_to_int(const uint8_t *little_endian_bytes);
		uint32_t u4_to_int(const uint8_t *little_endian_bytes);

  	public:
		Gps(void);
		~Gps(void);
		PVTData GetPvt(bool polling, uint16_t timeOutMillis);
};

#endif // GPS_H

