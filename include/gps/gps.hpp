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
 * - ubx/ubx_msg.hpp: Defines UBX message structures for communication with the GPS module.
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

#include "ubx/ubx_msg.hpp"

#include <fcntl.h>
#include <span>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
extern "C" {
	#include <i2c/smbus.h>
	#include <linux/i2c-dev.h>
	#include <linux/i2c.h>
}

/** I2C Specifics */
constexpr uint8_t GPS_I2C_ADDRESS = 0x42;
constexpr const char* GPS_I2C_BUS = "/dev/i2c-1";

constexpr int BYTE_SHIFT_AMOUNT = 8;
constexpr int BYTE_MASK = 0xFF;
constexpr int HALF_WORD_SHIFT_AMOUNT = 16;
constexpr int THREE_BYTE_SHIFT_AMOUNT = 24;

constexpr int DATA_STREAM_REGISTER = 0xFF;

constexpr int AVAILABLE_BYTES_MSB = 0xFD;
constexpr int AVAILABLE_BYTES_LSB = 0xFE;

constexpr int DEFAULT_TIMEOUT_MILLS = 2000;
constexpr int DEFAULT_UPDATE_MILLS = 1000;
constexpr int DEFAULT_SEND_RATE = 0x01;
constexpr int DEFAULT_INTERVAL_MILLS = 50;
constexpr bool DEFAULT_POLLING_STATE = false;
constexpr int DEFAULT_YEAR = -1;

constexpr int VALID_DATE_FLAG = 0x01;
constexpr int VALID_TIME_FLAG = 0x02;
constexpr int FULLY_RESOLVED_FLAG = 0x04;
constexpr int VALID_MAG_FLAG = 0x08;
constexpr int INVALID_SYNC1_FLAG = 0xFF;

/** SAM-M8Q Limits */
constexpr int MAX_MONTH = 12;
constexpr int MIN_MONTH = 1;
constexpr int MAX_DAY = 31;
constexpr int MIN_DAY = 1;
constexpr int MAX_HOUR = 23;
constexpr int MIN_HOUR = 0;
constexpr int MAX_MINUTE = 59;
constexpr int MIN_MINUTE = 0;
constexpr int MAX_SECOND = 59;
constexpr int MIN_SECOND = 0;
constexpr float MAX_LONGTITUTE = 180.0F;
constexpr float MIN_LONGTITUTE = -180.0F;
constexpr float MAX_LATITUDE = 90.0F;
constexpr float MIN_LATITUDE = -90.0F;
constexpr int MAX_DEGREE = 360;
constexpr int MIN_DEGREE = 0;
constexpr int MAX_MAG_DEGREE_ACCURACY = 180;

constexpr float MAX_DYNAMICS_G = 4.0F;                        // Maximum dynamics in g
constexpr float MAX_ALTITUDE_METERS = 50000.0F;               // Maximum altitude in meters
constexpr float MIN_ALTITUDE_METERS = -50000.0F;              // Minimum altitude in meters
constexpr float MAX_VELOCITY_MPS = 500.0F;                    // Maximum velocity in meters per second
constexpr float MIN_VELOCITY_MPS = -500.0F;                   // Minimum velocity in meters per second
constexpr float VELOCITY_ACCURACY_THRESHOLD_MPS = 0.05F;      // Velocity accuracy in meters per second
constexpr float HEADING_ACCURACY_DEGREES = 0.3F;              // Heading accuracy in degrees

constexpr float HORIZONTAL_ACCURACY_GPS_GLONASS_M = 2.5F;     // Horizontal position accuracy for GPS & GLONASS in meters
constexpr float HORIZONTAL_ACCURACY_GALILEO_M = 8.0F;         // Horizontal position accuracy for Galileo in meters (To be confirmed)

constexpr int MAX_NAVIGATION_UPDATE_RATE_HZ_GPS = 10;         // Max navigation update rate for GPS in Hz
constexpr int MAX_NAVIGATION_UPDATE_RATE_HZ_OTHER = 18;       // Max navigation update rate for GLONASS and Galileo in Hz

constexpr int COLD_START_TTFF_SECONDS = 26;                   // Time-To-First-Fix for cold start in seconds
constexpr int HOT_START_TTFF_SECONDS = 1;                     // Time-To-First-Fix for hot start in seconds
constexpr int AIDED_START_TTFF_SECONDS = 2;                   // Time-To-First-Fix for aided starts in seconds

constexpr int SENSITIVITY_TRACK_NAV_DBM = -165;               // Sensitivity for tracking & navigation in dBm
constexpr int SENSITIVITY_REACQUISITION_DBM = -158;           // Sensitivity for reacquisition in dBm
constexpr int SENSITIVITY_COLD_HOT_START_DBM = -146;          // Sensitivity for cold and hot starts in dBm

constexpr int MEASUREMENT_PERIOD_MILLIS_1_SEC = 1000;         // Measurement period in milliseconds
constexpr int MEASUREMENT_PERIOD_MILLIS_100_MS = 100;         // Measurement period in milliseconds
constexpr int MEASUREMENT_PERIOD_MILLIS_25_MS = 25;           // Measurement period in milliseconds

constexpr float LONGTITUDE_SCALE = 1e-07F;
constexpr float LATTITUDE_SCALE = 1e-07F;
constexpr float MOTION_HEADING_SCALE = 1e-05F;
constexpr float MOTION_HEADING_ACCURACY_SCALE = 1e-05F;
constexpr float VEHICLE_HEADING_SCALE = 1e-05F;
constexpr float MAGNETIC_DECLINATION_SCALE = 1e-02F;
constexpr float MAGNETIC_DECLINATION_ACCURACY_SCALE = 1e-02F;

constexpr int DEFAULT_NAVIGATION_RATE = 1;
constexpr int DEFAULT_TIME_REF = 0;

/** Error Handling */
constexpr int INVALID_YEAR_FLAG = 0xBEEF;
constexpr int INVALID_SYNC_FLAG = 255;

struct alignas(128) PVTData {
  // Time Information
  uint16_t year;                        // Year (UTC)
  uint8_t month;                        // Month (UTC)
  uint8_t day;                          // Day of the month (UTC)
  uint8_t hour;                         // Hour of the day (UTC)
  uint8_t min;                          // Minute of the hour (UTC)
  uint8_t sec;                          // Second of the minute (UTC)

  // Validity Flags
  uint8_t validTimeFlag;                // Validity flags for time
  uint8_t validDateFlag;                // Validity flags for time
  uint8_t fullyResolved;                // Validity flags for time
  uint8_t validMagFlag;                 // Validity flags for time

  // GNSS
  uint8_t gnssFix;
  uint8_t fixStatusFlags;
  uint8_t numberOfSatellites;

  // Coordinates
  float longitude;
  float latitude;                      // Latitude (degrees * 1e7)
  int32_t height;                       // Height above ellipsoid (millimeters)
  int32_t heightMSL;                    // Height above mean sea level (millimeters)

  // Accuracy Information
  uint32_t horizontalAccuracy;          // Horizontal accuracy estimate (millimeters)
  uint32_t verticalAccuracy;            // Vertical accuracy estimate (millimeters)

  // Velocity and Heading
  int32_t velocityNorth;                // Velocity in the north direction (millimeters/second)
  int32_t velocityEast;                 // Velocity in the east direction (millimeters/second)
  int32_t velocityDown;                 // Velocity in the down direction (millimeters/second)
  int32_t groundSpeed;                  // Ground speed (millimeters/second)
  float vehicalHeading;
  float motionHeading;                // Heading of motion (degrees * 1e5)
  uint32_t speedAccuracy;               // Speed accuracy estimate (millimeters/second)
  float motionHeadingAccuracy;

  // Magnetic Declination
  float magneticDeclination;          // Magnetic declination (degrees * 1e2)
  float magnetDeclinationAccuracy;   // Declination accuracy (degrees * 1e2)
};

struct alignas(4) MeasurementParams {
	uint16_t measurementPeriodMillis = DEFAULT_UPDATE_MILLS;
	uint8_t navigationRate = DEFAULT_NAVIGATION_RATE;
	uint8_t timeref = DEFAULT_TIME_REF;
};

class GPS {
private:
  UBX ubx;
  PVTData pvtData;
  int i2c_fd;
  int16_t currentYear;

  void ubxSetup();
  bool writeUbxMessage() const;
  uint16_t getAvailableBytes() const;
  UbxMessage readUbxMessage();
  bool setMessageSendRate(uint8_t sendRate = DEFAULT_SEND_RATE);
  bool setMeasurementFrequency(const MeasurementParams& params);

  static double bytes_to_double(const uint8_t *little_endian_bytes);
  static int16_t i2_to_int(std::span<const uint8_t, 2> bytes);
  static uint16_t u2_to_int(std::span<const uint8_t, 2> bytes);
  static int32_t i4_to_int(std::span<const uint8_t, 4> bytes);
  static uint32_t u4_to_int(std::span<const uint8_t, 4> bytes);

public:
  explicit GPS(int16_t currentYear);
  ~GPS();
  PVTData GetPvt(bool polling);
};

#endif // GPS_H

