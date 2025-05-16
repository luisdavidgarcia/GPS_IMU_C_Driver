/*
 * imu.h - Header file for IMU (Inertial Measurement Unit) sensor integration
 *
 * This header file provides an interface for interacting with an IMU sensor over
 * the I2C communication protocol. It includes functions and constants for retrieving
 * accelerometer, gyroscope, and magnetometer data from the IMU sensor. The code is
 * designed for a specific IMU sensor and may require configuration adjustments for
 * different hardware or use cases.
 *
 * CREDIT/CODE MODIFIED FROM:
 * https://github.com/GoScoutOrg/Rover/blob/749a7758aef85ed877ad6db56e223f91a708abdf/src/rover/imu.py
 *
 * Dependencies:
 * - i2c/smbus.h, linux/i2c-dev.h, linux/i2c.h: Required for I2C communication.
 * - cstdint, sys/ioctl.h, time.h: Standard C++ libraries.
 *
 * Constants and Definitions:
 * - Various constants for I2C addresses, register addresses, data sizes, and sensitivities.
 *
 * Class:
 * - Imu: A class that encapsulates IMU functionality, including I2C communication,
 *   sensor data retrieval, and telemetry functions.
 *
 * Usage:
 * - Include this header file in your C++ project to interact with an IMU sensor.
 * - Instantiate the Imu class to communicate with the IMU sensor and retrieve data.
 *
 * Note: This code is designed for a specific IMU sensor and may require adaptation for
 *       other IMU sensors or hardware configurations. Refer to the provided credit and
 *       documentation for further details on usage and customization.
 */

#ifndef IMU_H
#define IMU_H

#include <sys/cdefs.h>
extern "C" {
	#include <i2c/smbus.h>
	#include <linux/i2c-dev.h>
	#include <linux/i2c.h>
}
#include <cstdint>
#include <sys/ioctl.h>
#include <ctime>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

/** IMU Constants */
constexpr uint16_t TIME_DELAY_MS = 1000;
constexpr size_t ACCEL_MAG_DATA_SIZE = 12;
constexpr float MATH_PI = 3.1416;
constexpr float DEG_TO_RAD = MATH_PI / 180.0F;
constexpr float RAD_TO_DEG = 180.0F / MATH_PI;
constexpr float SENSORS_GRAVITY_STD = 9.807F;
constexpr float GYRO_MAX_THRESHOLD_RPS = 2200.0F;
constexpr float ACCEL_MAX_THRESHOLD_MPS2 = 17.6F;
constexpr int BYTE_SHIFT_AMOUNT = 8;
constexpr int X_AXIS = 0;
constexpr int Y_AXIS = 1;
constexpr int Z_AXIS = 2;
constexpr int BITS_PER_BYTE = 8;
constexpr int BYTE_MASK = 0xFF;
constexpr float ACCEL_SCALE = 0.3F;

/** I2C Specifics */
constexpr uint8_t IMU_I2C_ADDRESS = 0x69;
constexpr const char* IMU_I2C_BUS = "/dev/i2c-1";
constexpr uint8_t IMU_ID = 0xEA;

/** General Registers */
constexpr uint8_t BANK_SEL = 0x7F;
constexpr uint8_t BANK_REG_0 = 0x00;
constexpr uint8_t BANK_REG_1 = 0x10;
constexpr uint8_t BANK_REG_2 = 0x20;
constexpr uint8_t BANK_REG_3 = 0x30;

/** Bank 0 Registers */
constexpr uint8_t WHO_AM_I = 0x00;
constexpr uint8_t PWR_MGMT_1 = 0x06;
constexpr uint8_t INT_PIN_CFG = 0x0F;

/** Bank 3 Registers */
constexpr uint8_t I2C_MST_CTRL = 0x01;
constexpr uint8_t I2C_SLV0_ADDR = 0x03;
constexpr uint8_t I2C_SLV0_REG = 0x04;
constexpr uint8_t I2C_SLV0_CTRL = 0x05;
constexpr uint8_t I2C_SLV4_ADDR = 0x13;
constexpr uint8_t I2C_SLV4_REG = 0x14;
constexpr uint8_t I2C_SLV4_CTRL = 0x15;
constexpr uint8_t I2C_SLV4_DO = 0x16;

/** Gyroscope Registers */
constexpr uint8_t GYRO_REG_START = 0x00;
constexpr uint8_t GYRO_CONFIG = 0x01;
constexpr uint8_t GYRO_XOUT_H = 0x33;
constexpr uint8_t GYRO_XOUT_L = 0x34;
constexpr uint8_t GYRO_YOUT_H = 0x35;
constexpr uint8_t GYRO_YOUT_L = 0x36;
constexpr uint8_t GYRO_ZOUT_H = 0x37;
constexpr uint8_t GYRO_ZOUT_L = 0x38;

/** Accelerometer Registers */
constexpr uint8_t ACCEL_REG_START = 0x00;
constexpr uint8_t ACCEL_CONFIG = 0x01;
constexpr uint8_t ACCEL_XOUT_H = 0x2D;
constexpr uint8_t ACCEL_XOUT_L = 0x2E;
constexpr uint8_t ACCEL_YOUT_H = 0x2F;
constexpr uint8_t ACCEL_YOUT_L = 0x30;
constexpr uint8_t ACCEL_ZOUT_H = 0x31;
constexpr uint8_t ACCEL_ZOUT_L = 0x32;

/** Magnetometer Registers */
constexpr uint8_t MAGNETO_REG_START = 0x00;
constexpr uint8_t MAGNETO_CONFIG = 0x01;
constexpr uint8_t MAGNETO_XOUT_H = 0x3C;
constexpr uint8_t MAGNETO_XOUT_L = 0x3D;
constexpr uint8_t MAGNETO_YOUT_H = 0x3E;
constexpr uint8_t MAGNETO_YOUT_L = 0x3F;
constexpr uint8_t MAGNETO_ZOUT_H = 0x40;
constexpr uint8_t MAGNETO_ZOUT_L = 0x41;

/** Gyroscope sensitivity at 250dps */
constexpr float GYRO_SENSITIVITY_250DPS = 1.0F / 131.0F;
/** Gyroscope sensitivity at 500dps */
constexpr float GYRO_SENSITIVITY_500DPS = 1.0F / 65.5F;
constexpr size_t GYRO_DATA_SIZE = 6;
constexpr uint8_t GYRO_CONFIG_VALUE = 0x11 << 1;

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
constexpr float ACCEL_MG_LSB_2G = 1.0F / 16384.0F;
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
constexpr float ACCEL_MG_LSB_4G = 1.0F / 8192.0F;
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
constexpr float ACCEL_MG_LSB_8G = 1.0F / 4096.0F;

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
constexpr float MAG_UT_LSB = 0.15F;
constexpr float MAG_MAX_THRESHOLD = 5000.0F; // µT, adjust as needed

/** Filter constants */
constexpr float ALPHA 			   = 0.5F; // (range: 0-1)
constexpr float ACCEL_X_OFFSET = -0.0567F;
constexpr float ACCEL_Y_OFFSET = -0.0141F;
constexpr float ACCEL_Z_OFFSET = -0.7006F;
constexpr float GYRO_X_BIAS 	 = -0.00845F;
constexpr float GYRO_Y_BIAS    = 0.00647F;
constexpr float GYRO_Z_BIAS    = -0.00955F;

struct alignas(8) AccelMPS2 
{
	int16_t x; // m/s^2
	int16_t y; // m/s^2
	int16_t z; // m/s^2
};

struct alignas(8) GyroRPS 
{
	int16_t x; // rad/s
	int16_t y; // rad/s
	int16_t z; // rad/s
};

struct alignas(8) MagUT 
{
	int16_t x; // µT
	int16_t y; // µT
	int16_t z; // µT
};

struct alignas(16) ScaledAccelMPS2 
{
	float x; // m/s^2
	float y; // m/s^2
	float z; // m/s^2
};

struct alignas(16) ScaledGyroRPS
{
	float x; // rad/s
	float y; // rad/s
	float z; // rad/s
};

struct alignas(16) ScaledMagUT 
{
	float x; // µT
	float y; // µT
	float z; // µT
};

class IMU
{
private:
	int i2c_fd;
	AccelMPS2 accelerometer;
	GyroRPS gyroscope;
	MagUT magnetometer;
	void begin() const;

public:
	IMU();
	~IMU();

	void Read();

	const AccelMPS2& GetRawAcceleration() const { 
		return accelerometer; 
	}
	const GyroRPS& GetRawAngularVelocity() { 
		return gyroscope; 
	}
	const MagUT& GetRawMagneticField() const { 
		return magnetometer; 
	}

	ScaledAccelMPS2 GetScaledAcceleration() const { 
		ScaledAccelMPS2 acceleration = {
			static_cast<float>(accelerometer.x) * ACCEL_MG_LSB_2G - ACCEL_X_OFFSET,
			static_cast<float>(accelerometer.y) * ACCEL_MG_LSB_2G - ACCEL_Y_OFFSET,
			static_cast<float>(accelerometer.z) * ACCEL_MG_LSB_2G - ACCEL_Z_OFFSET,
		};

		return acceleration;
	}
	
	ScaledGyroRPS GetScaledAngularVelocity() const {
		ScaledGyroRPS angularVelocity = {
			static_cast<float>(gyroscope.x) * GYRO_SENSITIVITY_250DPS * DEG_TO_RAD - GYRO_X_BIAS,
			static_cast<float>(gyroscope.x) * GYRO_SENSITIVITY_250DPS * DEG_TO_RAD - GYRO_Y_BIAS,
			static_cast<float>(gyroscope.x) * GYRO_SENSITIVITY_250DPS * DEG_TO_RAD - GYRO_Z_BIAS,
		};

		return angularVelocity;	
	}
	
	ScaledMagUT GetScaledMagneticField() const {
		ScaledMagUT magneticField = {
			static_cast<float>(magnetometer.x) * MAG_UT_LSB,
			static_cast<float>(magnetometer.y) * MAG_UT_LSB,
			static_cast<float>(magnetometer.z) * MAG_UT_LSB,
		};

		return magneticField;
	}
};

#endif // IMU_H