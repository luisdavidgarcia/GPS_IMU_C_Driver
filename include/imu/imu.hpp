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

extern "C" {
	#include <i2c/smbus.h>
	#include <linux/i2c-dev.h>
	#include <linux/i2c.h>
}
#include <cstdint>
#include <sys/ioctl.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

/** IMU Constants */
#define TIME_DELAY_MS 1000
#define ACCEL_MAG_DATA_SIZE 12
#define PI 3.14159265359f
#define DEG_TO_RAD PI / 180.0
#define RAD_TO_DEG 180.0 / PI
#define SENSORS_GRAVITY_STD 9.807F
#define GYRO_MAX_THRESHOLD 2200.0 // rad/s, adjust as needed
#define ACCEL_MAX_THRESHOLD 17.6 // m/s², adjust as needed
#define BYTE_SHIFT_AMOUNT 8
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define BITS_PER_BYTE 8
#define BYTE_MASK 0xFF
#define ACCEL_SCALE 0.3

/** I2C Specifics */
#define IMU_I2C_ADDRESS 0x69
#define IMU_I2C_BUS "/dev/i2c-1"
#define IMU_ID 0xEA

/** General Registers */
#define BANK_SEL 0x7F
#define BANK_REG_0 0x00
#define BANK_REG_1 0x10
#define BANK_REG_2 0x20
#define BANK_REG_3 0x30

/** Bank 0 Registers */
#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define INT_PIN_CFG 0x0F

/** Bank 3 Registers */
#define I2C_MST_CTRL 0x01
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define I2C_SLV4_ADDR 0x13
#define I2C_SLV4_REG 0x14
#define I2C_SLV4_CTRL 0x15
#define I2C_SLV4_DO 0x16

/** Gyroscope Registers */
#define GYRO_REG_START 0x00
#define GYRO_CONFIG 0x01
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

/** Accelerometer Registers */
#define ACCEL_REG_START 0x00
#define ACCEL_CONFIG 0x01
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

/** Magnetometer Registers */
#define MAGNETO_REG_START 0x00
#define MAGNETO_CONFIG 0x01
#define MAGNETO_XOUT_H 0x3C
#define MAGNETO_XOUT_L 0x3D
#define MAGNETO_YOUT_H 0x3E
#define MAGNETO_YOUT_L 0x3F
#define MAGNETO_ZOUT_H 0x40
#define MAGNETO_ZOUT_L 0x41

/** Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS (1/131.0F)
/** Gyroscope sensitivity at 500dps */
#define GYRO_SENSITIVITY_500DPS (1/65.5F) 
#define GYRO_DATA_SIZE 6
#define GYRO_CONFIG_VALUE 0x11 << 1

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (1/16384.0F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (1/8192.0F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (/4096.0F)

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.15)
#define MAG_MAX_THRESHOLD 5000 // µT, adjust as needed

const float alpha = 0.5; // Adjust this parameter to tweak the filter (range: 0-1)
const float accel_x_offset = -0.05673657500210876;
const float accel_y_offset = -0.014051752249833504;
const float accel_z_offset = -0.700553398935738;
const float gyro_x_bias = -0.008447830282040561;
const float gyro_y_bias = 0.0064697791963203004;
const float gyro_z_bias = -0.009548081446790717;

class Imu {
private:
	int i2c_fd;
	int16_t accelerometer[3];
	int16_t magnetometer[3];
	int16_t gyroscope[3];
	void begin(void);

public:
	Imu();
	~Imu();
	void ReadSensorData(void);

    const int16_t* GetRawAccelerometerData() { return accelerometer; }
    const int16_t* GetRawMagnetometerData() { return magnetometer; }
    const int16_t* GetRawGyroscopeData() { return gyroscope; }

	float GetAccelX() { return static_cast<float>(accelerometer[X_AXIS]) * ACCEL_MG_LSB_2G - accel_x_offset; }
	float GetAccelY() { return -1 * static_cast<float>(accelerometer[Y_AXIS]) * ACCEL_MG_LSB_2G - accel_y_offset; }
	float GetAccelZ() { return static_cast<float>(accelerometer[Z_AXIS]) * ACCEL_MG_LSB_2G - accel_z_offset; }
	float GetGyroX() { return (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyroscope[X_AXIS]) - gyro_x_bias); }
	float GetGyroY() { return (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyroscope[Y_AXIS]) - gyro_y_bias); }
	float GetGyroZ() { return (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyroscope[Z_AXIS]) - gyro_z_bias); }
	float GetMagX() { return static_cast<float>(magnetometer[X_AXIS]) * MAG_UT_LSB; }
	float GetMagY() { return static_cast<float>(magnetometer[Y_AXIS]) * MAG_UT_LSB; }
	float GetMagZ() { return static_cast<float>(magnetometer[Z_AXIS]) * MAG_UT_LSB; }
};

#endif // IMU_H
