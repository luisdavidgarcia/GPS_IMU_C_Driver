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
#define PI 3.14159265359F
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
	void printAccel(void);
	void printMag(void);
	void printGyro(void);
	void readSensorData(void);

	// Rename function names to GetScaledAccelerometerData, etc.
    const int16_t* getAccelerometerData() {
		int8_t badRead = 0;
		// 0 = x, 1 = y, 2 = z
		for (int i = 0; i < 3; i++) {
			// if (accelerometer[i] > ACCEL_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else if (accelerometer[i] < -ACCEL_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else {
				if (accelerometer[i] < 0) {
					accelerometer[i] = accelerometer[i] * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD + ACCEL_SCALE;
				} else {
					accelerometer[i] = accelerometer[i] * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD - ACCEL_SCALE;
				}
			// }
		}

		if (badRead) {
			accelerometer[0] = accelerometer[1] = accelerometer[2] = ACCEL_MAX_THRESHOLD;
		}

        return accelerometer;
    }

    const int16_t* getMagnetometerData() {
		int8_t badRead = 0;
		for (int i = 0; i < 3; i++) {
			// if (magnetometer[i] > MAG_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else if (magnetometer[i] < -MAG_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else {
				magnetometer[i] = magnetometer[i] * MAG_UT_LSB;
			// }
		}

		if (badRead) {
			magnetometer[0] = magnetometer[1] = magnetometer[2] = MAG_MAX_THRESHOLD;
		}

        return magnetometer;
    }

    const int16_t* getGyroscopeData() {
		int8_t badRead = 0;
		for (int i = 0; i < 3; i++) {
			// if (gyroscope[i] > GYRO_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else if (gyroscope[i] < -GYRO_MAX_THRESHOLD) {
			// 	badRead = 1;
			// 	break;
			// } else {
				gyroscope[i] = gyroscope[i] * GYRO_SENSITIVITY_250DPS * DEG_TO_RAD;
			// }
		}

		if (badRead) {
			gyroscope[0] = gyroscope[1] = gyroscope[2] = GYRO_MAX_THRESHOLD;
		}

        return gyroscope;
    }
};

#endif // IMU_H
