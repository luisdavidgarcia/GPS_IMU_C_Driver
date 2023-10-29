#include "imu.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

Imu::Imu() {
    const char *deviceName = "/dev/i2c-1";
    i2c_fd = open(deviceName, O_RDWR);
    if (i2c_fd < 0) {
        perror("Unable to open I2C device");
    }

    if (ioctl(i2c_fd, I2C_SLAVE, SLAVE_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
    }
}

Imu::~Imu() {
    close(i2c_fd);
}

/**
 * @brief   Read sensor data from IMU over I2C.
 *          Replace this section with your actual I2C communication to read sensor data.
 *          For example:
 *          accelerometer[0] = <read accelerometer X value>;
 *          accelerometer[1] = <read accelerometer Y value>;
 *          accelerometer[2] = <read accelerometer Z value>;
 *          magnetometer[0] = <read magnetometer X value>;
 *          magnetometer[1] = <read magnetometer Y value>;
 *          magnetometer[2] = <read magnetometer Z value>;
 *          gyroscope[0] = <read gyroscope X value>;
 *          gyroscope[1] = <read gyroscope Y value>;
 *          gyroscope[2] = <read gyroscope Z value>;
 */
void Imu::readSensorData() {
    // Read accelerometer data
    uint8_t accelData[DATA_SIZE];
    i2c_smbus_read_i2c_block_data(i2c_fd, ACCEL_REG_START, DATA_SIZE, accelData);
    accelerometer[0] = (int16_t)(accelData[0] << BYTE_SHIFT_AMOUNT | accelData[1]);
    accelerometer[1] = (int16_t)(accelData[2] << BYTE_SHIFT_AMOUNT | accelData[3]);
    accelerometer[2] = (int16_t)(accelData[4] << BYTE_SHIFT_AMOUNT | accelData[5]);

    // Read magnetometer data
    uint8_t magData[DATA_SIZE];
    i2c_smbus_read_i2c_block_data(i2c_fd, MAGNETO_REG_START, DATA_SIZE, magData);
    magnetometer[0] = (int16_t)(magData[0] << BYTE_SHIFT_AMOUNT | magData[1]);
    magnetometer[1] = (int16_t)(magData[2] << BYTE_SHIFT_AMOUNT | magData[3]);
    magnetometer[2] = (int16_t)(magData[4] << BYTE_SHIFT_AMOUNT | magData[5]);

    // Read gyroscope data
    uint8_t gyroData[DATA_SIZE];
    i2c_smbus_read_i2c_block_data(i2c_fd, GYRO_REG_START, DATA_SIZE, gyroData);
    gyroscope[0] = (int16_t)(gyroData[0] << BYTE_SHIFT_AMOUNT | gyroData[1]);
    gyroscope[1] = (int16_t)(gyroData[2] << BYTE_SHIFT_AMOUNT | gyroData[3]);
    gyroscope[2] = (int16_t)(gyroData[4] << BYTE_SHIFT_AMOUNT | gyroData[5]);
}


/**
 * @brief   Print accelerometer data to console.
 */
void Imu::printAccel() {
    printf("Acceleration (m/s^2): (%0.3f, %0.3f, %0.3f)\n",
           accelerometer[0], accelerometer[1], accelerometer[2]);
}

/**
 * @brief   Print magnetometer data to console.
 */
void Imu::printMag() {
    printf("Magnetometer (uTesla): (%0.3f, %0.3f, %0.3f)\n",
           magnetometer[0], magnetometer[1], magnetometer[2]);
}

/**
 * @brief   Print gyroscope data to console.
 */
void Imu::printGyro() {
    printf("Gyroscope (radians/s): (%0.3f, %0.3f, %0.3f)\n",
           gyroscope[0], gyroscope[1], gyroscope[2]);
}

/**
 * @brief   Continuously print sensor data with a specified delay.
 * @param   delay   Delay in milliseconds between data prints.
 */
void Imu::Telementary(int delay) {
    while (true) {
        time_t t = time(NULL);
        struct tm *tm_info = localtime(&t);
        printf("%s", asctime(tm_info));
        readSensorData();
        printAccel();
        printMag();
        printGyro();
        usleep(delay * TIME_DELAY_MS);
    }
}
