#include "imu.h"
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

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

Imu::~Imu() { close(i2c_fd); }

/**
 * @brief   Read sensor data from IMU over I2C.
 *          Replace this section with your actual I2C communication to read
 * sensor data. For example: accelerometer[0] = <read accelerometer X value>;
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
  // uint8_t buffer[DATA_SIZE];
  uint8_t accel_mag_buffer[ACCEL_MAG_DATA_SIZE];
  uint8_t buf[1];
  buf[0] = i2c_smbus_read_byte_data(i2c_fd, 0x00);
  buf[1] = i2c_smbus_read_byte_data(i2c_fd, 0x46);
  printf("Read First Byte: 0x%x%x\n", buf[0], buf[1]);
  i2c_smbus_write_byte(i2c_fd, ACCEL_REG_START);
  i2c_smbus_read_i2c_block_data(i2c_fd, ACCEL_REG_START, ACCEL_MAG_DATA_SIZE,
                                accel_mag_buffer);

  // i2c_smbus_read_i2c_block_data(i2c_fd, ACCEL_REG_START, DATA_SIZE, buffer);
  accelerometer[0] = (int16_t)(accel_mag_buffer[0] << BYTE_SHIFT_AMOUNT |
                               accel_mag_buffer[1]) >>
                     2;
  accelerometer[1] = (int16_t)(accel_mag_buffer[2] << BYTE_SHIFT_AMOUNT |
                               accel_mag_buffer[3]) >>
                     2;
  accelerometer[2] = (int16_t)(accel_mag_buffer[4] << BYTE_SHIFT_AMOUNT |
                               accel_mag_buffer[5]) >>
                     2;

  // Convert raw accelerometer data to M/S^2
  accelerometer[0] *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
  accelerometer[1] *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
  accelerometer[2] *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;

  // Read magnetometer data
  // uint8_t buffer[DATA_SIZE];
  // i2c_smbus_read_i2c_block_data(i2c_fd, MAGNETO_REG_START, DATA_SIZE,
  // buffer);
  magnetometer[0] =
      (int16_t)(accel_mag_buffer[6] << BYTE_SHIFT_AMOUNT | accel_mag_buffer[7]);
  magnetometer[1] =
      (int16_t)(accel_mag_buffer[8] << BYTE_SHIFT_AMOUNT | accel_mag_buffer[9]);
  magnetometer[2] = (int16_t)(accel_mag_buffer[10] << BYTE_SHIFT_AMOUNT |
                              accel_mag_buffer[11]);

  magnetometer[0] *= MAG_UT_LSB;
  magnetometer[1] *= MAG_UT_LSB;
  magnetometer[2] *= MAG_UT_LSB;

  // Read gyroscope data
  // uint8_t gryo_buffer[DATA_SIZE];
  uint8_t gyro_buffer[GYRO_DATA_SIZE];
  i2c_smbus_write_byte(i2c_fd, GYRO_REG_START);
  i2c_smbus_read_i2c_block_data(i2c_fd, GYRO_REG_START, GYRO_DATA_SIZE,
                                gyro_buffer);
  gyroscope[0] =
      (int16_t)(gyro_buffer[0] << BYTE_SHIFT_AMOUNT | gyro_buffer[1]);
  gyroscope[1] =
      (int16_t)(gyro_buffer[2] << BYTE_SHIFT_AMOUNT | gyro_buffer[3]);
  gyroscope[2] =
      (int16_t)(gyro_buffer[4] << BYTE_SHIFT_AMOUNT | gyro_buffer[5]);

  gyroscope[0] *= GYRO_SENSITIVITY_250DPS;
  gyroscope[1] *= GYRO_SENSITIVITY_250DPS;
  gyroscope[2] *= GYRO_SENSITIVITY_250DPS;
}

/**
 * @brief   Print accelerometer data to console.
 */
void Imu::printAccel() {
  printf("Acceleration (m/s^2): (%0.3f, %0.3f, %0.3f)\n", accelerometer[0],
         accelerometer[1], accelerometer[2]);
}

/**
 * @brief   Print magnetometer data to console.
 */
void Imu::printMag() {
  printf("Magnetometer (uTesla): (%0.3f, %0.3f, %0.3f)\n", magnetometer[0],
         magnetometer[1], magnetometer[2]);
}

/**
 * @brief   Print gyroscope data to console.
 */
void Imu::printGyro() {
  printf("Gyroscope (radians/s): (%0.3f, %0.3f, %0.3f)\n", gyroscope[0],
         gyroscope[1], gyroscope[2]);
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
