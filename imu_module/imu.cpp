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

  // Check ID
  if (i2c_smbus_read_byte_data(i2c_fd, 0x00) != 0xEA) {
    perror("Failed to identify chip");
  }
}

Imu::~Imu() { close(i2c_fd); }

void Imu::begin() {
  // TODO: Create error_handler for register writes
  
  // Select Clock to Automatic (Init Accel and Gyro)
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x06, 0x01);

  /* Init Magnometer */
  // Master Pass Through set to false (For Magnometer)
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x0F, 0x00); 

  // Enable Master (For Magnometer)
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x01, 0x17);
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x03, 0x20); 

  // Transact directly with an I2C device, one byte at a time (For Magnometer)
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x13, 0x0C); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x14, 0x31); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x16, 0x08); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x15, 0x80); 

  // Set up Slaves with Master (For Magnometer)
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x03, 0x8C); 
  i2c_smbus_write_byte_data(i2c_file, 0x04, 0x10); 
  i2c_smbus_write_byte_data(i2c_file, 0x05, 0x89); 

  /* Reset Bank to Zero 0 For Reading Data */
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
}

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
    /* Reset bank to 0 on every read */
    i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank

    /* Read accelerometer data */
    uint8_t accel_x_h, accel_x_l, accel_y_h, accel_y_l, accel_z_h, accel_z_l;
    accel_x_h = i2c_smbus_read_byte_data(i2c_file, 0x2D);
    accel_x_l = i2c_smbus_read_byte_data(i2c_file, 0x2E);
    accel_y_h = i2c_smbus_read_byte_data(i2c_file, 0x2F);
    accel_y_l = i2c_smbus_read_byte_data(i2c_file, 0x30);
    accel_z_h = i2c_smbus_read_byte_data(i2c_file, 0x31);
    accel_z_l = i2c_smbus_read_byte_data(i2c_file, 0x32);

    // Converting Raw Accel Data to Readable data 
    accelerometer[0] = (accel_x_h << 8) | (accel_x_l & 0xFF);
    accelerometer[1] = (accel_y_h << 8) | (accel_y_l & 0xFF);
    accelerometer[2] = (accel_z_h << 8) | (accel_z_l & 0xFF);

    /* Read gyroscope data */
    uint8_t gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
    gyro_x_h = i2c_smbus_read_byte_data(i2c_file, 0x33);
    gyro_x_l = i2c_smbus_read_byte_data(i2c_file, 0x34);
    gyro_y_h = i2c_smbus_read_byte_data(i2c_file, 0x35);
    gyro_y_l = i2c_smbus_read_byte_data(i2c_file, 0x36);
    gyro_z_h = i2c_smbus_read_byte_data(i2c_file, 0x37);
    gyro_z_l = i2c_smbus_read_byte_data(i2c_file, 0x38);

    // Converting Raw Gyro Data to Readable data 
    gyroscope[0] = (gyro_x_h << 8) | (gyro_x_l & 0xFF);
    gyroscope[1] = (gyro_y_h << 8) | (gyro_y_l & 0xFF);
    gyroscope[2] = (gyro_z_h << 8) | (gyro_z_l & 0xFF);

    /* Read magentometer data */
    uint8_t mag_x_h, mag_x_l, mag_y_h, mag_y_l, mag_z_h, mag_z_l;
    mag_x_h = i2c_smbus_read_byte_data(i2c_file, 0x3C);
    mag_x_l = i2c_smbus_read_byte_data(i2c_file, 0x3D);
    mag_y_h = i2c_smbus_read_byte_data(i2c_file, 0x3E);
    mag_y_l = i2c_smbus_read_byte_data(i2c_file, 0x3F);
    mag_z_h = i2c_smbus_read_byte_data(i2c_file, 0x40);
    mag_z_l = i2c_smbus_read_byte_data(i2c_file, 0x41);

    // Converting Raw Mag Data to Readable data 
    magnetometer[0] = (mag_x_l << 8) | (mag_x_h & 0xFF);
    magnetometer[1] = (mag_y_l << 8) | (mag_y_h & 0xFF);
    magnetometer[2] = (mag_z_l << 8) | (mag_z_h & 0xFF);
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
