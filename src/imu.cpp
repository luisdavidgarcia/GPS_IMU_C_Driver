#include "../include/imu.h"

/**
 * @brief   Constructor for the Imu class.
 *
 * Initializes the I2C communication with the IMU sensor, sets its address, and performs an initial identification check.
 */
Imu::Imu() {
	const char *deviceName = IMU_I2C_BUS;
	i2c_fd = open(deviceName, O_RDWR);
	if (i2c_fd < 0) {
		perror("Unable to open I2C device");
	}

	if (ioctl(i2c_fd, I2C_SLAVE, IMU_I2C_ADDRESS) < 0) {
		perror("Failed to acquire bus access and/or talk to slave");
	}

	if (i2c_smbus_read_byte_data(i2c_fd, WHO_AM_I) != IMU_ID) {
		perror("Failed to identify chip");
	}

	begin();
}

/**
 * @brief   Destructor for the Imu class.
 *
 * Closes the I2C device when the Imu object is destroyed.
 */
Imu::~Imu() {
	printf("About to close fd\n");
	close(i2c_fd);
}

/**
 * @brief   Initialize the IMU sensor configuration.
 *
 * Sets up the IMU sensor by configuring registers and settings
 * for accelerometer, gyroscope, and magnetometer communication.
 */
void Imu::begin() {
	// Select Clock to Automatic (Init Accel and Gyro)
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_0); //set bank
	i2c_smbus_write_byte_data(i2c_fd, PWR_MGMT_1, 0x01);

	/* Init Magnometer */
	// Master Pass Through set to false (For Magnometer)
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_0); //set bank
	i2c_smbus_write_byte_data(i2c_fd, INT_PIN_CFG, 0x00);

	// Enable Master (For Magnometer)
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_MST_CTRL, 0x17);
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, 0x00); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV0_ADDR, 0x20);

	// Transact directly with an I2C device, one byte at a time (For Magnometer)
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV4_ADDR, 0x0C);
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV4_REG, 0x31);
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV4_DO, 0x08);
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV4_CTRL, 0x80);

	// Set up Slaves with Master (For Magnometer)
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_3); //set bank
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV0_ADDR, 0x8C);
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV0_REG, 0x10);
	i2c_smbus_write_byte_data(i2c_fd, I2C_SLV0_CTRL, 0x89);

	/* Reset Bank to Zero 0 For Reading Data */
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_0); //set bank
}

/**
 * @brief   Read sensor data from IMU over I2C.
 *          Replace this section with your actual I2C communication to read
 * 			sensor data. For example: accelerometer[0] = <read accelerometer X value>;
 *          accelerometer[1] = <read accelerometer Y value>;
 *          accelerometer[2] = <read accelerometer Z value>;
 *          gyroscope[0] = <read gyroscope X value>;
 *          gyroscope[1] = <read gyroscope Y value>;
 *          gyroscope[2] = <read gyroscope Z value>;
 *          magnetometer[0] = <read magnetometer X value>;
 *          magnetometer[1] = <read magnetometer Y value>;
 *          magnetometer[2] = <read magnetometer Z value>;
 */
void Imu::readSensorData(void) {
    /* Reset bank to 0 on every read */
    i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_0); //set bank

    /* Read accelerometer data */
    uint8_t accel_x_h, accel_x_l, accel_y_h, accel_y_l, accel_z_h, accel_z_l;
    accel_x_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_XOUT_H);
    accel_x_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_XOUT_L);
    accel_y_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_YOUT_H);
    accel_y_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_YOUT_L);
    accel_z_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_ZOUT_H);
    accel_z_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_ZOUT_L);

    // Converting Raw Accel Data to Readable data
    accelerometer[0] = (accel_x_h << BITS_PER_BYTE) | (accel_x_l & BYTE_MASK);
    accelerometer[1] = (accel_y_h << BITS_PER_BYTE) | (accel_y_l & BYTE_MASK);
    accelerometer[2] = (accel_z_h << BITS_PER_BYTE) | (accel_z_l & BYTE_MASK);

    /* Read gyroscope data */
    uint8_t gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
    gyro_x_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_XOUT_H);
    gyro_x_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_XOUT_L);
    gyro_y_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_YOUT_H);
    gyro_y_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_YOUT_L);
    gyro_z_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_ZOUT_H);
    gyro_z_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_ZOUT_L);

    // Converting Raw Gyro Data to Readable data
    gyroscope[0] = (gyro_x_h << BITS_PER_BYTE) | (gyro_x_l & BYTE_MASK);
    gyroscope[1] = (gyro_y_h << BITS_PER_BYTE) | (gyro_y_l & BYTE_MASK);
    gyroscope[2] = (gyro_z_h << BITS_PER_BYTE) | (gyro_z_l & BYTE_MASK);

    /* Read magentometer data */
	uint8_t mag_x_h, mag_x_l, mag_y_h, mag_y_l, mag_z_h, mag_z_l;
    mag_x_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_XOUT_H);
    mag_x_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_XOUT_L);
    mag_y_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_YOUT_H);
    mag_y_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_YOUT_L);
    mag_z_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_ZOUT_H);
    mag_z_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_ZOUT_L);

    // Converting Raw Mag Data to Readable data
    magnetometer[0] = (mag_x_l << BITS_PER_BYTE) | (mag_x_h & BYTE_MASK);
    magnetometer[1] = (mag_y_l << BITS_PER_BYTE) | (mag_y_h & BYTE_MASK);
    magnetometer[2] = (mag_z_l << BITS_PER_BYTE) | (mag_z_h & BYTE_MASK);
}

/**
 * @brief   Print accelerometer data to console.
 */
void Imu::printAccel(void) {
  	printf("Acceleration (m/s^2): (X: %d, Y: %d, Z: %d)\n", accelerometer[0],
        accelerometer[1], accelerometer[2]);
}

/**
 * @brief   Print magnetometer data to console.
 */
void Imu::printMag(void) {
  	printf("Magnetometer (uTesla): (X: %d, Y: %d, Z: %d)\n", magnetometer[0],
        magnetometer[1], magnetometer[2]);
}

/**
 * @brief   Print gyroscope data to console.
 */
void Imu::printGyro(void) {
  	printf("Gyroscope (radians/s): (X: %d, Y: %d, Z: %d)\n", gyroscope[0],
    	gyroscope[1], gyroscope[2]);
}

