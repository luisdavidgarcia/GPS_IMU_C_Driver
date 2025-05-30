#include "IMU/IMU.hpp"

/**
 * @brief   Constructor for the IMU class.
 *
 * Initializes the I2C communication with the IMU sensor, sets its address, and 
 * performs an initial identification check.
 */
IMU::IMU() : accelerometer{}, gyroscope{}, magnetometer{}
{
	const char *deviceName = IMU_I2C_BUS;
	i2c_fd_ = open(deviceName, O_RDWR | O_CLOEXEC);
	if (i2c_fd_ < 0) {
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
 * @brief   Destructor for the IMU class.
 *
 * Closes the I2C device when the IMU object is destroyed.
 */
IMU::~IMU() 
{
	printf("About to close fd\n");
	close(i2c_fd);
}

/**
 * @brief   Initialize the IMU sensor configuration.
 *
 * Sets up the IMU sensor by configuring registers and settings
 * for accelerometer, gyroscope, and magnetometer communication.
 */
void IMU::begin() const {
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
 * 					sensor data. For example: accelerometer[0] = <read accelerometer X value>;
 *          accelerometer[1] = <read accelerometer Y value>;
 *          accelerometer[2] = <read accelerometer Z value>;
 *          gyroscope[0] = <read gyroscope X value>;
 *          gyroscope[1] = <read gyroscope Y value>;
 *          gyroscope[2] = <read gyroscope Z value>;
 *          magnetometer[0] = <read magnetometer X value>;
 *          magnetometer[1] = <read magnetometer Y value>;
 *          magnetometer[2] = <read magnetometer Z value>;
 */
void IMU::Read() {
	/* Reset bank to 0 on every read */
	i2c_smbus_write_byte_data(i2c_fd, BANK_SEL, BANK_REG_0); //set bank

	/* Read accelerometer data */
	uint8_t accel_x_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_XOUT_H);
	uint8_t accel_x_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_XOUT_L);
	uint8_t accel_y_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_YOUT_H);
	uint8_t accel_y_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_YOUT_L);
	uint8_t accel_z_h = i2c_smbus_read_byte_data(i2c_fd, ACCEL_ZOUT_H);
	uint8_t accel_z_l = i2c_smbus_read_byte_data(i2c_fd, ACCEL_ZOUT_L);

	// Converting Raw Accel Data to Readable data
	accelerometer.x = static_cast<int16_t>(
		(accel_x_h << BITS_PER_BYTE) | (accel_x_l & BYTE_MASK)
	);
	accelerometer.y = static_cast<int16_t>(
		(accel_y_h << BITS_PER_BYTE) | (accel_y_l & BYTE_MASK)
	);
	accelerometer.z = static_cast<int16_t>(
		(accel_z_h << BITS_PER_BYTE) | (accel_z_l & BYTE_MASK)
	);

	/* Read gyroscope data */
	uint8_t gyro_x_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_XOUT_H);
	uint8_t gyro_x_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_XOUT_L);
	uint8_t gyro_y_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_YOUT_H);
	uint8_t gyro_y_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_YOUT_L);
	uint8_t gyro_z_h = i2c_smbus_read_byte_data(i2c_fd, GYRO_ZOUT_H);
	uint8_t gyro_z_l = i2c_smbus_read_byte_data(i2c_fd, GYRO_ZOUT_L);

	// Converting Raw Gyro Data to Readable data
	gyroscope.x = static_cast<int16_t>(
		(gyro_x_h << BITS_PER_BYTE) | (gyro_x_l & BYTE_MASK)
	);
	gyroscope.y = static_cast<int16_t>(
		(gyro_y_h << BITS_PER_BYTE) | (gyro_y_l & BYTE_MASK)
	);
	gyroscope.z = static_cast<int16_t>(
		(gyro_z_h << BITS_PER_BYTE) | (gyro_z_l & BYTE_MASK)
	);

	/* Read magentometer data */
	uint8_t mag_x_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_XOUT_H);
	uint8_t mag_x_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_XOUT_L);
	uint8_t mag_y_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_YOUT_H);
	uint8_t mag_y_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_YOUT_L);
	uint8_t mag_z_h = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_ZOUT_H);
	uint8_t mag_z_l = i2c_smbus_read_byte_data(i2c_fd, MAGNETO_ZOUT_L);

	// Converting Raw Mag Data to Readable data
	magnetometer.x = static_cast<int16_t>(
		(mag_x_l << BITS_PER_BYTE) | (mag_x_h & BYTE_MASK)
	);
	magnetometer.y = static_cast<int16_t>(
		(mag_y_l << BITS_PER_BYTE) | (mag_y_h & BYTE_MASK)
	);
	magnetometer.z = static_cast<int16_t>(
		(mag_z_l << BITS_PER_BYTE) | (mag_z_h & BYTE_MASK)
	);
}
