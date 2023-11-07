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

#define TIME_DELAY_MS 1000
#define ACCEL_REG_START 0x00
#define MAGNETO_REG_START 0x00
#define ACCEL_MAG_DATA_SIZE 12
#define SENSORS_GRAVITY_STD 9.81F
#define BYTE_SHIFT_AMOUNT 8
#define BANK_VALUE 0x02
#define PWR_MGMT_2_VALUE 0x000000

#define SLAVE_ADDRESS 0x69

/** General Registers */
#define BANK_SEL 0x7F
#define PWR_MGMT_2 0x07

/** Gyroscope Registers */
#define GYRO_REG_START 0x00
#define GYRO_CONFIG 0x01
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

/** Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS (0.0078125F) // Table 35 of datasheet
#define GYRO_DATA_SIZE 6
#define GYRO_CONFIG_VALUE 0x11 << 1

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

class Imu {
private:
  int i2c_fd;
  float accelerometer[3];
  float magnetometer[3];
  float gyroscope[3];
  void begin(void);
  void readSensorData(void);

public:
  Imu();
  ~Imu();
  void printAccel();
  void printMag();
  void printGyro();
  void Telementary(int delay);
};

#endif // IMU_H
