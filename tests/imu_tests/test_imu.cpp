#include "../../imu_module/imu.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

/*
int main(void) {
  Imu imu_module;

  imu_module.Telementary(1000);

  return EXIT_SUCCESS;
}
#include <i2c/smbus.h>
#include <stdio.h>
*/

int main() {

  int i2c_file;
  i2c_file = open("/dev/i2c-1", O_RDWR);

  // ICM-20948 I2C address
  int addr = 0x69;

  ioctl(i2c_file, I2C_SLAVE, addr);

  // Select user bank 0
  uint8_t bank_sel = 0x7F;
  uint8_t bank_val = 0x02;
  i2c_smbus_write_byte_data(i2c_file, bank_sel, bank_val);

  // Configure gyroscope full scale range to +/- 2000 dps
  uint8_t gyro_config = 0x11 << 1;
  i2c_smbus_write_byte_data(i2c_file, 0x01, gyro_config);

  // Set gyroscope ODR to 1125 Hz
  // uint8_t gyro_smplrt_div = 0x01;
  // i2c_smbus_write_byte_data(i2c_file, 0x00, gyro_smplrt_div);

  // Enable gyroscope measurment
  uint8_t pwr_mgmt_2 = 0x000;
  i2c_smbus_write_byte_data(i2c_file, 0x07, pwr_mgmt_2);

  while (1) {
    // Read gyroscope data
    uint8_t gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
    gyro_x_h = i2c_smbus_read_byte_data(i2c_file, 0x33);
    gyro_x_l = i2c_smbus_read_byte_data(i2c_file, 0x34);
    gyro_y_h = i2c_smbus_read_byte_data(i2c_file, 0x35);
    gyro_y_l = i2c_smbus_read_byte_data(i2c_file, 0x36);
    gyro_z_h = i2c_smbus_read_byte_data(i2c_file, 0x37);
    gyro_z_l = i2c_smbus_read_byte_data(i2c_file, 0x38);

    int16_t gyro_x = (gyro_x_h << 8) | gyro_x_l;
    int16_t gyro_y = (gyro_y_h << 8) | gyro_y_l;
    int16_t gyro_z = (gyro_z_h << 8) | gyro_z_l;

    printf("Gyro X: %d\n", gyro_x);
    printf("Gyro Y: %d\n", gyro_y);
    printf("Gyro Z: %d\n", gyro_z);

    sleep(1);
  }

  close(i2c_file);

  return 0;
}
