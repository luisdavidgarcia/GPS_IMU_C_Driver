// #include "../../imu_module/imu.h"
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
  // Set I2C bus
  int i2c_file;
  i2c_file = open("/dev/i2c-1", O_RDWR);

  // Set  ICM-20948 I2C address
  int addr = 0x69;
  if (ioctl(i2c_file, I2C_SLAVE, addr) < 0) {
    printf("Failed to Set Bus\n");
    return -1;
  }

  // Check ID
  if (i2c_smbus_read_byte_data(i2c_file, 0x00) != 0xEA) {
    printf("Failed to Identify Chip\n");
    return -1;
  }

  // Software Reset
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); // set bank
  i2c_smbus_write_byte_data(i2c_file, 0x06, 0x81);

  // Sleep Mode
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x06, 0x01);

  // Turn off Low Power
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x06, 0x01);

  // Set Scalling
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x05, 0x40);

  // Set accelerometer full scale to +/-2g
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x14, 0x01);

  // Set gyroscope full scale to +/-250dps
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x01, 0x01);

  // set low pass filter for both accel and gyro (separate functions)
  //i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  //i2c_smbus_write_byte_data(i2c_file, 0x14, 0x39);
  //i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  //i2c_smbus_write_byte_data(i2c_file, 0x01, 0x39); 
  
  // disable digital low pass filters on both accel and gyro
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x14, 0x38);
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x20); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x01, 0x38); 

  // Start Magnometer
  // Master Pass Through set to false
  //i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  //i2c_smbus_write_byte_data(i2c_file, 0x0F, 0x00); 

  /*
  // Enable Master
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x01, 0x17);
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x03, 0x20); 

  // Transact directly with an I2C device, one byte at a time
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x13, 0x0C); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x14, 0x31); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x16, 0x08); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x15, 0x80); 

  // Set up Slaves with Master  
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x30); //set bank
  i2c_smbus_write_byte_data(i2c_file, 0x03, 0x8C); 
  i2c_smbus_write_byte_data(i2c_file, 0x04, 0x10); 
  i2c_smbus_write_byte_data(i2c_file, 0x05, 0x89); 
  i2c_smbus_write_byte_data(i2c_file, 0x7F, 0x00); //set bank
  */


  while (1) {
    // Read gyroscope data
    uint8_t gyro_x_h, gyro_x_l, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;
    gyro_x_h = i2c_smbus_read_byte_data(i2c_file, 0x33);
    gyro_x_l = i2c_smbus_read_byte_data(i2c_file, 0x34);
    gyro_y_h = i2c_smbus_read_byte_data(i2c_file, 0x35);
    gyro_y_l = i2c_smbus_read_byte_data(i2c_file, 0x36);
    gyro_z_h = i2c_smbus_read_byte_data(i2c_file, 0x37);
    gyro_z_l = i2c_smbus_read_byte_data(i2c_file, 0x38);

    int16_t gyro_x = (gyro_x_h << 8) | (gyro_x_l & 0xFF);
    int16_t gyro_y = (gyro_y_h << 8) | (gyro_y_l & 0xFF);
    int16_t gyro_z = (gyro_z_h << 8) | (gyro_z_l & 0xFF);

    uint8_t accel_x_h, accel_x_l, accel_y_h, accel_y_l, accel_z_h, accel_z_l;
    accel_x_h = i2c_smbus_read_byte_data(i2c_file, 0x2D);
    accel_x_l = i2c_smbus_read_byte_data(i2c_file, 0x2E);
    accel_y_h = i2c_smbus_read_byte_data(i2c_file, 0x2F);
    accel_y_l = i2c_smbus_read_byte_data(i2c_file, 0x30);
    accel_z_h = i2c_smbus_read_byte_data(i2c_file, 0x31);
    accel_z_l = i2c_smbus_read_byte_data(i2c_file, 0x32);

    int16_t accel_x = (accel_x_h << 8) | (accel_x_l & 0xFF);
    int16_t accel_y = (accel_y_h << 8) | (accel_y_l & 0xFF);
    int16_t accel_z = (accel_z_h << 8) | (accel_z_l & 0xFF);

    printf("Gyro X: %d Gyro Y: %d Gyro Z: %d\n", gyro_x, gyro_y, gyro_z);
    printf("Accel X: %d Accel Y: %d Accel Z: %d\n", accel_x, accel_y, accel_z);

    sleep(1);
  }

  close(i2c_file);

  return 0;
}
