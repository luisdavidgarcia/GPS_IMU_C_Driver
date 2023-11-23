#include "../../imu_module/imu.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <thread>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

int main(void) {
  Imu imu_module;
  imu_module.Telementary(1000);
  // imu_readSensorData();
  // const int16_t *accel_data = imu_module.getAccelerometerData();
  // const int16_t *gyro_data = imu_module.getGyroscopeData();
  // const int16_t *mag_data = imu_module.getMagnetometerData();

  return 0;
}


