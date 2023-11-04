#include "../../imu_module/imu.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

int main(void) {
  Imu imu_module;

  imu_module.Telementary(1000);

  return EXIT_SUCCESS;
}
