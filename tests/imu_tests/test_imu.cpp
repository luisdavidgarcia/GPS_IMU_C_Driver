#include "../../imu_module/imu.h"
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

int main(void) {
  Imu imu_module; 
  imu_module.Telementary(1000);

  return 0;
}


