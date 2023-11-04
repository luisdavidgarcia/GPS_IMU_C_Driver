#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include "../../imu_module/imu.h"

extern "C" {
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
}

int main(void) {
    printf("Hi there!\n");

    return EXIT_SUCCESS;
}
