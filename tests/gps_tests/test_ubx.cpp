#include "imu.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>

extern "C" {
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
}

int main(void) {
    std::vector<uint8_t> payload;
    // Populate the vector with 20 hex values of 0x00
    for (int i = 0; i < 20; i++) {
        payload.push_back(0x00);
    }
    std::vector<uint8_t> ubx_message = composeMessage(CFG_CLASS, CFG_PRT, 20, payload);
//   std::string temp = ping("Hello World");
//   std::cout << temp << std::endl;
    int file;
    int adapter_nr = 1; /* probably dynamically determined */
    char filename[20];

    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0) {
      /* ERROR HANDLING; you can check errno to see what went wrong */
      exit(1);
    }

    int addr = 0x42; /* The I2C address */

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
      /* ERROR HANDLING; you can check errno to see what went wrong */
      exit(1);
    }

    __u8 reg = 0x10; /* Device register to access */
    __s32 res;
    char buf[ubx_message.size()];
            /*
             * Using I2C Write, equivalent of
             * i2c_smbus_write_word_data(file, reg, 0x6543)
             */
    for (int i = 0; i < ubx_message.size(); i++) {
      buf[i] = ubx_message[i];	
      printf("Before Write buf[%d]: 0x%x\n", i, buf[i]);
    }

    for (int i = 0; i < ubx_message.size(); i++) {
      res = i2c_smbus_write_byte(file, buf[i]);
        if (res < 0) {
          /* ERROR HANDLING: i2c transaction failed */
            printf("Failed i2c transcation!\n");
        } 
    }

            /* Using I2C Read, equivalent of i2c_smbus_read_byte(file) */
    for (int i = 0; i < ubx_message.size(); i++) {
        res = i2c_smbus_read_byte(file);
        if (res < 0) {
                std::cout << "Failed to read in I2C transaction" << std::endl;
        } else {
                      printf("Read 0x%x from register 0x%x\n", res, reg);
       }
    }
return 0;
}
