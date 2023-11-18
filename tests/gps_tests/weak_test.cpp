extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

int main(void) {
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
		//uint8_t reg = 0x10; /* Device register to access */
		__s32 res;
		char buf[10];
		//uint8_t temp = 0x10;

		/* Using SMBus commands */
		res = i2c_smbus_read_word_data(file, reg);
		if (res < 0) {
		  /* ERROR HANDLING: i2c transaction failed */
			printf("Failed i2c transcation!\n");
		} else {
		  /* res contains the read word */
		  std::cout << "Result 1: " << res << std::endl;
		}

		/*
		 * Using I2C Write, equivalent of
		 * i2c_smbus_write_word_data(file, reg, 0x6543)
		 */
		buf[0] = reg;
		buf[1] = 0x43;
		buf[2] = 0x65;
		
		printf("Current buf[0]: 0x%x\n", buf[0]);

		if (write(file, buf, 3) != 3) {
		  /* ERROR HANDLING: i2c transaction failed */
			std::cout << "Failed to write in I2C transaction" << std::endl;
		}

		/* Using I2C Read, equivalent of i2c_smbus_read_byte(file) */
		if (read(file, buf, 1) != 1) {
		  /* ERROR HANDLING: i2c transaction failed */
		  std::cout << "Failed to read in I2C transaction" << std::endl;
		} else {
		  /* buf[0] contains the read byte */
			printf("Current buf[1]: 0x%x\n", buf[1]);
		}

	return 0;
}

