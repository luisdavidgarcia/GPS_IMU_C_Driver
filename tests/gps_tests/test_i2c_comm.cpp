#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>


int main() {
    const char *deviceName = "/dev/i2c-1";  // Use the correct I2C bus device
    int i2cFile = open(deviceName, O_RDWR);
    if (i2cFile < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return 1;
    }
    int slaveAddress = 0x42;  // Use the correct I2C device address
    if (ioctl(i2cFile, I2C_SLAVE, slaveAddress) < 0) {
        std::cerr << "Unable to set I2C slave address" << std::endl;
        close(i2cFile);
        return 1;
    }
    // Define the message to send
    uint8_t message[] = "Hello, I2C Device!";
    ssize_t bytesWritten = write(i2cFile, message, sizeof(message));
    if (bytesWritten != sizeof(message)) {
        std::cerr << "Write error" << std::endl;
        close(i2cFile);
        return 1;
    }
    // Read the response from the I2C device
    uint8_t response[32];  // Allocate a buffer for the response
    ssize_t bytesRead = read(i2cFile, response, sizeof(response));
    //response[bytesRead] = '\0';

	std::cout << "Response contents: ";
	for(int i = 0; i < bytesRead; ++i) {
	    std::cout << std::hex << static_cast<int>(response[i]) << " ";
	}
	std::cout << std::endl;


    if (bytesRead < 0) {
        std::cerr << "Read error" << std::endl;
    } else {
        std::cout << "Received response: " << std::string(reinterpret_cast<char*>(response), bytesRead) << std::endl;
    }
    close(i2cFile);
    return 0;
}
