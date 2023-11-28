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
#include <csignal>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

// Define a flag to indicate if the program should exit gracefully.
volatile bool exit_flag = false;

// Signal handler function for Ctrl+C (SIGINT)
void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "Ctrl+C received. Cleaning up..." << std::endl;

        // Set the exit flag to true to trigger graceful exit.
        exit_flag = true;
    }
}

int main(void) {
  // Register the signal handler for SIGINT (Ctrl+C)
  signal(SIGINT, signal_handler);

  Imu imu_module;
  while (!exit_flag) {
    imu_module.readSensorData();
    // const int16_t *accel_data = imu_module.getAccelerometerData();
    // const int16_t *gyro_data = imu_module.getGyroscopeData();
    // const int16_t *mag_data = imu_module.getMagnetometerData();
    imu_module.printAccel();
    imu_module.printGyro();
    imu_module.printMag();
    printf("\n");
    sleep(1);
  }
    // Perform any necessary cleanup before exiting
    std::cout << "Exiting program." << std::endl;

    // Exit the program
    std::exit(0);
}


