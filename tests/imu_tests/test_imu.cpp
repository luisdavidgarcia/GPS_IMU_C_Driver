#include "imu.h"
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
#include <iostream>

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
    imu_module.ReadSensorData();
    printf("--------------------\n");

    const int16_t *accel_data = imu_module.GetRawAccelerometerData();
    if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
      printf("Accelerometer data is invalid.\n");
      continue;
    } else {
      printf("Acceleration (m/s^2): (X: %d, Y: %d, Z: %d)\n", accel_data[0], accel_data[1], accel_data[2]);
    }

    const int16_t *gyro_data = imu_module.GetRawGyroscopeData();
    if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
      printf("Gyroscope data is invalid.\n");
      continue;
    } else {
      printf("Gyroscope (radians/s): (X: %d, Y: %d, Z: %d)\n", gyro_data[0], gyro_data[1], gyro_data[2]);
    }

    const int16_t *mag_data = imu_module.GetRawMagnetometerData();
    if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
      printf("Magnetometer data is invalid.\n");
      continue;
    } else {
      printf("Magnetometer (uTesla): (X: %d, Y: %d, Z: %d)\n", mag_data[0], mag_data[1], mag_data[2]);
    }

    printf("--------------------\n");
    sleep(1);
  }
    // Perform any necessary cleanup before exiting
    std::cout << "Exiting program." << std::endl;

    // Exit the program
    std::exit(0);
}


