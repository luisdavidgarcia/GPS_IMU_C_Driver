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
#include <fstream>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

volatile bool exit_flag = false;

void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "Ctrl+C received. Cleaning up..." << std::endl;
        exit_flag = true;
    }
}

int main(void) {
  signal(SIGINT, signal_handler);

  Imu imu_module;
  std::ofstream mag_data_file("tests/calibration/raw_mag_data.csv");

  // Check if the file is open
  if (!mag_data_file.is_open()) {
    std::cerr << "Failed to open raw_mag_data.csv for writing." << std::endl;
    return 1;
  }

  // Write CSV header
  // mag_data_file << "MagX,MagY,MagZ\n";

  while (!exit_flag) {
    imu_module.readSensorData();

    const int16_t *mag_data = imu_module.getMagnetometerData();
    if (mag_data[0] != MAG_MAX_THRESHOLD && mag_data[1] != MAG_MAX_THRESHOLD && mag_data[2] != MAG_MAX_THRESHOLD) {
      mag_data_file << mag_data[0] << "," << mag_data[1] << "," << mag_data[2] << std::endl;
    }

    sleep(0.1);
  }

  mag_data_file.close();
  std::cout << "Exiting program and closing file." << std::endl;
  return 0;
}
