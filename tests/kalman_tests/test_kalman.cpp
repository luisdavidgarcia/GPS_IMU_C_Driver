#include "gps.h"
#include "imu.h"
#include "ekfNavINS.h"
#include <fstream> 
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>

#define CURRENT_YEAR 2024

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

// Add variables to store previous filtered values
float prevFilteredAx = 0, prevFilteredAy = 0, prevFilteredAz = 0;
float prevFilteredMx = 0, prevFilteredMy = 0, prevFilteredMz = 0;

int main(void) {
    // Register the signal handler for SIGINT (Ctrl+C)
    signal(SIGINT, signal_handler);

    Imu imu_module;
    ekfNavINS ekf;
    float pitch, roll, yaw;
    float Gxyz[3], Axyz[3], Mxyz[3];
    const float alpha = 0.5; // Adjust this parameter to tweak the filter (range: 0-1)

    while(!exit_flag) {
        // PVTData gps_data = gps_module.GetPvt(true, 1);
        // All data for IMU is normalized already for 250dps, 2g, and 4 gauss
        imu_module.ReadSensorData();
        // if (gps_data.year == CURRENT_YEAR && gps_data.numberOfSatellites > 0) {
        const int16_t *accel_data = imu_module.GetAccelerometerData();
        if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
            printf("Accelerometer data is invalid.\n");
            continue;
        }

        const int16_t *gyro_data = imu_module.GetGyroscopeData();
        if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
            printf("Gyroscope data is invalid.\n");
            continue;
        }

        const int16_t *mag_data = imu_module.GetMagnetometerData();
        if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
            printf("Magnetometer data is invalid.\n");
            continue;
        }

        Gxyz[0] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[0]));// - G_offset[0]);
        Gxyz[1] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[1]));// - G_offset[1]);
        Gxyz[2] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[2]));// - G_offset[2]);
        Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Axyz[1] = static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
        Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
        Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

        // Low-pass filter for accelerometer data
        float filteredAx = alpha * prevFilteredAx + (1 - alpha) * Axyz[0];
        float filteredAy = alpha * prevFilteredAy + (1 - alpha) * Axyz[1];
        float filteredAz = alpha * prevFilteredAz + (1 - alpha) * Axyz[2];

        // Low-pass filter for magnetometer data
        float filteredMx = alpha * prevFilteredMx + (1 - alpha) * Mxyz[0];
        float filteredMy = alpha * prevFilteredMy + (1 - alpha) * Mxyz[1];
        float filteredMz = alpha * prevFilteredMz + (1 - alpha) * Mxyz[2];

        // Update previous values for next iteration
        prevFilteredAx = filteredAx;
        prevFilteredAy = filteredAy;
        prevFilteredAz = filteredAz;
        prevFilteredMx = filteredMx;
        prevFilteredMy = filteredMy;
        prevFilteredMz = filteredMz;

        std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(filteredAx, filteredAy, filteredAz, filteredMx, filteredMy, filteredMz);
        printf("Roll 	  : %2.3f\n", ekf.getRoll_rad());
        printf("Pitch     : %2.3f\n", ekf.getPitch_rad());
        printf("Yaw       : %2.3f\n", ekf.getHeading_rad());

        // Write only the IMU data to a file
        std::ofstream outfile("tests/kalman_tests/rpy_data.txt");
        if (outfile.is_open()) {
            outfile << ekf.getRoll_rad() << "," << ekf.getPitch_rad() << "," << ekf.getHeading_rad() << std::endl;
            outfile.flush(); // Flush the stream
            outfile.close(); // Close the file to save the changes
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }

        printf("\n---------------------\n");
        sleep(0.5);
    }

    return 0;
}
