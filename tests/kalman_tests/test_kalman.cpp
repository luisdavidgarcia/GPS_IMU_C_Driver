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

int main(void) {
    // Register the signal handler for SIGINT (Ctrl+C)
    signal(SIGINT, signal_handler);

    Imu imu_module;
    ekfNavINS ekf;
    float pitch, roll, yaw;
    float Gxyz[3], Axyz[3], Mxyz[3];
    float filteredAx = 0, filteredAy = 0, filteredAz = 0;
    float filteredMx = 0, filteredMy = 0, filteredMz = 0;
    const float alpha = 0.5; // Adjust this parameter to tweak the filter (range: 0-1)

    // Open a file in write mode to log data
    std::ofstream imuDataFile("tests/kalman_tests/accel_gyro_imu_data.csv");
    if (imuDataFile.is_open()) {
        // Write headers to the CSV file
        imuDataFile << "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ\n";
    } else {
        std::cerr << "Unable to open file for writing IMU data." << std::endl;
        return 1; // Exit if file cannot be opened
    }

    while(!exit_flag) {
        // All data for IMU is normalized already for 250dps, 2g, and 4 gauss
        imu_module.ReadSensorData();
        const int16_t *accel_data = imu_module.GetRawAccelerometerData();
        if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
            printf("Accelerometer data is invalid.\n");
            continue;
        }

        const int16_t *gyro_data = imu_module.GetRawGyroscopeData();
        if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
            printf("Gyroscope data is invalid.\n");
            continue;
        }

        const int16_t *mag_data = imu_module.GetRawMagnetometerData();
        if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
            printf("Magnetometer data is invalid.\n");
            continue;
        }

        printf("AccelX: %d, AccelY: %d, AccelZ: %d\n", accel_data[0], accel_data[1], accel_data[2]);
        printf("GyroX: %d, GyroY: %d, GyroZ: %d\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        printf("MagX: %d, MagY: %d, MagZ: %d\n", mag_data[0], mag_data[1], mag_data[2]);

        // Write the IMU data to the file
        if (imuDataFile.is_open()) {
            imuDataFile << Axyz[0] << "," << Axyz[1] << "," << Axyz[2] << ","
                        << Gxyz[0] << "," << Gxyz[1] << "," << Gxyz[2] << ","
                        << Mxyz[0] << "," << Mxyz[1] << "," << Mxyz[2] << "\n";
        }

        // Gxyz[0] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[0]));// - G_offset[0]);
        // Gxyz[1] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[1]));// - G_offset[1]);
        // Gxyz[2] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[2]));// - G_offset[2]);
        // Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G; //* SENSORS_GRAVITY_STD;
        // Axyz[1] = -1 * static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G; //* SENSORS_GRAVITY_STD;
        // Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G; //* SENSORS_GRAVITY_STD;
        // Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
        // Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
        // Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

        // // Low-pass filter for accelerometer data
        // filteredAx = alpha * filteredAx + (1 - alpha) * Axyz[0];
        // filteredAy = alpha * filteredAy + (1 - alpha) * Axyz[1];
        // filteredAz = alpha * filteredAz + (1 - alpha) * Axyz[2];

        // // Low-pass filter for magnetometer data
        // filteredMx = alpha * filteredMx + (1 - alpha) * Mxyz[0];
        // filteredMy = alpha * filteredMy + (1 - alpha) * Mxyz[1];
        // filteredMz = alpha * filteredMz + (1 - alpha) * Mxyz[2];

        // std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(filteredAx, filteredAy, filteredAz, filteredMx, filteredMy, filteredMz);
        // printf("Roll 	  : %2.3f\n", ekf.getRoll_rad());
        // printf("Pitch     : %2.3f\n", ekf.getPitch_rad());
        // printf("Yaw       : %2.3f\n", ekf.getHeading_rad());

        // // Write only the IMU data to a file
        // std::ofstream outfile("tests/kalman_tests/rpy_data.txt");
        // if (outfile.is_open()) {
        //     outfile << ekf.getRoll_rad() << "," << ekf.getPitch_rad() << "," << ekf.getHeading_rad() << std::endl;
        //     outfile.flush(); // Flush the stream
        //     outfile.close(); // Close the file to save the changes
        // } else {
        //     std::cerr << "Unable to open file for writing." << std::endl;
        // }

        // printf("\n---------------------\n");
        sleep(1);
    }

    imuDataFile.close(); // Close the file when done
    return 0;
}
