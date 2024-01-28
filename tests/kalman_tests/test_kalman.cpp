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
// Might help: https://teslabs.com/articles/magnetometer-calibration/

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

    // Gps gps_module;
    Imu imu_module;
    ekfNavINS ekf;
    float pitch, roll, yaw;
    float Gxyz[3], Axyz[3], Mxyz[3];

    while(!exit_flag) {
        // PVTData gps_data = gps_module.GetPvt(true, 1);
        // All data for IMU is normalized already for 250dps, 2g, and 4 gauss
        imu_module.readSensorData();
        // if (gps_data.year == CURRENT_YEAR && gps_data.numberOfSatellites > 0) {
            const int16_t *accel_data = imu_module.getAccelerometerData();
            if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
                printf("Accelerometer data is invalid.\n");
                continue;
            }

            const int16_t *gyro_data = imu_module.getGyroscopeData();
            if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
                printf("Gyroscope data is invalid.\n");
                continue;
            }

            const int16_t *mag_data = imu_module.getMagnetometerData();
            if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
                printf("Magnetometer data is invalid.\n");
                continue;
            }

            Gxyz[0] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[0]) - G_offset[0]);
            Gxyz[1] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[1]) - G_offset[1]);
            Gxyz[2] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[2]) - G_offset[2]);
            Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
            Axyz[1] = static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
            Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
            Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
            Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
            Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

            // printf("Filtered Gyroscope (rad/s): (X: %f, Y: %f, Z: %f)\n", filteredGxyz[0], filteredGxyz[1], filteredGxyz[2]);
            // printf("Filtered Acceleration (m/s^2): (X: %f, Y: %f, Z: %f)\n", filteredAxyz[0], filteredAxyz[1], filteredAxyz[2]);
            // printf("Filtered Magnetometer (uTesla): (X: %f, Y: %f, Z: %f)\n", filteredMxyz[0], filteredMxyz[1], filteredMxyz[2]);

            printf("Gyroscope (rad/s): (X: %f, Y: %f, Z: %f)\n", Gxyz[0], Gxyz[1], Gxyz[2]);
            printf("Acceleration (m/s^2): (X: %f, Y: %f, Z: %f)\n", Axyz[0], Axyz[1], Axyz[2]);
            printf("Magnetometer (uTesla): (X: %f, Y: %f, Z: %f)\n", Mxyz[0], Mxyz[1], Mxyz[2]);

            std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(Axyz[0], Axyz[1], Axyz[2], Mxyz[0], Mxyz[1], Mxyz[2]);
            // std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(filteredAxyz[0], filteredAxyz[1], filteredAxyz[2], filteredMxyz[0], filteredMxyz[1], filteredMxyz[2]);

            // ekf.ekf_update(time(NULL) /*,gps.getTimeOfWeek()*/, gps_data.velocityNorth*1e-3, gps_data.velocityEast*1e-3,
            //               gps_data.velocityDown*1e-3, gps_data.latitude*DEG_TO_RAD,
            //               gps_data.longitude*DEG_TO_RAD, (gps_data.height*1e-3),
            //               gx, gy, gz,
            //               ax, ay, az, hx, hy, hz);

            // printf("Latitude  : %2.7f %2.7f\n", gps_data.latitude, ekf.getLatitude_rad()*RAD_TO_DEG);
            // printf("Longitude : %2.7f %2.7f\n", gps_data.longitude, ekf.getLongitude_rad()*RAD_TO_DEG);
            // printf("Altitude  : %2.3f %2.3f\n", gps_data.height*1e-3, ekf.getAltitude_m());
            // printf("Speed (N) : %2.3f %2.3f\n", gps_data.velocityNorth*1e-3, ekf.getVelNorth_ms());
            // printf("Speed (E) : %2.3f %2.3f\n", gps_data.velocityEast*1e-3, ekf.getVelEast_ms());
            // printf("Speed (D) : %2.3f %2.3f\n", gps_data.velocityDown*1e-3, ekf.getVelDown_ms());
            printf("Roll 	  : %2.3f\n", ekf.getRoll_rad());
            printf("Pitch     : %2.3f\n", ekf.getPitch_rad());
            printf("Yaw       : %2.3f\n", ekf.getHeading_rad());

            // Write all the data to a file - GPS & IMU
            // std::ofstream outfile("tests/kalman_tests/data.txt", std::ios_base::app); // Open in append mode
            // if (outfile.is_open()) {
            //     outfile << gps_data.latitude << "," << ekf.getLatitude_rad() * RAD_TO_DEG << ","
            //             << gps_data.longitude << "," << ekf.getLongitude_rad() * RAD_TO_DEG << ","
            //             << gps_data.height * 1e-3 << "," << ekf.getAltitude_m() << ","
            //             << gps_data.velocityNorth * 1e-3 << "," << ekf.getVelNorth_ms() << ","
            //             << gps_data.velocityEast * 1e-3 << "," << ekf.getVelEast_ms() << ","
            //             << gps_data.velocityDown * 1e-3 << "," << ekf.getVelDown_ms() << ","
            //             << ekf.getRoll_rad() << "," << ekf.getPitch_rad() << "," << ekf.getHeading_rad()
            //             << std::endl;
            //     outfile.close();
            // } else {
            //     std::cerr << "Unable to open file for writing." << std::endl;
            // }

            // Write only the IMU data to a file
            std::ofstream outfile("tests/kalman_tests/data.txt");
            if (outfile.is_open()) {
                outfile << ekf.getRoll_rad() << "," << ekf.getPitch_rad() << "," << ekf.getHeading_rad() << std::endl;
                outfile.flush(); // Flush the stream
                outfile.close(); // Close the file to save the changes
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

            printf("\n---------------------\n");
    //    } else {
    //        printf("No GPS data\n");
    //    }
        sleep(1);
    }

    return 0;
}
