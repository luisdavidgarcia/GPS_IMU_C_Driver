#include "gps.h"
#include "imu.h"
#include "ekfNavINS.h"
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <libserialport.h>

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

    Gps gps_module;
    Imu imu_module;
    ekfNavINS ekf;
    float ax, ay, az, gx, gy, gz, hx, hy, hz, pitch, roll, yaw;

    // Open and configure the serial port
    struct sp_port *port;
    sp_get_port_by_name("/dev/ttyAMA1", &port);
    sp_open(port, SP_MODE_READ_WRITE);
    sp_set_baudrate(port, 115200);  // Set baud rate

    while(!exit_flag) {
//        PVTData gps_data = gps_module.GetPvt(true, 1);
        // All data for IMU is normalized already for 250dps, 2g, and 4 gauss
        imu_module.readSensorData();
//        if (gps_data.year == CURRENT_YEAR && gps_data.numberOfSatellites > 0) {
            const int16_t *accel_data = imu_module.getAccelerometerData();
            if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
                printf("Accelerometer data is invalid.\n");
                continue;
            }
            else {
                printf("Acceleration (m/s^2): (X: %d, Y: %d, Z: %d)\n", accel_data[0], accel_data[1], accel_data[2]);
            }

            // Normalize acceleration values to g's
            ax = static_cast<float>(accel_data[0]);
            ay = -1 * static_cast<float>(accel_data[1]);
            az = static_cast<float>(accel_data[2]);

            const int16_t *gyro_data = imu_module.getGyroscopeData();
            if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
                printf("Gyroscope data is invalid.\n");
                continue;
            }
            else {
                printf("Gyroscope (radians/s): (X: %d, Y: %d, Z: %d)\n", gyro_data[0], gyro_data[1], gyro_data[2]);
            }

            gx = static_cast<float>(gyro_data[0]);
            gy = static_cast<float>(gyro_data[1]);
            gz = static_cast<float>(gyro_data[2]);

            const int16_t *mag_data = imu_module.getMagnetometerData();
            if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
                printf("Magnetometer data is invalid.\n");
                continue;
            }
            else {
                printf("Magnetometer (uTesla): (X: %d, Y: %d, Z: %d)\n", mag_data[0], mag_data[1], mag_data[2]);
            }

            hx = static_cast<float>(mag_data[0]);
            hy = static_cast<float>(mag_data[1]);
            hz = static_cast<float>(mag_data[2]);

            std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(ax, ay, az, hx, hy, hz);

            // Format the data to be sent
            std::stringstream data_stream;
            data_stream << ekf.getRoll_rad() << "," << ekf.getPitch_rad() << "," << ekf.getHeading_rad() << "\n";

            // Send the data over UART
            sp_blocking_write(port, data_stream.str().c_str(), data_stream.str().length(), 1000);

//            ekf.ekf_update(time(NULL) /*,gps.getTimeOfWeek()*/, gps_data.velocityNorth*1e-3, gps_data.velocityEast*1e-3,
//                           gps_data.velocityDown*1e-3, gps_data.latitude*DEG_TO_RAD,
//                           gps_data.longitude*DEG_TO_RAD, (gps_data.height*1e-3),
//                           gx, gy, gz,
//                           ax, ay, az, hx, hy, hz);
//
//            printf("Latitude  : %2.7f %2.7f\n", gps_data.latitude, ekf.getLatitude_rad()*RAD_TO_DEG);
//            printf("Longitude : %2.7f %2.7f\n", gps_data.longitude, ekf.getLongitude_rad()*RAD_TO_DEG);
//            printf("Altitude  : %2.3f %2.3f\n", gps_data.height*1e-3, ekf.getAltitude_m());
//            printf("Speed (N) : %2.3f %2.3f\n", gps_data.velocityNorth*1e-3, ekf.getVelNorth_ms());
//            printf("Speed (E) : %2.3f %2.3f\n", gps_data.velocityEast*1e-3, ekf.getVelEast_ms());
//            printf("Speed (D) : %2.3f %2.3f\n", gps_data.velocityDown*1e-3, ekf.getVelDown_ms());
            printf("Roll 	  : %2.3f\n", ekf.getRoll_rad());
            printf("Pitch     : %2.3f\n", ekf.getPitch_rad());
            printf("Yaw       : %2.3f\n", ekf.getHeading_rad());

            printf("\n---------------------\n");
//        } else {
//            printf("No GPS data\n");
//        }
        sleep(1);
    }

    return 0;
}
