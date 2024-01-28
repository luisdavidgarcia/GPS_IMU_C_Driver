#include "imu.h"
#include "ekfIMU.h"
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <chrono>

#define CURRENT_YEAR 2024

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
    EKF_IMU ekf;
    float pitch, roll, yaw;
    float Gxyz[3], Axyz[3], Mxyz[3];
    float filteredAx = 0, filteredAy = 0, filteredAz = 0;
    float filteredMx = 0, filteredMy = 0, filteredMz = 0;
    const float alpha = 0.5; // Adjust this parameter to tweak the filter (range: 0-1)
    const float accel_x_offset = -0.05673657500210876;
    const float accel_y_offset = -0.014051752249833504;
    const float accel_z_offset = -0.700553398935738;
    const float gyro_x_bias = -0.008447830282040561;
    const float gyro_y_bias = 0.0064697791963203004;
    const float gyro_z_bias = -0.009548081446790717;

    auto lastTime = std::chrono::high_resolution_clock::now();

    while(!exit_flag) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - lastTime).count();
        lastTime = currentTime;

        imu_module.ReadSensorData();
        // ... Data acquisition and validation ...
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

        // Apply accelerometer offsets
        Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G - accel_x_offset;
        Axyz[1] = -1 * static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G - accel_y_offset;
        Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G - accel_z_offset;

        // Apply gyroscope biases
        Gxyz[0] = (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[0]) - gyro_x_bias);
        Gxyz[1] = (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[1]) - gyro_y_bias);
        Gxyz[2] = (GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[2]) - gyro_z_bias );

        Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
        Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
        Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

        // // Low-pass filter for accelerometer data
        // filteredAx = alpha * filteredAx + (1 - alpha) * Axyz[0];
        // filteredAy = alpha * filteredAy + (1 - alpha) * Axyz[1];
        // filteredAz = alpha * filteredAz + (1 - alpha) * Axyz[2];

        // // Low-pass filter for magnetometer data
        // filteredMx = alpha * filteredMx + (1 - alpha) * Mxyz[0];
        // filteredMy = alpha * filteredMy + (1 - alpha) * Mxyz[1];
        // filteredMz = alpha * filteredMz + (1 - alpha) * Mxyz[2];

        // Update and Predict?
        // Convert raw sensor readings to Eigen vectors
        Eigen::Vector3f gyro(Gxyz[0], Gxyz[1], Gxyz[2]);
        Eigen::Vector3f accel(Axyz[0], Axyz[1], Axyz[2]);
        Eigen::Vector3f mag(Mxyz[0], Mxyz[1], Mxyz[2]);

        // EKF prediction step
        ekf.Predict(gyro, dt);

        // EKF update step
        ekf.Update(accel, mag);

        // Get the estimated orientation (pitch, roll, yaw)
        std::tie(pitch, roll, yaw) = ekf.GetPitchRollYaw();

        // Write to file
        std::ofstream outfile("tests/kalman_tests/rpy_data.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << roll << "," << pitch << "," << yaw << std::endl;
            outfile.close();
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }

        sleep(1);
    }

    return 0;
}
