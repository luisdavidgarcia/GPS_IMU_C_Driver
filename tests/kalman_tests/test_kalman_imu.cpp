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
    float filteredAx, filteredAy, filteredAz;
    float filteredMx, filteredMy, filteredMz;
    const float alpha = 0.5; // Low-pass filter constant

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

        Gxyz[0] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[0]));// - G_offset[0]);
        Gxyz[1] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[1]));// - G_offset[1]);
        Gxyz[2] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * (static_cast<float>(gyro_data[2]));// - G_offset[2]);
        Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Axyz[1] = static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
        Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
        Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
        Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

        // Low-pass filter
        filteredAx = alpha * filteredAx + (1 - alpha) * Axyz[0];
        filteredAy = alpha * filteredAy + (1 - alpha) * Axyz[1];
        filteredAz = alpha * filteredAz + (1 - alpha) * Axyz[2];
        filteredMx = alpha * filteredMx + (1 - alpha) * Mxyz[0];
        filteredMy = alpha * filteredMy + (1 - alpha) * Mxyz[1];
        filteredMz = alpha * filteredMz + (1 - alpha) * Mxyz[2];

        ekf.predict(Gxyz[0], Gxyz[1], Gxyz[2], dt);
        ekf.update(filteredAx, filteredAy, filteredAz, filteredMx, filteredMy, filteredMz);

        auto [pitch, roll, yaw] = ekf.getOrientation();
        printf("Roll: %2.3f, Pitch: %2.3f, Yaw: %2.3f\n", roll, pitch, yaw);

        // Write to file
        std::ofstream outfile("tests/kalman_tests/rpy_data.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << roll << "," << pitch << "," << yaw << std::endl;
            outfile.close();
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }

        sleep(0.5);
    }

    return 0;
}
