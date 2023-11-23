#include "../../imu_module/imu.h"
#include "matplotlibcpp.h"
#include <vector>
#include <unistd.h> // for sleep

namespace plt = matplotlibcpp;

void plotData(const std::string &title, const std::vector<double> &time, const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z) {
    plt::clf(); // Clear the current figure
    plt::named_plot("X", time, x);
    plt::named_plot("Y", time, y);
    plt::named_plot("Z", time, z);
    plt::title(title);
    plt::legend();
}

int main() {
    Imu imu_module;
    std::vector<double> time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

    double elapsedTime = 0.0;
    const double updateInterval = 0.1; // 100 ms
    const int maxDataPoints = 100; // Maximum number of points to display on graph

    // Initialize matplotlib
    plt::figure_size(1200, 780); // Set the size of the figure

    while (true) {
        imu_module.readSensorData();
        const int16_t *accel_data = imu_module.getAccelerometerData();
        const int16_t *gyro_data = imu_module.getGyroscopeData();
        const int16_t *mag_data = imu_module.getMagnetometerData();

        // Update the data vectors
        if (time.size() > maxDataPoints) {
            time.erase(time.begin());
            accel_x.erase(accel_x.begin());
            accel_y.erase(accel_y.begin());
            accel_z.erase(accel_z.begin());
            gyro_x.erase(gyro_x.begin());
            gyro_y.erase(gyro_y.begin());
            gyro_z.erase(gyro_z.begin());
            mag_x.erase(mag_x.begin());
            mag_y.erase(mag_y.begin());
            mag_z.erase(mag_z.begin());
        }

        time.push_back(elapsedTime);
        accel_x.push_back(accel_data[0]);
        accel_y.push_back(accel_data[1]);
        accel_z.push_back(accel_data[2]);
        gyro_x.push_back(gyro_data[0]);
        gyro_y.push_back(gyro_data[1]);
        gyro_z.push_back(gyro_data[2]);
        mag_x.push_back(mag_data[0]);
        mag_y.push_back(mag_data[1]);
        mag_z.push_back(mag_data[2]);

        elapsedTime += updateInterval;

        // TODO: Try to generate subplots
        // Plot Accelerometer Data
        plotData("Accelerometer Data", time, accel_x, accel_y, accel_z);

        // Plot Gyroscope Data
        plotData("Gyroscope Data", time, gyro_x, gyro_y, gyro_z);

        // Plot Magnetometer Data
        plotData("Magnetometer Data", time, mag_x, mag_y, mag_z);

        plt::pause(updateInterval); // Update the plot

        // Sleep or wait for the next read cycle
        usleep(updateInterval * 1e6);
    }

    return 0;
}
