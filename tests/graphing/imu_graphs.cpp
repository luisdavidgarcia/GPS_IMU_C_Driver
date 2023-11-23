#include "../../imu_module/imu.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

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
        const int16_t *accel_data = imu_module.getAccelData();
        const int16_t *gyro_data = imu_module.getGyroData();
        const int16_t *mag_data = imu_module.getMagData();

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

        // Clear and plot new data
        plt::clf(); // Clear the current figure
        plt::subplot(3, 1, 1);
        plt::named_plot("Accel X", time, accel_x);
        plt::named_plot("Accel Y", time, accel_y);
        plt::named_plot("Accel Z", time, accel_z);
        plt::legend();

        plt:subplot(3, 1, 2);
        plt::named_plot("Gyro X", time, gyro_x);
        plt::named_plot("Gyro Y", time, gyro_y);
        plt::named_plot("Gyro Z", time, gyro_z);
        plt::legend();

        plt:subplot(3, 1, 3);
        plt::named_plot("Mag X", time, mag_x);
        plt::named_plot("Mag Y", time, mag_y);
        plt::named_plot("Mag Z", time, mag_z);
        plt::legend();

        plt::pause(updateInterval); // Update the plot

        // Sleep or wait for the next read cycle
        usleep(updateInterval * 1e6);
    }

    return 0;
}
