#include "../../imu_module/imu.h"
#include <matplot/matplot.h>
#include <vector>
#include <chrono>
#include <thread>
#include <math.h>

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

int main() {
    using namespace matplot;

    Imu imu;
    std::vector<double> time_axis;
    std::vector<double> accelX, accelY, accelZ;
    std::vector<double> magX, magY, magZ;
    std::vector<double> gyroX, gyroY, gyroZ;

    // Create a figure and set up subplots
    auto f = figure(true); // true for keeping the figure window open
    f->layout(3, 1); // 3 rows, 1 column

    // Accelerometer subplot
    subplot(3, 1, 0);
    title("Accelerometer Data");
    auto accel_plot_x = plot(time_axis, accelX, "r-"); // X-axis in red
    auto accel_plot_y = plot(time_axis, accelY, "g-"); // Y-axis in green
    auto accel_plot_z = plot(time_axis, accelZ, "b-"); // Z-axis in blue
    xlabel("Time (s)");
    ylabel("Acceleration (m/s^2)");

    // Gyroscope subplot
    subplot(3, 1, 1);
    title("Gyroscope Data");
    auto gyro_plot_x = plot(time_axis, gyroX, "r-"); // X-axis in red
    auto gyro_plot_y = plot(time_axis, gyroY, "g-"); // Y-axis in green
    auto gyro_plot_z = plot(time_axis, gyroZ, "b-"); // Z-axis in blue
    xlabel("Time (s)");
    ylabel("Gyroscope (radians/s)");

    // Magnetometer subplot
    subplot(3, 1, 2);
    title("Magnetometer Data");
    auto mag_plot_x = plot(time_axis, magX, "r-"); // X-axis in red
    auto mag_plot_y = plot(time_axis, magY, "g-"); // Y-axis in green
    auto mag_plot_z = plot(time_axis, magZ, "b-"); // Z-axis in blue
    xlabel("Time (s)");
    ylabel("Magnetometer (uTesla)");

    auto start = std::chrono::steady_clock::now();

    while (true) {
        imu.readSensorData();
        const int16_t* accelData = imu.getAccelerometerData();
        const int16_t* magData = imu.getMagnetometerData();
        const int16_t* gyroData = imu.getGyroscopeData();

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        time_axis.push_back(elapsed);

        // Update accelerometer data
        accel_plot_x->x_data(time_axis).y_data(accelX);
        accel_plot_y->y_data(accelY);
        accel_plot_z->y_data(accelZ);

        // Update gyroscope data
        gyro_plot_x->x_data(time_axis).y_data(gyroX);
        gyro_plot_y->y_data(gyroY);
        gyro_plot_z->y_data(gyroZ);

        // Update magnetometer data
        mag_plot_x->x_data(time_axis).y_data(magX);
        mag_plot_y->y_data(magY);
        mag_plot_z->y_data(magZ);

        // Redraw the plot
        show(); // or draw() based on your Matplot++ version

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

