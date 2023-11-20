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

    // Initialize plots
    // Accelerometer plot
    figure();
    title("Accelerometer Data");
    auto accel_plot_x = plot(time_axis, accelX, "r-"); // X-axis in red
    hold(on);
    auto accel_plot_y = plot(time_axis, accelY, "g-"); // Y-axis in green
    auto accel_plot_z = plot(time_axis, accelZ, "b-"); // Z-axis in blue
    hold(off);
    xlabel("Time (s)");
    ylabel("Acceleration (m/s^2)");

    // Gyroscope Plot
    figure();
    title("Gyroscope Data");
    auto gryo_plot_x = plot(time_axis, gryoX, "r-"); // X-axis in red
    hold(on);
    auto gryo_plot_y = plot(time_axis, gryoY, "g-"); // Y-axis in green
    auto gryo_plot_z = plot(time_axis, gryoZ, "b-"); // Z-axis in blue
    hold(off);
    xlabel("Time (s)");
    ylabel("Gyroscope (radians/s)");

    // Magetometer plot
    figure();
    title("Magetometer Data");
    auto mag_plot_x = plot(time_axis, magX, "r-"); // X-axis in red
    hold(on);
    auto mag_plot_y = plot(time_axis, magY, "g-"); // Y-axis in green
    auto mag_plot_z = plot(time_axis, magZ, "b-"); // Z-axis in blue
    hold(off);
    xlabel("Time (s)");
    ylabel("Magetometer (uTesla)");

    auto start = std::chrono::steady_clock::now();

    while (true) {
        const int16_t* accelData = imu.getAccelerometerData();
        const int16_t* magData = imu.getMagnetometerData();
        const int16_t* gryoData = imu.getGyroscopeData();

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        time_axis.push_back(elapsed);

        // Update accelerometer data
        accelX.push_back(accelData[X_AXIS]);
        accelY.push_back(accelData[Y_AXIS]);
        accelZ.push_back(accelData[Z_AXIS]);
        accel_plot_x->x_data(time_axis);
        accel_plot_x->y_data(accelX);
        accel_plot_y->y_data(accelY);
        accel_plot_z->y_data(accelZ);
        show(); 

        // Update gyroscope data
        gryoX.push_back(gryoData[X_AXIS]);
        gryoY.push_back(gryoData[Y_AXIS]);
        gryoZ.push_back(gryoData[Z_AXIS]);
        gryo_plot_x->x_data(time_axis);
        gryo_plot_x->y_data(gryoX);
        gryo_plot_y->y_data(gryoY);
        gryo_plot_z->y_data(gryoZ);
        show(); 

        // Update magnetometer data
        magX.push_back(magData[X_AXIS]);
        magY.push_back(magData[Y_AXIS]);
        magZ.push_back(magData[Z_AXIS]);
        mag_plot_x->x_data(time_axis);
        mag_plot_x->y_data(magX);
        mag_plot_y->y_data(magY);
        mag_plot_z->y_data(magZ);
        show(); 

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

