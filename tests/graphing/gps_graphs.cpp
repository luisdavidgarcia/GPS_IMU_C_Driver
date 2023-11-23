#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time, longitude, latitude, height, heightMSL, horizontalAccuracy, verticalAccuracy;
    std::vector<double> velocityNorth, velocityEast, velocityDown, groundSpeed, vehicleHeading, motionHeading;
    std::vector<double> speedAccuracy, motionHeadingAccuracy;

    double elapsedTime = 0.0;
    const double updateInterval = 1.0; // Update interval in seconds
    const int maxDataPoints = 100; // Maximum number of points to display on graph

    plt::figure_size(800, 400); // Adjust the size as needed

    while (true) {
        PVTData data = gps_module.GetPvt(true, 1);

        if (time.size() > maxDataPoints) {
            time.erase(time.begin());
            longitude.erase(longitude.begin());
            latitude.erase(latitude.begin());
            height.erase(height.begin());
            heightMSL.erase(heightMSL.begin());
            horizontalAccuracy.erase(horizontalAccuracy.begin());
            verticalAccuracy.erase(verticalAccuracy.begin());

            velocityNorth.erase(velocityNorth.begin());
            velocityEast.erase(velocityEast.begin());
            velocityDown.erase(velocityDown.begin());
            groundSpeed.erase(groundSpeed.begin());
            vehicleHeading.erase(vehicleHeading.begin());
            motionHeading.erase(motionHeading.begin());
            speedAccuracy.erase(speedAccuracy.begin());
            motionHeadingAccuracy.erase(motionHeadingAccuracy.begin());
        }

        // Update the time and data vectors
        double current_time = data.hour + data.min / 60.0 + data.sec / 3600.0;
        time.push_back(current_time);
        longitude.push_back(static_cast<double>(data.longitude));
        latitude.push_back(static_cast<double>(data.latitude));
        height.push_back(static_cast<double>(data.height));
        heightMSL.push_back(static_cast<double>(data.heightMSL));
        horizontalAccuracy.push_back(static_cast<double>(data.horizontalAccuracy));
        verticalAccuracy.push_back(static_cast<double>(data.verticalAccuracy));

        velocityNorth.push_back(static_cast<double>(data.velocityNorth));
        velocityEast.push_back(static_cast<double>(data.velocityEast));
        velocityDown.push_back(static_cast<double>(data.velocityDown));
        groundSpeed.push_back(static_cast<double>(data.groundSpeed));
        vehicleHeading.push_back(static_cast<double>(data.vehicalHeading));
        motionHeading.push_back(static_cast<double>(data.motionHeading));
        speedAccuracy.push_back(static_cast<double>(data.speedAccuracy));
        motionHeadingAccuracy.push_back(static_cast<double>(data.motionHeadingAccuracy));

        // Clear the current figure and create subplots
        plt::clf();

        plt::figure(1);
        plt::title("Longitude");
        plt::plot(time, longitude);

        plt::figure(2);
        plt::title("Latitude");
        plt::plot(time, latitude);

        plt::figure(3);
        plt::title("Height");
        plt::plot(time, height);

        plt::figure(4);
        plt::title("Height above MSL");
        plt::plot(time, heightMSL);

        plt::figure(5);
        plt::title("Horizontal Accuracy");
        plt::plot(time, horizontalAccuracy);

        plt::figure(6);
        plt::title("Vertical Accuracy");
        plt::plot(time, verticalAccuracy);

        plt::figure(7);
        plt::title("North Velocity");
        plt::plot(time, velocityNorth);

        plt::figure(8);
        plt::title("East Velocity");
        plt::plot(time, velocityEast);

        plt::figure(9);
        plt::title("Down Velocity");
        plt::plot(time, velocityDown);

        plt::figure(10);
        plt::title("Ground Speed");
        plt::plot(time, groundSpeed);

        plt::figure(11);
        plt::title("Vehicle Heading");
        plt::plot(time, vehicleHeading);

        plt::figure(12);
        plt::title("Motion Heading");
        plt::plot(time, motionHeading);

        plt::figure(13);
        plt::title("Speed Accuracy");
        plt::plot(time, speedAccuracy);

        plt::figure(14);
        plt::title("Motion Heading Accuracy");
        plt::plot(time, motionHeadingAccuracy);

        // Update the plot and wait for the next data update
        plt::pause(updateInterval);
        elapsedTime += updateInterval;
        sleep(1);
    }

    return 0;
}
