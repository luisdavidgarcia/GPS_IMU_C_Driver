#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time, longitude, latitude, height, heightMSL, horizontalAccuracy, verticalAccuracy;
    std::vector<int32_t> velocityNorth, velocityEast, velocityDown, groundSpeed, vehicleHeading, motionHeading;
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
        time.push_back(elapsedTime);
        longitude.push_back(data.longitude);
        latitude.push_back(data.latitude);
        height.push_back(data.height);
        heightMSL.push_back(data.heightMSL);
        horizontalAccuracy.push_back(data.horizontalAccuracy);
        verticalAccuracy.push_back(data.verticalAccuracy);

        // velocityNorth.push_back(static_cast<double>(data.velocityNorth));
        // velocityEast.push_back(static_cast<double>(data.velocityEast));
        // velocityDown.push_back(static_cast<double>(data.velocityDown));
        // groundSpeed.push_back(static_cast<double>(data.groundSpeed));
        // vehicleHeading.push_back(static_cast<double>(data.vehicalHeading));
        // motionHeading.push_back(static_cast<double>(data.motionHeading));
        // speedAccuracy.push_back(static_cast<double>(data.speedAccuracy));
        // motionHeadingAccuracy.push_back(static_cast<double>(data.motionHeadingAccuracy));
        velocityNorth.push_back(data.velocityNorth);
        velocityEast.push_back(data.velocityEast);
        velocityDown.push_back(data.velocityDown);

        printf("RAW Velocity North: %f\n", data.velocityNorth);
        // printf("Cast Velocity East: %f\n", static_cast<double>(data.velocityEast));
        // printf("Cast Velocity Down: %f\n", static_cast<double>(data.velocityDown));

        // groundSpeed.push_back(static_cast<double>(data.groundSpeed));
        // vehicleHeading.push_back(static_cast<double>(data.vehicalHeading));
        // motionHeading.push_back(static_cast<double>(data.motionHeading));
        // speedAccuracy.push_back(static_cast<double>(data.speedAccuracy));
        // motionHeadingAccuracy.push_back(static_cast<double>(data.motionHeadingAccuracy));

        // Clear the current figure and create subplots
        // plt::clf();

        // plt::figure(1);
        // plt::title("Longitude");
        // plt::plot(time, longitude);

        // plt::figure(2);
        // plt::title("Latitude");
        // plt::plot(time, latitude);

        // plt::figure(1);
        // plt::clf(); // Clear the current figure
        // plt::title("Height");
        // plt::named_plot("Height",time, height);
        // plt::named_plot("Height above MSL", time, heightMSL);
        // plt::legend();
        // plt::pause(0.01);

        // plt::figure(2);
        // plt::clf(); // Clear the current figure
        // plt::title("Accuracy");
        // plt::named_plot("Horizontal Accuracy", time, horizontalAccuracy);
        // plt::named_plot("Vertical Accuracy", time, verticalAccuracy);
        // plt::named_plot("Speed Accuracy", time, speedAccuracy);
        // plt::named_plot("Motion Heading Accuracy", time, motionHeadingAccuracy);
        // plt::legend();
        // plt::pause(0.01);

        elapsedTime += updateInterval;
        // Print Last value in velocityNorth
        printf("Last Velocity North: %f\n", velocityNorth.back());
        printf("Vector: ");
        for (int i = 0; i < velocityNorth.size(); i++) {
            printf("%f , ", velocityNorth[i]);
        }
        printf("\n");

        plt::figure(1);
        plt::clf(); // Clear the current figure
        plt::title("Velocities");
        plt::named_plot("North Velocity", time, velocityNorth);
        // plt::named_plot("East Velocity", time, velocityEast);
        // plt::named_plot("Down Velocity", time, velocityDown);
        plt::legend();
        plt::pause(0.1);

        // plt::figure(4);
        // plt::clf(); // Clear the current figure
        // plt::title("Ground Speed");
        // plt::plot(time, groundSpeed);
        // plt::legend();
        // plt::pause(0.01);

        // plt::figure(5);
        // plt::clf(); // Clear the current figure
        // plt::title("Headings");
        // plt::named_plot("Vehical Heading", time, vehicleHeading);
        // plt::named_plot("Motion Heading", time, motionHeading);
        // plt::legend();
        // plt::pause(0.01);

        // Update the plot and wait for the next data update
        // plt::pause(updateInterval);

        // Sleep or wait for the next read cycle
        //usleep(updateInterval * 1e6);
        sleep(1);
    }

    return 0;
}
