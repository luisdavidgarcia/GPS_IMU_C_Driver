#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time;
    std::vector<double> velocityNorth, velocityEast, velocityDown;

    std::vector<double> longitude, latitude;
    //std::vector<double> velocityNorth, velocityEast;

    double elapsedTime = 0.0;
    const double updateInterval = 1.0; // Update interval in seconds
    const int maxDataPoints = 100; // Maximum number of points to display on graph

    plt::figure_size(800, 400); // Adjust the size as needed
    plt::ion(); // Turn on interactive mode

    while (true) {
        PVTData data = gps_module.GetPvt(true, 1);
        if (data.year == 2023) {
            if (time.size() > maxDataPoints) {
                time.erase(time.begin());
                velocityNorth.erase(velocityNorth.begin());
                velocityEast.erase(velocityEast.begin());
                velocityDown.erase(velocityDown.begin());
                longitude.erase(longitude.begin());
                latitude.erase(latitude.begin());
            }

            // Update the time and data vectors
            time.push_back(elapsedTime);
            velocityNorth.push_back(data.velocityNorth);
            velocityEast.push_back(data.velocityEast);
            velocityDown.push_back(data.velocityDown);
            longitude.push_back(data.longitude);
            latitude.push_back(data.latitude);

            // Print type of velocityNorth
            printf("RAW Velocity North: %d\n", data.velocityNorth);
            printf("Vector Velocity North: %f\n", velocityNorth.back());

            elapsedTime += updateInterval;

            plt::figure(1);
            plt::clf(); // Clear the current figure
            plt::title("Velocities");
            plt::named_plot("North Velocity", time, velocityNorth);
            plt::named_plot("East Velocity", time, velocityEast);
            plt::named_plot("Down Velocity", time, velocityDown);
            plt::legend();
            plt::pause(0.1);

            // Plotting the longitude and latitude
             plt::figure(2);
            plt::clf(); // Clear the current figure
            plt::title("GPS Coordinates");
            plt::xlabel("Longitude");
            plt::ylabel("Latitude");

            // Scatter plot for longitude and latitude with larger dots
            plt::scatter(longitude, latitude, 10); // 10 is the size of the dots

            plt::pause(0.1);
        } else {
            printf("GPS data is invalid.\n");
        }

        sleep(1);
    }

    return 0;
}
