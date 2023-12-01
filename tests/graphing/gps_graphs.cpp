#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time;
    std::vector<double> velocityNorth, velocityEast;
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
            }

            if ((data.velocityNorth < 500 || data.velocityNorth > -500) && (data.velocityEast < 500 || data.velocityEast > -500)) {
                // Update the time and data vectors
                time.push_back(elapsedTime);
                velocityNorth.push_back(data.velocityNorth);
                velocityEast.push_back(data.velocityEast);

                // Print type of velocityNorth
                printf("RAW Velocity North: %d\n", data.velocityNorth);
                printf("Vector Velocity North: %f\n", velocityNorth.back());

                elapsedTime += updateInterval;

                plt::figure(1);
                plt::clf(); // Clear the current figure
                plt::title("Velocities");
                plt::named_plot("North Velocity", time, velocityNorth);
                plt::named_plot("East Velocity", time, velocityEast);
                plt::legend();
                plt::pause(0.5);
            }
        }

        sleep(1);
    }

    return 0;
}
