#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time;
    std::vector<int32_t> velocityNorth;

    double elapsedTime = 0.0;
    const double updateInterval = 1.0; // Update interval in seconds
    const int maxDataPoints = 100; // Maximum number of points to display on graph

    plt::figure_size(800, 400); // Adjust the size as needed

    while (true) {
        PVTData data = gps_module.GetPvt(true, 1);

        if (time.size() > maxDataPoints) {
            time.erase(time.begin());
            velocityNorth.erase(velocityNorth.begin());
        }

        // Update the time and data vectors
        time.push_back(elapsedTime);
        velocityNorth.push_back(data.velocityNorth);

        // Print type of velocityNorth
        printf("Type Velocity North: %s\n", typeid(data.velocityNorth).name());
        printf("RAW Velocity North: %d\n", data.velocityNorth);

        elapsedTime += updateInterval;
        // Print Last value in velocityNorth
        printf("Last Velocity North: %d\n", velocityNorth.back());
        printf("Vector: ");
        for (int i = 0; i < velocityNorth.size(); i++) {
            printf("%d , ", velocityNorth[i]);
        }
        printf("\n");

        plt::figure(1);
        plt::clf(); // Clear the current figure
        plt::title("Velocities");
        plt::named_plot("North Velocity", time, velocityNorth);
        plt::legend();
        plt::pause(0.1);


        sleep(1);
    }

    return 0;
}
