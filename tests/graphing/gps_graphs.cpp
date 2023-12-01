// #include "../../gps_module/gps.h"
// #include "matplotlibcpp.h"
// #include <vector>

// namespace plt = matplotlibcpp;

// int main() {
//     Gps gps_module;
//     std::vector<double> time;
//     //std::vector<int32_t> velocityNorth, velocityEast;
//     std::vector<double> velocityNorth, velocityEast;

//     double elapsedTime = 0.0;
//     const double updateInterval = 1.0; // Update interval in seconds
//     const int maxDataPoints = 100; // Maximum number of points to display on graph

//     plt::figure_size(800, 400); // Adjust the size as needed
//     plt::ion(); // Turn on interactive mode

//     while (true) {
//         PVTData data = gps_module.GetPvt(true, 1);

//         if (time.size() > maxDataPoints) {
//             time.erase(time.begin());
//             velocityNorth.erase(velocityNorth.begin());
//             velocityEast.erase(velocityEast.begin());
//         }

//         // Update the time and data vectors
//         time.push_back(elapsedTime);
//         velocityNorth.push_back(data.velocityNorth);
//         velocityEast.push_back(data.velocityEast);

//         // Print type of velocityNorth
//         // printf("Type Velocity North: %s\n", typeid(data.velocityNorth).name());
//         // printf("RAW Velocity North: %d\n", data.velocityNorth);

//         elapsedTime += updateInterval;
//         // Print Last value in velocityNorth
//         // printf("Last Velocity North: %d\n", velocityNorth.back());
//         // printf("Vector: ");
//         // for (int i = 0; i < velocityNorth.size(); i++) {
//         //     printf("%d , ", velocityNorth[i]);
//         // }
//         // printf("\n");

//         plt::figure(1);
//         plt::clf(); // Clear the current figure
//         plt::title("Velocities");
//         plt::named_plot("North Velocity", time, velocityNorth);
//         plt::named_plot("East Velocity", time, velocityEast);
//         plt::legend();
//         plt::pause(0.5);

//         sleep(1);
//     }

//     return 0;
// }

#include "../../gps_module/gps.h"
#include "matplotlibcpp.h"
#include <vector>
#include <unistd.h>  // for sleep

namespace plt = matplotlibcpp;

int main() {
    Gps gps_module;
    std::vector<double> time;
    std::vector<double> velocityNorth, velocityEast;

    double elapsedTime = 0.0;
    const double updateInterval = 1.0; // Update interval in seconds
    const int maxDataPoints = 100; // Maximum number of points to display on graph

    plt::figure_size(800, 400); // Adjust the size as needed
    plt::ion(); // Turn on interactive mode
    plt::title("Velocities");
    plt::xlabel("Time");
    plt::ylabel("Velocity");

    // Create empty plots for North and East velocities
    plt::plot(time, velocityNorth, "b-", "North Velocity");
    plt::plot(time, velocityEast, "r-", "East Velocity");
    plt::legend();

    while (true) {
        PVTData data = gps_module.GetPvt(true, 1);

        if (time.size() > maxDataPoints) {
            time.erase(time.begin());
            velocityNorth.erase(velocityNorth.begin());
            velocityEast.erase(velocityEast.begin());
        }

        // Update the time and data vectors
        time.push_back(elapsedTime);
        velocityNorth.push_back(data.velocityNorth);
        velocityEast.push_back(data.velocityEast);

        // Update the plots with new data
        plt::update(); // Update the plot with new data

        elapsedTime += updateInterval;

        sleep(1);
    }

    return 0;
}
