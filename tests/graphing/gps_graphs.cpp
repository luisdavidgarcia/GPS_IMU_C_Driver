#include "../../gps_module/gps.h"
#include <matplot/matplot.h>
#include <vector>
#include <chrono>
#include <thread>

int main() {
    using namespace matplot;

    Gps gps_module;

    std::vector<double> time_axis;
    std::vector<double> longitude, latitude, height, heightMSL;
    std::vector<double> horizAccuracy, vertAccuracy, velNorth, velEast, velDown, groundSpeed, vehHeading, motHeading, speedAccuracy, motHeadAccuracy;

    // Initialize first plot for location and altitude data
    figure();
    auto location_altitude_plot = plot(time_axis, longitude, "r-");
    hold(on);
    plot(time_axis, latitude, "g-");
    plot(time_axis, height, "b-");
    plot(time_axis, heightMSL, "c-");
    hold(off);
    title("GPS Location and Altitude Data");
    xlabel("Time (s)");
    ylabel("Values");

    // Initialize second plot for accuracy and velocity data
    figure();
    auto accuracy_velocity_plot = plot(time_axis, horizAccuracy, "r-");
    hold(on);
    plot(time_axis, vertAccuracy, "g-");
    plot(time_axis, velNorth, "b-");
    plot(time_axis, velEast, "c-");
    plot(time_axis, velDown, "m-");
    plot(time_axis, groundSpeed, "y-");
    plot(time_axis, vehHeading, "k-");
    plot(time_axis, motHeading, "r--");
    plot(time_axis, speedAccuracy, "g--");
    plot(time_axis, motHeadAccuracy, "b--");
    hold(off);
    title("GPS Accuracy and Velocity Data");
    xlabel("Time (s)");
    ylabel("Values");

    while (true) {
        PVTData data = gps_module.GetPvt(true, 1);

        // Updating the time axis
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        time_axis.push_back(duration);

        // Update data for first graph
        longitude.push_back(data.longitude);
        latitude.push_back(data.latitude);
        height.push_back(data.height);
        heightMSL.push_back(data.heightMSL);

        // Update data for second graph
        horizAccuracy.push_back(data.horizontalAccuracy);
        vertAccuracy.push_back(data.verticalAccuracy);
        velNorth.push_back(data.velocityNorth);
        velEast.push_back(data.velocityEast);
        velDown.push_back(data.velocityDown);
        groundSpeed.push_back(data.groundSpeed);
        vehHeading.push_back(data.vehicalHeading);
        motHeading.push_back(data.motionHeading);
        speedAccuracy.push_back(data.speedAccuracy);
        motHeadAccuracy.push_back(data.motionHeadingAccuracy);

        // Reshowing the plots
        location_altitude_plot->x_data(time_axis);
        location_altitude_plot->y_data(longitude);
        plot(time_axis, latitude, "g-"); 
        plot(time_axis, height, "b-");   
        plot(time_axis, heightMSL, "c-"); 
        show(); // Reshow the first plot

        accuracy_velocity_plot->x_data(time_axis);
        accuracy_velocity_plot->y_data(horizAccuracy);
        plot(time_axis, vertAccuracy, "g-");        
        plot(time_axis, velNorth, "b-");              
        plot(time_axis, velEast, "c-");             
        plot(time_axis, velDown, "m-");             
        plot(time_axis, groundSpeed, "y-");           
        plot(time_axis, vehHeading, "k-");            
        plot(time_axis, motHeading, "r--");         
        plot(time_axis, speedAccuracy, "g--");
        plot(time_axis, motHeadAccuracy, "b--");    
        show(); // Reshow the second plot

        // Sleep for 2 seconds
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return 0;
}

