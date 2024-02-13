#include "gps.h"
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>

#define CURRENT_YEAR 2024

// Define a flag to indicate if the program should exit gracefully.
volatile bool exit_flag = false;

// Signal handler function for Ctrl+C (SIGINT)
void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "Ctrl+C received. Cleaning up..." << std::endl;

        // Set the exit flag to true to trigger graceful exit.
        exit_flag = true;
    }
}

int main(void) {
  // Register the signal handler for SIGINT (Ctrl+C)
  signal(SIGINT, signal_handler);

  Gps gps_module;
  while(!exit_flag) {

    PVTData data = gps_module.GetPvt(true, 1);
    if (data.year == CURRENT_YEAR && data.numberOfSatellites > 0) {
      /* Use these times to plot on x-axis */
      printf("Year: %d\n", data.year);
      printf("Month: %d\n", data.month);
      printf("Day: %d\n", data.day);
      printf("Hour: %d\n", data.hour);
      printf("Min: %d\n", data.min);
      printf("Sec: %d\n\n", data.sec);
      /* Don't Plot These */
      printf("Valid Time Flag: %d\n", data.validTimeFlag);
      printf("Valid Date Flag: %d\n", data.validDateFlag);
      printf("Fully Resolved Flag: %d\n", data.fullyResolved);
      printf("Valid Magnetic Flag: %d\n", data.validMagFlag);
      printf("GNSS Fix: %d\n", data.gnssFix);
      printf("Fix Status Flags: %d\n", data.fixStatusFlags);
      printf("Number of Satellites: %d\n\n", data.numberOfSatellites);
      /* Plot the Long, Lat, and Height */
      printf("Longitude: %f\n", data.longitude);
      printf("Latitude: %f\n", data.latitude);
      printf("Height: %d\n", data.height);
      printf("Height above MSL: %d\n", data.heightMSL);
      printf("Horizontal Accuracy: %u\n", data.horizontalAccuracy);
      printf("Vertical Accuracy: %u\n", data.verticalAccuracy);
      /* Plot the Velocities and Headings */
      printf("North Velocity: %d\n", data.velocityNorth);
      printf("East Velocity: %d\n", data.velocityEast);
      printf("Down Velocity: %d\n", data.velocityDown);
      printf("Ground Speed: %d\n", data.groundSpeed);
      printf("Vehicle Heading: %d\n", data.vehicalHeading);
      printf("Motion Heading: %d\n", data.motionHeading);
      printf("Speed Accuracy: %u\n", data.speedAccuracy);
      printf("Motion Heading Accuracy: %d\n", data.motionHeadingAccuracy);
      /* Don't Plot These */
      //printf("Magnetic Declination: %d\n", data.magneticDeclination);
      //printf("Magnetic Declination Accuracy: %u\n", data.magnetDeclinationAccuracy);
      printf("\n---------------------\n");
    } else {
      printf("No data\n");
    }

    // Sleep for 10 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // sleep(0.5);
  }

  return 0;
}
