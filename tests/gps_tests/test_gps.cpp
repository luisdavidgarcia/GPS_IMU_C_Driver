#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {

if (data.year == 2023) {
      printf("Year: %d\n", data.year);
      printf("Month: %d\n", data.month);
      printf("Day: %d\n", data.day);  
      printf("Hour: %d\n", data.hour);
      printf("Min: %d\n", data.min);
      printf("Sec: %d\n", data.sec);
      printf("Valid Time Flag: %d\n", data.validTimeFlag);
      printf("Valid Date Flag: %d\n", data.validDateFlag);
      printf("Fully Resolved Flag: %d\n", data.fullyResolved);
      printf("Valid Magnetic Flag: %d\n", data.validMagFlag);
      printf("GNSS Fix: %d\n", data.gnssFix);
      printf("Fix Status Flags: %d\n", data.fixStatusFlags);
      printf("Number of Satellites: %d\n", data.numberOfSatellites);
      printf("Longitude: %d\n", data.longitude);
      printf("Latitude: %d\n", data.latitude);
      printf("Height: %d\n", data.height);
      printf("Height above MSL: %d\n", data.heightMSL);
      printf("Horizontal Accuracy: %u\n", data.horizontalAccuracy);
      printf("Vertical Accuracy: %u\n", data.verticalAccuracy);
      printf("North Velocity: %d\n", data.velocityNorth);
      printf("East Velocity: %d\n", data.velocityEast);
      printf("Down Velocity: %d\n", data.velocityDown);
      printf("Ground Speed: %d\n", data.groundSpeed);
      printf("Vehicle Heading: %d\n", data.vehicalHeading);
      printf("Motion Heading: %d\n", data.motionHeading);
      printf("Speed Accuracy: %u\n", data.speedAccuracy);
      printf("Motion Heading Accuracy: %d\n", data.motionHeadingAccuracy);
      printf("Magnetic Declination: %d\n", data.magneticDeclination);
      printf("Magnetic Declination Accuracy: %u\n", data.magnetDeclinationAccuracy);
      printf("\n---------------------\n");
    } else {
      printf("No data\n"); 
    }

  return 0;
}
