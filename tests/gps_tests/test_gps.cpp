#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {

  Gps gps_module;
  while(1) {
    PVTData data = gps_module.GetPvt(true, 1);
    if (data.year == 2023) {
      printf("Year: %d\n", data.year);
      printf("Month: %d\n", data.month);
      printf("Day: %d\n", data.day);  
      printf("Hour: %d\n", data.hour);
      printf("Min: %d\n", data.min);
      printf(
          "Sec: %d\n", data.sec);  // TODO: Convert to seconds with decimal
    }
    sleep(1);
  }

  return 0;
}
