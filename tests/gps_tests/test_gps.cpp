#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {

  Gps gps_module;
  while(1) {
    PVTData data = gps_module.GetPvt(true, 1);
    printf("Year: %d\n", data.year);
    sleep(1);
  }

  return 0;
}
