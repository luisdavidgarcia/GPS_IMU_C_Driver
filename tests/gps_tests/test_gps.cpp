#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {

  Gps gps_module;
  while(1) {
    PVTData data = gps_module.GetPvt(false, 1);
    sleep(1);
  }

  return 0;
}
