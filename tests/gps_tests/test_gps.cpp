#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {

  Gps gps_module;
  PVTData data = gps_module.GetPvt();

  return 0;
}
