#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {
    
    Gps gps_module;
    gps_module.UbxOnly();
    gps_module.WaitForAcknowledge(CFG_CLASS, CFG_PRT);

    return 0;
}
