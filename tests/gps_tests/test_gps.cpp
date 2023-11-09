#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {
    
    Gps gps_module;
    gps_module.UbxOnly();

    return EXIT_SUCCESS;
}
