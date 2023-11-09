#include "../../gps_module/gps.h"
#include <stdio.h>

int main(void) {
    
    Gps gps_module;
    uint16_t num_bytes = gps_module.getAvailableBytes();

    return EXIT_SUCCESS;
}
