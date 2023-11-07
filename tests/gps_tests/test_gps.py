import melopero_samm8q as mp
import melopero_ubx as ubx
import time
import sys

import pdb; pdb.set_trace()
dev = mp.SAM_M8Q()
dev.ubx_only()
dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_PRT)
dev.set_message_frequency(ubx.NAV_CLASS, ubx.NAV_PVT, 1)
dev.wait_for_acknowledge(ubx.CFG_CLASS ,ubx.CFG_MSG)
dev.set_measurement_frequency(500, 1)
dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_RATE)

#One Measurement every 5 seconds for 300 times
#means 5 * 300 seconds = 1500 seconds = 25 minutes
log_file_name = "log_{}.txt".format(int(time.time() % 1000))
for i in range(10):
    print("Measurement {} / 300".format(i))
    try:
        info = dev.get_pvt() #returns a dictionary containing the PVT data
        if info is not None:
            with open(log_file_name, 'a') as log_file:
              log_file.write("[{}/{}/{}] {}h:{}m:{}s".format(info['year'], info['month'], info['day'],
                                 info['hour'], info['minute'], info['second']))
              log_file.write('\n')
              log_file.write("Longitude : ")
              log_file.write(str(info["longitude"]))
              log_file.write('\n')
              log_file.write("Latitude : ")
              log_file.write(str(info["latitude"]))
              log_file.write('\n')
              log_file.write('\n')
              log_file.flush()
    except:
        print("Unexpected error:", sys.exc_info()[0])

    time.sleep(1)
