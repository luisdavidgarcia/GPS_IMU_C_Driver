import melopero_samm8q as mp
import melopero_ubx as ubx
import time
import sys

YEAR_TAG = "year"
MONTH_TAG = "month"
DAY_TAG = "day"
HOUR_TAG = "hour"
MINUTE_TAG = "minute"
SECOND_TAG = "second"

VALID_TIME_TAG = "valid_time"
VALID_DATE_TAG = "valid_date"
FULLY_RESOLVED_TAG = "fully_resolved"
VALID_MAG_DEC_TAG = "valid_magnetic_declination"

GNSS_FIX_TAG = "GNSS_FIX"
FIX_STATUS_FLAGS_TAG = "fix_flags"
NUM_SATELLITES_TAG = "num_satellites"

LATITUDE_TAG = "latitude"
LONGITUDE_TAG = "longitude"
ELLIPSOID_HEIGHT_TAG = "ellipsoid_height"
MSL_HEIGHT_TAG = "MSL_height"
HORIZONTAL_ACCURACY_TAG = "horizontal_accuracy"
VERTICAL_ACCURACY_TAG = "vertical_accuracy"

NED_VELOCITY_TAG = "NED_velocity"
GROUND_SPEED_TAG = "ground_speed"
VEHICLE_HEADING_TAG = "vehicle_heading"
MOTION_HEADING_TAG = "motion_heading"
SPEED_ACCURACY_TAG = "speed_accuracy"
HEADING_ACCURACY_TAG = "heading_accuracy"

MAGNETIC_DECLINATION_TAG = "magnetic_declination"
MAG_DEC_ACCURACY_TAG = "mag_deg_acc"


#import pdb; pdb.set_trace()
dev = mp.SAM_M8Q()
dev.ubx_only()
# #dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_PRT)
dev.set_message_frequency(ubx.NAV_CLASS, ubx.NAV_PVT, 1)
# #dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_MSG)
#dev.set_measurement_frequency(500, 1)
#dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_RATE)
#info = dev.get_pvt()
# One Measurement every 5 seconds for 300 times
# means 5 * 300 seconds = 1500 seconds = 25 minutes
log_file_name = "log_dog.txt"
for i in range(10):
    print("Measurement {} / 300".format(i))
    try:
        info = dev.get_pvt()  # returns a dictionary containing the PVT data
        if info is not None:
            with open(log_file_name, "a") as log_file:
                log_file.write(
                    "[{}/{}/{}] {}h:{}m:{}s".format(
                        info[YEAR_TAG],
                        info[MONTH_TAG],
                        info[DAY_TAG],
                        info[HOUR_TAG],
                        info[MINUTE_TAG],
                        info[SECOND_TAG],
                    )
                )
                log_file.write("\n")

                # Add other data fields here
                log_file.write("Valid Time: {}\n".format(info.get(VALID_TIME_TAG, 'N/A')))
                log_file.write("Valid Date: {}\n".format(info.get(VALID_DATE_TAG, 'N/A')))
                log_file.write("Fully Resolved: {}\n".format(info.get(FULLY_RESOLVED_TAG, 'N/A')))
                log_file.write("Valid Magnetic Declination: {}\n".format(info.get(VALID_MAG_DEC_TAG, 'N/A')))
                log_file.write("GNSS Fix: {}\n".format(info.get(GNSS_FIX_TAG, 'N/A')))
                log_file.write("Fix Status Flags: {}\n".format(info.get(FIX_STATUS_FLAGS_TAG, 'N/A')))
                log_file.write("Number of Satellites: {}\n".format(info.get(NUM_SATELLITES_TAG, 'N/A')))
                log_file.write("Latitude: {}\n".format(info.get(LATITUDE_TAG, 'N/A')))
                log_file.write("Longitude: {}\n".format(info.get(LONGITUDE_TAG, 'N/A')))
                log_file.write("Ellipsoid Height: {}\n".format(info.get(ELLIPSOID_HEIGHT_TAG, 'N/A')))
                log_file.write("MSL Height: {}\n".format(info.get(MSL_HEIGHT_TAG, 'N/A')))
                log_file.write("Horizontal Accuracy: {}\n".format(info.get(HORIZONTAL_ACCURACY_TAG, 'N/A')))
                log_file.write("Vertical Accuracy: {}\n".format(info.get(VERTICAL_ACCURACY_TAG, 'N/A')))
                log_file.write("NED Velocity: {}\n".format(info.get(NED_VELOCITY_TAG, 'N/A')))
                log_file.write("Ground Speed: {}\n".format(info.get(GROUND_SPEED_TAG, 'N/A')))
                log_file.write("Vehicle Heading: {}\n".format(info.get(VEHICLE_HEADING_TAG, 'N/A')))
                log_file.write("Motion Heading: {}\n".format(info.get(MOTION_HEADING_TAG, 'N/A')))
                log_file.write("Speed Accuracy: {}\n".format(info.get(SPEED_ACCURACY_TAG, 'N/A')))
                log_file.write("Heading Accuracy: {}\n".format(info.get(HEADING_ACCURACY_TAG, 'N/A')))
                log_file.write("Magnetic Declination: {}\n".format(info.get(MAGNETIC_DECLINATION_TAG, 'N/A')))
                log_file.write("Magnetic Declination Accuracy: {}\n".format(info.get(MAG_DEC_ACCURACY_TAG, 'N/A')))

                log_file.write("\n")
                log_file.flush()
    except:
        print("Unexpected error:", sys.exc_info()[0])

    time.sleep(1)
