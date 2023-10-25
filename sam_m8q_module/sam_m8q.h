#ifndef SAM_M8Q
#define SAM_M8Q

#pragma once

#include <cstdint>
#include <vector>
#include <map>

#define DEFAULT_I2C_ADDRESS 0x42
#define DATA_STREAM_REGISTER 0xFF
#define DEFAULT_I2C_BUS 1 

#define YEAR_TAG "year"
#define MONTH_TAG "month"
#define DAY_TAG "day"
#define HOUR_TAG "hour"
#define MINUTE_TAG "minute"
#define SECOND_TAG "second"

#define VALID_TIME_TAG "valid_time"
#define VALID_DATE_TAG "valid_date"
#define FULLY_RESOLVED_TAG "fully_resolved"
#define VALID_MAG_DEC_TAG "valid_magnetic_declination"

#define GNSS_FIX_TAG "GNSS_FIX"
#define FIX_STATUS_FLAGS_TAG "fix_flags"
#define NUM_SATELLITES_TAG "num_satellites"

#define LATITUDE_TAG "latitude"
#define LONGITUDE_TAG "longitude"
#define ELLIPSOID_HEIGHT_TAG "ellipsoid_height"
#define MSL_HEIGHT_TAG "MSL_height"
#define HORIZONTAL_ACCURACY_TAG "horizontal_accuracy"
#define VERTICAL_ACCURACY_TAG "vertical_accuracy"

#define NED_VELOCITY_TAG "NED_velocity"
#define GROUND_SPEED_TAG "ground_speed"
#define VEHICLE_HEADING_TAG "vehicle_heading"
#define MOTION_HEADING_TAG "motion_heading"
#define SPEED_ACCURACY_TAG "speed_accuracy"
#define HEADING_ACCURACY_TAG "heading_accuracy"

#define MAGNETIC_DECLINATION_TAG "magnetic_declination"
#define MAG_DEC_ACCURACY_TAG "mag_deg_acc"

#include <vector>
#include <map>
#include "UBX_MSG.h"  // Include the UBX message handling header

class SAM_M8Q {
  private:
      int32_t curr_i2c_addr;
      int32_t curr_i2c_bus;
      std::map<std::string, int32_t> pvt_data;

  public:
      // Constructor
      SAM_M8Q(int32_t i2c_addr = DEFAULT_I2C_ADDRESS, int32_t i2c_bus = 1);
      // Destructor
      ~SAM_M8Q() = default;
      void ubx_only();
      void set_message_frequency(int32_t msg_class, int32_t msg_id, int32_t freq);
      void set_measurement_frequency(int32_t measurement_period_ms, int32_t navigation_rate, int32_t timeref);
      int32_t available_bytes();
      void write_message(const std::vector<uint8_t>& buffer);
      std::vector<uint8_t> read_message();
      std::vector<uint8_t> poll_message(int32_t msg_class, int32_t msg_id);
      std::vector<uint8_t> wait_for_message(int32_t time_out_s, double interval_s, int32_t msg_cls, int32_t msg_id);
      bool wait_for_acknowledge(int32_t msg_class, int32_t msg_id, bool verbose);
      std::map<std::string, int32_t> get_pvt(bool polling, int32_t time_out_s);
};

#endif // SAM_M8Q
