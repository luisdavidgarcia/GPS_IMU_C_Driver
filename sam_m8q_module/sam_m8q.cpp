#include "SAM_M8Q.h"

SAM_M8Q::SAM_M8Q(int32_t i2c_addr, int32_t i2c_bus) : curr_i2c_addr(i2c_addr), curr_i2c_bus(i2c_bus) {
    // Allows the device to setup, avoiding i2c error 5
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/**
 * @brief   Sets the communication protocol to UBX (only) both for input and output
 */
void SAM_M8Q::ubx_only() {
  std::vector<uint8_t> payload = {0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_PRT, 20, payload);
  write_message(message);
}

/**
 * @brief   Send rate is relative to the event a message is registered on.
 *          For example, if the rate of a navigation message is set to 2,
 *          the message is sent every second navigation solution
 * @param   msg_class   UBX message class
 * @param   msg_id      UBX message ID
 * @param   freq        Frequency value
 */
void SAM_M8Q::set_message_frequency(int32_t msg_class, int32_t msg_id, int32_t freq) {
    std::vector<uint8_t> payload = {static_cast<uint8_t>(msg_class), static_cast<uint8_t>(msg_id), static_cast<uint8_t>(freq), 0x00, 0x00, 0x00, 0x00, 0x00};
    std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_MSG, 8, payload);
    write_message(message);
}

/**
 * @brief   Sets the measurement frequency and aligns it to a specified time system.
 * @param   measurement_period_ms    Elapsed time between GNSS measurements
 * @param   navigation_rate         The ratio between the number of measurements and the number of navigation solutions
 * @param   timeref                 The time system to which measurements are aligned
 */
void SAM_M8Q::set_measurement_frequency(int32_t measurement_period_ms, int32_t navigation_rate, int32_t timeref) {
    std::vector<uint8_t> payload = UBX_MSG::int_to_u2(measurement_period_ms);
    payload.insert(payload.end(), UBX_MSG::int_to_u2(navigation_rate).begin(), UBX_MSG::int_to_u2(navigation_rate).end());
    payload.insert(payload.end(), UBX_MSG::int_to_u2(timeref).begin(), UBX_MSG::int_to_u2(timeref).end());

    std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::CFG_CLASS, UBX_MSG::CFG_RATE, 6, payload);
    write_message(message);
}

/**
 * @brief   Returns the number of bytes available if a timeout is specified it
 *          tries to read the number of bytes for the given amount of millis
 * @return  Number of available bytes
 */
int32_t SAM_M8Q::available_bytes() {
  uint8_t msb, lsb;
  {
	  SMBus bus(curr_i2c_bus);
	  msb = bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER);
	  lsb = bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER + 1);
  }
  return (msb << 8) | lsb;
}

/**
 * @brief   Writes a message to the device
 * @param   buffer  Vector containing the message to be written
 */
void SAM_M8Q::write_message(const std::vector<uint8_t>& buffer) {
  SMBus bus(curr_i2c_bus);
  i2c_msg msg_out(curr_i2c_addr, I2C_MSG_WRITE, buffer.size(), const_cast<uint8_t*>(buffer.data()));
  bus.i2c_rdwr({msg_out});
}

/**
 * @brief   Reads a message from the device
 * @return  Vector containing the read message
 */
std::vector<uint8_t> SAM_M8Q::read_message() {
  int32_t msg_length = available_bytes();
  std::vector<uint8_t> msg;

  if (msg_length > UBX_MSG::MAX_MESSAGE_LENGTH) {
	  return msg;
  }

  {
	  SMBus bus(curr_i2c_bus);
	  for (int32_t i = 0; i < msg_length; ++i) {
		  msg.push_back(bus.read_byte_data(curr_i2c_addr, DATA_STREAM_REGISTER));
	  }
  }

  return msg;
}

/**
 * @brief   Sends a polling message to request a specific UBX message
 * @param   msg_class   UBX message class
 * @param   msg_id      UBX message ID
 * @return  Vector containing the received message
 */
std::vector<uint8_t> SAM_M8Q::poll_message(int32_t msg_class, int32_t msg_id) {
  std::vector<uint8_t> msg = UBX_MSG::compose_message(msg_class, msg_id, 0);
  write_message(msg);
  return wait_for_message(1, 0.01, msg_class, msg_id);
}

/**
 * @brief   Waits for a specific UBX message to arrive
 * @param   time_out_s   Maximum amount of time to wait for the message to arrive in seconds
 * @param   interval_s   Interval in seconds between two readings
 * @param   msg_cls      The class of the message to wait for
 * @param   msg_id       The ID of the message to wait for
 * @return  Vector containing the received message, or an empty vector if timed out
 */
std::vector<uint8_t> SAM_M8Q::wait_for_message(int32_t time_out_s, double interval_s, int32_t msg_cls, int32_t msg_id) {
  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<uint8_t> to_compare = {UBX_MSG::SYNC_CHAR_1, UBX_MSG::SYNC_CHAR_2, static_cast<uint8_t>(msg_cls), static_cast<uint8_t>(msg_id)};
  std::vector<uint8_t> msg;

  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() < time_out_s) {
	  msg = read_message();
	  if (msg.size() >= to_compare.size() && std::equal(to_compare.begin(), to_compare.end(), msg.begin())) {
		  return msg;
	  }
	  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int32_t>(interval_s * 1000)));
  }

  return msg;
}

/**
 * @brief   Waits for an acknowledge message to be received
 * @param   msg_class   UBX message class
 * @param   msg_id      UBX message ID
 * @param   verbose     Whether to print verbose information
 * @return  True if the message was acknowledged, false otherwise
 */
bool SAM_M8Q::wait_for_acknowledge(int32_t msg_class, int32_t msg_id, bool verbose) {
    bool ack = false;
    std::vector<uint8_t> msg = wait_for_message(1, 0.01, UBX_MSG::ACK_CLASS);
    if (!msg.empty()) {
        if (msg[3] == UBX_MSG::ACK_ACK && msg[6] == static_cast<uint8_t>(msg_class) && msg[7] == static_cast<uint8_t>(msg_id)) {
            ack = true;
        }
    }

    if (verbose) {
        std::cout << " A message of class : " << UBX_MSG::msg_class_to_string(msg[6]) << " and id : " << static_cast<int32_t>(msg[7])
                  << " was " << (ack ? "" : "not ") << "acknowledged" << std::endl;
    }

    return ack;
}

/**
 * @brief   Updates and returns the pvt_data dictionary that contains the last received pvt data
 * @param   polling         If true, the pvt message is polled, else waits for the next navigation solution
 * @param   time_out_s      The maximum time to wait for the message
 * @return  A map containing the PVT data
 */
std::map<std::string, int32_t> SAM_M8Q::get_pvt(bool polling, int32_t time_out_s) {
    if (polling) {
        std::vector<uint8_t> message = UBX_MSG::compose_message(UBX_MSG::NAV_CLASS, UBX_MSG::NAV_PVT);
        write_message(message);
    }

    std::vector<uint8_t> read = wait_for_message(time_out_s, 0.01, UBX_MSG::NAV_CLASS, UBX_MSG::NAV_PVT);
    std::map<std::string, int32_t> pvt_data;

    if (!read.empty()) {
        int32_t start_payload = 6;

        // WARNING: POSITION_DOP AND ITOW ARE MISSING (NOT RETRIEVED)

        // Time solution
        int32_t year = UBX_MSG::u2_to_int({read[start_payload + 4], read[start_payload + 5]});
        int32_t month = read[start_payload + 6];
        int32_t day = read[start_payload + 7];
        int32_t hour = read[start_payload + 8];
        int32_t minutes = read[start_payload + 9];
        int32_t sec = read[start_payload + 10];
        int32_t valid_flag = read[start_payload + 11];

        // clarifying flags
        bool valid_date = valid_flag & 0x01;
        bool valid_time = valid_flag & 0x02;
        bool fully_resolved = valid_flag & 0x04;
        bool valid_mag = valid_flag & 0x08;

        // GNSS fix and flags
        int32_t gnss_fix = UBX_MSG::get_gnss_fix_type(read[start_payload + 20]);
        std::vector<uint8_t> fix_status_flags(read.begin() + start_payload + 21, read.begin() + start_payload + 23);
        int32_t num_satellites = read[start_payload + 23];

        // longitude and latitude are in Degrees
        double longitude = UBX_MSG::i4_to_int({read[start_payload + 24], read[start_payload + 25], read[start_payload + 26], read[start_payload + 27]}) * 1e-07;
        double latitude = UBX_MSG::i4_to_int({read[start_payload + 28], read[start_payload + 29], read[start_payload + 30], read[start_payload + 31]}) * 1e-07;

        // height, mean sea level height in millimeters
        int32_t height = UBX_MSG::i4_to_int({read[start_payload + 32], read[start_payload + 33], read[start_payload + 34], read[start_payload + 35]});
        int32_t height_MSL = UBX_MSG::i4_to_int({read[start_payload + 36], read[start_payload + 37], read[start_payload + 38], read[start_payload + 39]});

        // horizontal and vertical accuracy estimates in millimeters
        int32_t h_acc = UBX_MSG::u4_to_int({read[start_payload + 40], read[start_payload + 41], read[start_payload + 42], read[start_payload + 43]});
        int32_t v_acc = UBX_MSG::u4_to_int({read[start_payload + 44], read[start_payload + 45], read[start_payload + 46], read[start_payload + 47]});

        // North East Down velocity in mm / s
        int32_t n_vel = UBX_MSG::i4_to_int({read[start_payload + 48], read[start_payload + 49], read[start_payload + 50], read[start_payload + 51]});
        int32_t e_vel = UBX_MSG::i4_to_int({read[start_payload + 52], read[start_payload + 53], read[start_payload + 54], read[start_payload + 55]});
        int32_t d_vel = UBX_MSG::i4_to_int({read[start_payload + 56], read[start_payload + 57], read[start_payload + 58], read[start_payload + 59]});

        // Ground speed in mm / s and heading of motion in degrees + speed and heading accuracy estimates
        int32_t g_speed = UBX_MSG::i4_to_int({read[start_payload + 60], read[start_payload + 61], read[start_payload + 62], read[start_payload + 63]});
        double motion_heading = UBX_MSG::i4_to_int({read[start_payload + 64], read[start_payload + 65], read[start_payload + 66], read[start_payload + 67]}) * 1e-05;
        int32_t s_acc = UBX_MSG::u4_to_int({read[start_payload + 68], read[start_payload + 69], read[start_payload + 70], read[start_payload + 71]});
        double m_acc = UBX_MSG::u4_to_int({read[start_payload + 72], read[start_payload + 73], read[start_payload + 74], read[start_payload + 75]}) * 1e-05;

        // Heading of vehicle in degrees
        double vehicle_heading = UBX_MSG::i4_to_int({read[start_payload + 84], read[start_payload + 85], read[start_payload + 86], read[start_payload + 87]}) * 1e-05;

        // Magnetic declination and magnetic declination accuracy both in degrees
        double mag_deg = UBX_MSG::i2_to_int({read[start_payload + 88], read[start_payload + 89]}) * 1e-02;
        double mag_deg_acc = UBX_MSG::u2_to_int({read[start_payload + 90], read[start_payload + 91]}) * 1e-02;

        // time
        pvt_data[YEAR_TAG] = year;
        pvt_data[MONTH_TAG] = month;
        pvt_data[DAY_TAG] = day;
        pvt_data[HOUR_TAG] = hour;
        pvt_data[MINUTE_TAG] = minutes;
        pvt_data[SECOND_TAG] = sec;

        // flags
        pvt_data[VALID_TIME_TAG] = valid_time;
        pvt_data[VALID_DATE_TAG] = valid_date;
        pvt_data[FULLY_RESOLVED_TAG] = fully_resolved;
        pvt_data[VALID_MAG_DEC_TAG] = valid_mag;

        // GNSS
        pvt_data[GNSS_FIX_TAG] = gnss_fix;
        pvt_data[FIX_STATUS_FLAGS_TAG] = fix_status_flags;
        pvt_data[NUM_SATELLITES_TAG] = num_satellites;

        // Coordinates
        pvt_data[LONGITUDE_TAG] = longitude;
        pvt_data[LATITUDE_TAG] = latitude;
        pvt_data[ELLIPSOID_HEIGHT_TAG] = height;
        pvt_data[MSL_HEIGHT_TAG] = height_MSL;
        pvt_data[HORIZONTAL_ACCURACY_TAG] = h_acc;
        pvt_data[VERTICAL_ACCURACY_TAG] = v_acc;

        // Velocity and heading
        pvt_data[NED_VELOCITY_TAG] = std::make_tuple(n_vel, e_vel, d_vel);
        pvt_data[GROUND_SPEED_TAG] = g_speed;
        pvt_data[VEHICLE_HEADING_TAG] = vehicle_heading;
        pvt_data[MOTION_HEADING_TAG] = motion_heading;
        pvt_data[SPEED_ACCURACY_TAG] = s_acc;
        pvt_data[HEADING_ACCURACY_TAG] = m_acc;

        // Magnetic declination
        pvt_data[MAGNETIC_DECLINATION_TAG] = mag_deg;
        pvt_data[MAG_DEC_ACCURACY_TAG] = mag_deg_acc;
    }

    return pvt_data;
}

