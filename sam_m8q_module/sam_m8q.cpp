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
    // ...
}

/**
 * @brief   Waits for an acknowledge message to be received
 * @param   msg_class   UBX message class
 * @param   msg_id      UBX message ID
 * @param   verbose     Whether to print verbose information
 * @return  True if the message was acknowledged, false otherwise
 */
bool SAM_M8Q::wait_for_acknowledge(int32_t msg_class, int32_t msg_id, bool verbose) {
}

/**
 * @brief   Updates and returns the pvt_data dictionary that contains the last received pvt data
 * @param   polling         If true, the pvt message is polled, else waits for the next navigation solution
 * @param   time_out_s      The maximum time to wait for the message
 * @return  A map containing the PVT data
 */
std::map<std::string, int32_t> SAM_M8Q::get_pvt(bool polling, int32_t time_out_s) {
}

