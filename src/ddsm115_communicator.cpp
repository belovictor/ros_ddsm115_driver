#include "ddsm115_communicator.hpp"

namespace ddsm115
{
/**
 * @brief Construct a new DDSM115Communicator::DDSM115Communicator object
 * 
 * @param port_name 
 */
DDSM115Communicator::DDSM115Communicator(std::string port_name)
{
  struct termios tty;

  port_name_ = port_name;
  ddsm115_state_ = DDSM115State::STATE_NORMAL;
  pthread_mutex_init(&port_mutex_, NULL);
  ROS_INFO("Opening serial port %s", port_name_.c_str());
  port_fd_ = open(port_name_.c_str(), O_RDWR);
  if (port_fd_ <= 0)
  {
    ROS_ERROR("Unable to open port %s", port_name_.c_str());
    ddsm115_state_ = DDSM115State::STATE_FAILED;
    return;
  }
  ROS_INFO("Getting tty attributes");
  if (tcgetattr(port_fd_, &tty) != 0)
  {
    ROS_ERROR("Unable to get attributes for port %s", port_name_.c_str());
    ddsm115_state_ = DDSM115State::STATE_FAILED;
    return;
  }
  tty.c_cflag &= ~PARENB; // No parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CSIZE; // Clear byte size bits
  tty.c_cflag |= CS8; // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS; // Disable CTS/RTS
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // cfmakeraw(&tty);
  tty.c_cc[VTIME] = 1;  // Read timeout
  tty.c_cc[VMIN] = 0;
  // Set baud to 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  ROS_INFO("Setting tty attributes");
  if (tcsetattr(port_fd_, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Error %i setting port attributes: %s\n", errno, strerror(errno));
    ddsm115_state_ = DDSM115State::STATE_FAILED;
    return;
  }
}
/**
 * @brief Disconnect from DDSM115
 * 
 */
void DDSM115Communicator::disconnect()
{
  close(port_fd_);
}

/**
 * @brief Set wheel mode
 * 
 * @param wheel_id 
 * @param mode 
 */
void DDSM115Communicator::setWheelMode(int wheel_id, DDSM115Mode mode)
{
  uint8_t mode_cmd[] = {
    (uint8_t)wheel_id, DDSM115Command::COMMAND_SWITCH_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode
  };
  lockPort();
  write(port_fd_, &mode_cmd, sizeof(mode_cmd));
  unlockPort();
}

/**
 * @brief Drive DDSM115 wheel and get it's feedback
 * 
 * @param wheel_id 
 * @param rpm 
 * @return ddsm115_drive_response 
 */
ddsm115_drive_response DDSM115Communicator::setWheelRPM(int wheel_id, double rpm)
{
  ddsm115_drive_response result;
  int16_t rpm_value = (int16_t)rpm;
  // TODO: this implementation of data encoding is not endian safe
  uint8_t drive_cmd[] = { (uint8_t)wheel_id,
                          DDSM115Command::COMMAND_DRIVE_MOTOR,
                          (uint8_t)((rpm_value >> 8) & 0xFF),
                          (uint8_t)(rpm_value & 0xFF),
                          0x00,
                          0x00,
                          0x00,
                          0x00,
                          0x00,
                          0x00 };
  uint8_t drive_response[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  drive_cmd[9] = maximCrc8(drive_cmd, 9);
  lockPort();
  write(port_fd_, &drive_cmd, sizeof(drive_cmd));
  // tcdrain(port_fd_);
  int total_num_bytes = 0;
  int num_bytes = 0;
  for (int i = 0; i < sizeof(drive_response); i++) {
    num_bytes = read(port_fd_, &drive_response[i], 1);
    if (num_bytes <= 0) {
      break;
    }
    total_num_bytes += num_bytes;
  }
  unlockPort();
  if (num_bytes < 0)
  {
    ROS_ERROR("Error reading DDSM115 response for wheel id %d", wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (total_num_bytes < 10)
  {
    ROS_WARN("Timeout reading DDSM115 response for wheel id %d", wheel_id);
    // ROS_INFO("Received %d bytes", total_num_bytes);
    // for (int i = 0; i < 10; i++) {
    //   ROS_INFO("%02x", drive_response[i]);
    // }
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (drive_response[0] != wheel_id)
  {
    ROS_INFO("Received response for wheel %d instead of %d", drive_response[0], wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (drive_response[9] != maximCrc8(drive_response, 9))
  {
    ROS_ERROR("CRC error in response from wheel id %d", wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  // TODO: this implementation of data decoding is not endian safe
  int16_t drive_current = 0;
  int16_t drive_velocity = 0;
  uint16_t drive_position = 0;
  uint8_t swap;
  swap = drive_response[4];
  drive_response[4] = drive_response[5];
  drive_response[5] = swap;
  memcpy(&drive_current, &drive_response[2], 2);
  memcpy(&drive_velocity, &drive_response[4], 2);
  memcpy(&drive_position, &drive_response[6], 2);
  result.velocity = (double)drive_velocity;
  result.position = (double)drive_position * (360.0 / 65535.0);
  result.current = (double)drive_current * (8.0 / 32767.0);
  result.result = DDSM115State::STATE_NORMAL;
  return result;
}

/**
 * @brief Lock communication mutex
 * 
 */
void DDSM115Communicator::lockPort()
{
  pthread_mutex_lock(&port_mutex_);
}

/**
 * @brief Unlock communication mutex
 * 
 */
void DDSM115Communicator::unlockPort()
{
  pthread_mutex_unlock(&port_mutex_);
}

/**
 * @brief Get DDSM115 communication state
 * 
 * @return DDSM115State 
 */
DDSM115State DDSM115Communicator::getState()
{
  return ddsm115_state_;
}

/**
 * @brief Maxim CRC8 calculator
 * 
 * @param data 
 * @param size 
 * @return uint8_t 
 */
uint8_t DDSM115Communicator::maximCrc8(uint8_t* data, const unsigned int size)
{
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i)
  {
    uint8_t inbyte = data[i];
    for (unsigned char j = 0; j < 8; ++j)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

}  // namespace ddsm115
