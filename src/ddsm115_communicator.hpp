#ifndef DDSM115_COMMUNICATOR_HPP_
#define DDSM115_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <pthread.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

namespace ddsm115
{
enum DDSM115State
{
  STATE_NORMAL = 0x01,
  STATE_FAILED = 0x02
};

enum DDSM115Mode
{
  CURRENT_LOOP = 0x01,
  VELOCITY_LOOP = 0x02,
  POSITION_LOOP = 0x03
};

enum DDSM115Error
{
  SENSOR_ERROR = 0x01,
  OVERCURRENT_ERROR = 0x02,
  PHASE_OVERCURRENT_ERROR = 0x04,
  STALL_ERROR = 0x08,
  TROUBLESHOOTING_ERROR = 0x10
};

enum DDSM115Command
{
  COMMAND_DRIVE_MOTOR = 0x64,
  COMMAND_GET_OTHER_FEEDBACK = 0x74,
  COMMAND_SWITCH_MODE = 0xA0,
  COMMAND_SET_ID = 0x55,
  COMMAND_QUERY_ID = 0x64
};

struct ddsm115_drive_response
{
  uint8_t mode;
  double current;
  double velocity;
  double position;
  uint8_t error;
  DDSM115State result;
};

/**
 * @brief A class providing a communication interface to DDSM115 wheels
 * 
 */
class DDSM115Communicator
{
public:
  DDSM115Communicator(std::string port_name);
  void disconnect();
  void setWheelMode(int wheel_id, DDSM115Mode mode);
  ddsm115_drive_response setWheelRPM(int wheel_id, double rpm);
  DDSM115State getState();

private:
  std::string port_name_;
  int port_fd_;
  pthread_mutex_t port_mutex_;
  DDSM115State ddsm115_state_;
  void lockPort();
  void unlockPort();
  uint8_t maximCrc8(uint8_t* data, const unsigned int size);
};
}  // namespace ddsm115

#endif  // DDSM115_COMMUNICATOR_HPP_
