#include "ros_ddsm115_driver.hpp"
#include <signal.h>
#include "ddsm115_communicator.hpp"

// Array with wheel names
std::vector<std::string> wheel_names_list;
// Array with wheel DDSM115 ids
std::vector<int> wheel_ids_list;
// Array with wheel direction parameters
std::vector<std::string> wheel_directions_list;
// Array with target velocity subscribers
std::vector<ros::Subscriber> wheel_velocity_subs;
// Array with wheel velocity publishers
std::vector<ros::Publisher> wheel_velocity_pubs;
// Array with wheel angle publishers
std::vector<ros::Publisher> wheel_angle_pubs;
// Array with wheel current consumption publishers
std::vector<ros::Publisher> wheel_current_pubs;
// Array with wheel direction multipliers
std::vector<int> wheel_directions;
// DDSM115 communication interface
ddsm115::DDSM115Communicator* ddsm115_communicator;

bool intsAreDistinct(std::vector<int> arr);
bool stringsAreDistinct(std::vector<std::string> arr);
void onShutdown(int sig);
double vel2Rpm(double velocity);
double rpm2Vel(double rpm);
void wheelTargetVelocityCallback(const std_msgs::Float64::ConstPtr& target_velocity_msg, std::string wheel_name);
int wheelIndexByName(std::string wheel_name);

/**
 * @brief Check if ints in supplied array are distinct
 * 
 * @param arr 
 * @return true 
 * @return false 
 */
bool intsAreDistinct(std::vector<int> arr)
{
  std::unordered_set<int> s;
  for (int i = 0; i < (int)arr.size(); i++)
  {
    s.insert(arr[i]);
  }
  return (s.size() == arr.size());
}

/**
 * @brief Check if strings in supplied array are distinct
 * 
 * @param arr 
 * @return true 
 * @return false 
 */
bool stringsAreDistinct(std::vector<std::string> arr)
{
  int n = arr.size();
  std::unordered_set<std::string> s;
  for (int i = 0; i < n; i++)
  {
    s.insert(arr[i]);
  }
  return (s.size() == arr.size());
}

/**
 * @brief Handle shutdown
 * 
 * @param sig 
 */
void onShutdown(int sig)
{
  (void)sig;
  ROS_INFO("DDSM115 shutting down");
  // Stop all wheels
  for (int i = 0; i < wheel_ids_list.size(); i++)
  {
    ddsm115_communicator->setWheelRPM(wheel_ids_list[i], 0.0);
  }
  // Close DDSM115 communication
  ddsm115_communicator->disconnect();
  // Shut down the node
  ros::shutdown();
}

/**
 * @brief Convert rad/s velocity to RPM
 * 
 * @param velocity 
 * @return double 
 */
double vel2Rpm(double velocity)
{
  return velocity / (M_PI * 2) * 60;
}

/**
 * @brief Convert RPM to rad/s velocity
 * 
 * @param rpm 
 * @return double 
 */
double rpm2Vel(double rpm)
{
  return rpm / 60 * (M_PI * 2);
}

/**
 * @brief Call back for all wheels target velocity topics
 * 
 * @param target_velocity_msg 
 * @param wheel_name 
 */
void wheelTargetVelocityCallback(const std_msgs::Float64::ConstPtr& target_velocity_msg, std::string wheel_name)
{
  std_msgs::Float64 velocity_msg;
  std_msgs::Float64 angle_msg;
  std_msgs::Float64 current_msg;
  // ROS_INFO("I heard: [%f] for wheel \"%s\"", target_velocity_msg->data, wheel_name.c_str());
  int wheel_index = wheelIndexByName(wheel_name);
  if (wheel_index < 0)
  {
    ROS_WARN("Received target velocity for unknown wheel %s", wheel_name.c_str());
    return;
  }
  ddsm115::ddsm115_drive_response response = ddsm115_communicator->setWheelRPM(
      wheel_ids_list[wheel_index], vel2Rpm(target_velocity_msg->data) * wheel_directions[wheel_index]);
  if (response.result == ddsm115::DDSM115State::STATE_NORMAL)
  {
    velocity_msg.data = rpm2Vel(response.velocity) * wheel_directions[wheel_index];
    angle_msg.data = response.position * (2.0 * M_PI / 360.0) * wheel_directions[wheel_index];
    current_msg.data = response.current;
    wheel_velocity_pubs[wheel_index].publish(velocity_msg);
    wheel_angle_pubs[wheel_index].publish(angle_msg);
    wheel_current_pubs[wheel_index].publish(current_msg);
  }
}

/**
 * @brief Find wheel index by wheel name
 * 
 * @param wheel_name 
 * @return int 
 */
int wheelIndexByName(std::string wheel_name)
{
  std::vector<std::string>::iterator itr = std::find(wheel_names_list.begin(), wheel_names_list.end(), wheel_name);
  if (itr != wheel_names_list.end())
  {
    return std::distance(wheel_names_list.begin(), itr);
  }
  else
  {
    return -1;
  }
}

/**
 * @brief Main of the node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  std::string port_name;
  ros::init(argc, argv, "ros_ddsm115_driver_node");
  ros::NodeHandle node("~");
  ROS_INFO("DDSM115 starting up");
  // Check if we have all required parameters
  if (!node.hasParam("wheel_names"))
  {
    ROS_ERROR("Can't run without wheel_names parameter");
    return -1;
  }
  if (!node.hasParam("wheel_ids"))
  {
    ROS_ERROR("Can't run without wheel_ids parameter");
    return -1;
  }
  if (!node.hasParam("wheel_directions"))
  {
    ROS_ERROR("Can't run without wheel_directions parameter");
    return -1;
  }
  // Get all parameters
  node.param<std::string>("port_name", port_name, DEFAULT_SERIAL_PORT);
  node.getParam("wheel_names", wheel_names_list);
  node.getParam("wheel_ids", wheel_ids_list);
  node.getParam("wheel_directions", wheel_directions_list);

  // Check that wheel array parameters have equial length (specify all wheels)
  if (!(wheel_names_list.size() == wheel_ids_list.size() && wheel_ids_list.size() == wheel_directions_list.size()))
  {
    ROS_ERROR("wheel_names, wheel_ids and wheel_directions must be of an equal size");
    return -1;
  }
  // Check that wheel names and ids are distinct (can't run with 2 wheels with the same name or id)
  if (!stringsAreDistinct(wheel_names_list))
  {
    ROS_ERROR("wheel names must be distinct");
    return -1;
  }
  if (!intsAreDistinct(wheel_ids_list))
  {
    ROS_ERROR("wheel ids must be distinct");
    return -1;
  }

  // Open DDSM115 communication interface
  ddsm115_communicator = new ddsm115::DDSM115Communicator(port_name);
  if (ddsm115_communicator->getState() != ddsm115::DDSM115State::STATE_NORMAL)
  {
    ROS_ERROR("Failed to initialize DDSM115 communication");
    return -1;
  }

  // Do all setup for every wheel
  for (int i = 0; i < (int)wheel_names_list.size(); i++)
  {
    // Check that parameters are valid
    if (wheel_names_list[i].length() == 0)
    {
      ROS_ERROR("wheel name can't be empty");
      return -1;
    }
    if (wheel_ids_list[i] <= 0)
    {
      ROS_ERROR("wheel id must be >0");
      return -1;
    }
    ROS_INFO("Adding wheel %s id %d and direction %s", wheel_names_list[i].c_str(), wheel_ids_list[i],
             wheel_directions_list[i].c_str());
    // Create wheel direction multiplyers array
    if (wheel_directions_list[i] == "forward")
    {
      wheel_directions.push_back(1);
    }
    else
    {
      wheel_directions.push_back(-1);
    }
    /*
      Create wheel target velocity subscriber, bind with wheel name as additional parameter
      to handle all wheels with single callback
    */
    ros::Subscriber velocity_sub =
        node.subscribe<std_msgs::Float64>("/" + wheel_names_list[i] + "/target_velocity", 1,
                                          boost::bind(&wheelTargetVelocityCallback, _1, wheel_names_list[i]));
    wheel_velocity_subs.push_back(velocity_sub);
    // Create publisher for wheel velocity
    ros::Publisher velocity_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/current_velocity", 1);
    wheel_velocity_pubs.push_back(velocity_pub);
    // Create publisher for wheel angle
    ros::Publisher angle_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/angle", 1);
    wheel_angle_pubs.push_back(angle_pub);
    // Create publisher for wheel current consumption
    ros::Publisher current_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/current", 1);
    wheel_current_pubs.push_back(current_pub);
    // Set wheel mode to velocity loop
    ddsm115_communicator->setWheelMode(wheel_ids_list[i], ddsm115::DDSM115Mode::VELOCITY_LOOP);
  }
  // Add signal handler for safe shutdown of the node
  signal(SIGINT, onShutdown);
  // Run the node
  ros::spin();
  return 0;
}
