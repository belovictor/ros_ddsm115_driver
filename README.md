# ros_ddsm115_driver

## Overviewf

ROS driver for the [Waveshare DDSM115 motor wheel](https://www.waveshare.com/ddsm115.htm) connected through RS485 bus. Right now driver only supports velocity control mode as this was my prime area of interest for my own project but it can be extended to provide support for other modes of control.

## Usage

### Getting the code

To use the driver just check it out to your ROS workspace:
```
git clone https://github.com/belovictor/ros_ddsm115_driver.git
```

### Configuration

Driver is configured with the following configuration paramters:

| Parameter name         | Type     | Description                                                                        |
|------------------------|----------|------------------------------------------------------------------------------------|
| ```port_name```        | String   | RS485 serial port device name                                                      |
| ```wheel_names```      | String[] | Array of names of wheels which are then used to name topics corresponding to wheels|
| ```wheel_ids```        | Int[]    | Array of wheel ids as they are configured in DDSM115 for every wheel               |
| ```wheel_directions``` | String[] | Wheel directions either ```forward``` or ```backward```, ```backward``` reverses wheel control, angle and current velocity |

An example configuration for a 4 wheel robot is provided in config/ros_ddsm115_driver_config.yaml. Every wheel have to have a uniq name and uniq ID.
You can get all information needed to connect wheels and configure their IDs in [Waveshare documentation](https://www.waveshare.com/wiki/DDSM115)

### Running the driver

```
roslaunch ros_ddsm115_driver ros_ddsm115_driver.launch
```

### How it works?

The driver communicates with a set of DDSM115 motor wheels, connected to a RS485 bus, as specified in configuration, and exposes every wheel as a set of topics to ROS.

| Topic name                         | Message type           | Direction | Description                             |
|------------------------------------|------------------------|-----------|-----------------------------------------|
|```<wheel name>/angle```            | ```std_msgs/Float64``` | Outbound  | Angle of the wheel in rad               |
|```<wheel name>/current```          | ```std_msgs/Float64``` | Outbound  | Wheel current consumption               |
|```<wheel name>/current_velocity``` | ```std_msgs/Float64``` | Outbound  | Current wheel angular velocity in rad/s |
|```<whee; name>/target_velocity```  | ```std_msgs/Float64``` | Inbound   | Target wheel angular velocity in rad/s  |

So you can control every wheel by setting it's target velocity and get feedback through corresponding topics.

### Important to know

From time to time communication errors can happen on RS485 bus which lead to missing of single control messages from driver to wheels. From my experience this does not really affect robot operation, at least at 10Hz control rate.

DDSM115 motor wheels does not have built in safety mechanisms for loose of control, so if control flow stops, wheels will be running according to the last command they received. To partially fix it, driver node sends zero RPM command to stop all wheels on shutdown, but this will not cover all possible scenarios of loosing the control flow between ROS and motor wheels. As an example, if RS485 hardware/bus fails, wheels will continue to run according to the last command received.
