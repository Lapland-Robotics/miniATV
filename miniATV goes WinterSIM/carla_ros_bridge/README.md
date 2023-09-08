## carla_ros_bridge
All carla_ros_bridge files that have been changed for "miniATV goes WinterSIM".
Replace the original code in the opt/carla_ros_bridge/melodic folder
and modify the configuration files according to the instructions below.

#### Important! 
Use the debian instalation of the carla_ros_bridge instead of the GitHub installation.
The GitHub installation doesn't connect to the WinterSIM automatically and is a bit different than the debian version.
Follow the installation instructions here:
https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/#setting-the-ros-environment

### carla_ackermann_control_node.py
in opt/carla_ros_bridge/melodic/lib/carla_ackermann_control
Changes the ROS message type from AckermannDrive to AckermannDriveStamped to track any delays in the system with timestamps.
Modify AckermannControl.perspective in /opt/carla_ros_bridge/share/carla_ackermann_control as well by changing AckermannDrive to AckermannDriveStamped, or there will be a warning that the wrong ROS message type is used every time.

### gps_carla_ros_bridge_w_ego_vehicle.launch
can be in any launch file folder in opt/carla_opt/carla_ros_bridge/melodic/share, for example in .../carla_ros_bridge/launch. Launches carla_manual_control, carla_ackermann_control and an ego vehicle. Rosserial too, if the gps sensor is connected to the laptop instead of the miniATV for testing - usually rosserial and carla's simulation time do not work together at all, maybe it does in this case because use_sim_time is not set to True in the launch file itself?

### carla_manual_control.py
replaces /opt/carla_ros_bridge/melodic/lib/carla_manual_control/carla_manual_control.py
