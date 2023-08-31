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
