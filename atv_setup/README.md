# This is miniATV Robot

## At the moment
- nVidia Jetson Nano (JetPack 4.5) with RPLidar
- ROS Melodic and teb_local_planner (AMCL) on remote launch...

## Installation
### ROS Installation (Copy from ros.org)
[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

Configure your Ubuntu repositories
[https://help.ubuntu.com/community/Repositories/Ubuntu](https://help.ubuntu.com/community/Repositories/Ubuntu)

Setup your computer to accept software from packages.ros.org.
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Set up your keys
```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Make sure your Debian package index is up-to-date:
sudo apt update
```
$ sudo apt update        # Fetches the list of available updates
$ sudo apt upgrade       # Installs some updates; does not remove packages
$ sudo apt full-upgrade  # Installs updates; may also remove some packages, if needed
$ sudo apt autoremove    # Removes any old packages that are no longer needed
```

ROS Desktop-Full Install: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
```
$ sudo apt install ros-melodic-desktop-full
```
Source bash...
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
### Install dependencies for building ROS packages, run:
```
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
Initialize rosdep:
```
$ sudo rosdep init
$ rosdep update
```

### Make ROS workspace
Open new Terminal window.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Source bash...
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### teb_local_planner
[http://wiki.ros.org/teb_local_planner](http://wiki.ros.org/teb_local_planner)
Install the teb_local_planner package from the official ROS repositories:
```
$ sudo apt-get install ros-melodic-teb-local-planner
```
If you want to get familiar with the teb_local_planner then:
[http://wiki.ros.org/teb_local_planner/Tutorials](http://wiki.ros.org/teb_local_planner/Tutorials)
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
```
teb_local_planner_tutorial use Stage -simulator so:
```
$ sudo apt-get install ros-melodic-stage-ros
```



### ROS serial for UART (serial-port) communication (here with ESP32)
```
sudo apt-get install ros-melodic-rosserial
```

### RPLIDAR laser Lidar
[http://wiki.ros.org/rplidar](http://wiki.ros.org/rplidar)
Source: [https://github.com/Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros)
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Slamtec/rplidar_ros
$ cd ~/catkin_ws
$ catkin_make
```

## devrules
## dialout group
Add your username to dialout group so you don't need chmod every ttyUSBx port...
```
$ sudo adduser <your username> dialout
```

### Rename ESP32 USB-port:
1. Read serial number (ATTRS{serial}) 
```
$ udevadm info --name=/dev/ttyUSBx --attribute-walk # USBx = where ESP32 is connected
```
2. Replace ATTRS{serial} with yours ESP32 serial number in 99-esp32-usb.rules
3. Move 99-esp32-usb.rules to path /etc/udev/rules.d/

### Rename RPLidar USB-port:
Move 99-rplidar-usb.rules to path /etc/udev/rules.d/

***REBOOT SYSTEM (Just in case...)

## Create atv_setup ros package:

```
$ cd catkin_ws/src
$ catkin_create_pkg atv_setup
```
Move atv_setup folder contents to yours atv_setup folder in ROS workspace source folder (assuming here ws is ~/catkin_ws/atv_setup).
```
$ cd ~/catkin_ws
$ catkin_make
```

## Troubleshooter:
### NOTE! SOME REASON ros FULL IS NOT FULL INSTALLATION. OR I'M JUST NOOB.
YOU HAVE TO INSTALL SEVERAL PACKAGES AT TIME WHEN YOU NEED THEM (map_server, rosserial etc...).
Example
If some:
```
ERROR: cannot launch node of type [map_server/map_server]: map_server
```
than:
```
sudo apt-get install ros-melodic-map-server
```
NOTE! MISSING "map_server", BUT INSTALLATION PACKAGE IS "map-server"

If some:
```
***Failed to create the global_planner/GlobalPlanner********
```
than:
```
sudo apt-get install ros-melodic-global-planner
```

If some:
```
ImportError: No module named ackermann_msgs.msg
```
than:
```
sudo apt-get install ros-melodic-ackermann-msgs
```
