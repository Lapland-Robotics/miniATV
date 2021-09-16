### ATV Simulator

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

Desktop Install: ROS, rqt, rviz, and robot-generic libraries
```
$ sudo apt install ros-melodic-desktop
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
$ sudo apt install python-rosdep
$ sudo rosdep init
$ rosdep update
```

### Make ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Source bash...
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Create atv_simulator ros package:

```
$ cd catkin_ws/src
$ catkin_create_pkg atv_simulator
```
Move atv_simulator folder contents to yours atv_simulator folder in ROS workspace source folder (assuming here ws is ~/catkin_ws/src/atv_simulator).
```
$ cd ~/catkin_ws
$ catkin_make
```

# Note! If waypoints needed: Clone [Waypoints](https://github.com/Lapland-Robotics/Waypoints).

## Launch Simulator
```
$ roslaunch atv_simulator atv_in_stage.launch
```

## Troubleshooter:
If some:
```
ERROR: cannot launch node of type [map_server/map_server]: map_server
```
than:
```
sudo apt-get install ros-melodic-map-server
```
NOTE! MISSING "map_server", BUT INSTALLATION PACKAGE IS "map-server"

