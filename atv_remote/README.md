# This is for remote control and monitoring miniATV Robot

## Home
Move Home folder content to Home folder!

## Launch icon. desktop_apps(Linux "desktop" -launchs)
Remember edit paths (change username) in *.desktop files.
Move *.desktops to paht: /usr/share/applications.

## Installation
1. Install sshpass
```
$ sudo apt-get install sshpass
```
2. Take ssh to miniATV (atv_setup) (this update keys). Current setup: hk@172.16.200.200 (password = ask it from hkblo). Close connection after connection establishment.
```
$ ssh <username>@<x.x.x.x> # x.x.x.x = ip address in  miniATV (atv_setup)
```
3. In "ip_addresses.sh", check IP -addresses, username and password.
4. Add "ATVremote" launch icon to "Favorites" ("Softwares" and right click then "Add to Favorites"....)
5. Add "WaypointsRemote" launch icon to "Favorites" ("Softwares" and right click then "Add Favorites"....)


### ROS Installation (Copy from ros.org)
[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

Configure your Ubuntu repositories repositories to allow "restricted," "universe," and "multiverse.":
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

ROS Desktop Install: ROS, rqt, rviz, and robot-generic libraries
```
$ sudo apt install ros-melodic-desktop
```
Source bash...
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

## atv_remote ros package (Skip this):

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
### Create atv_remote package
```
$ cd catkin_ws/src
$ catkin_create_pkg atv_remote
```
Move rest of atv_remote folder contents to yours atv_remote folder in ROS workspace source folder (assuming here ws is ~/catkin_ws/atv_remote).
```
$ cd ~/catkin_ws
$ catkin_make
```
# How to remotely control miniATV....

### Launch ATV:
Click the ATVremote icon. <img src="https://user-images.githubusercontent.com/90048225/133504302-29b554fb-10eb-4bc9-a2b2-1b9463db9e3a.png" width="80" height="80">

### Launch Waypoints navigation:
Click the WaypointsRemote icon. <img src="https://user-images.githubusercontent.com/90048225/133504339-196b28fc-0f99-4c89-acb5-9ee2cd2886a5.png" width="80" height="80">
