# This is for remote control and monitoring miniATV Robot

## Home
Move Home folder content to Home folder!

## desktop_apps(Linux "desktop" -launchs)
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
4. Add "ATVremote" launch icon to "Favorites" ("All Softwares" and right click then "Add to Favorites"....)
5. Add "WaypointsRemote" launch icon to "Favorites" ("All Softwares" and right click then "Add to Favorites"....)

## atv_remote ros package:
Move rest of atv_remote (folder) to yours ROS workspace source folder (assuming here ws is ~/catkin_ws/src).
```
$ cd catkin_ws/src
$ catkin_create_pkg atv_remote
$ cd ~/catkin_ws
$ catkin_make
```

# How to remotely control miniATV....
Laucnh ATV: Click the ATVremote icon.
Laucnh Waypoints navigation: Click the WaypointsRemote icon.
