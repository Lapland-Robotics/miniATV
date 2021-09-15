# This is for remote control and monitoring miniATV Robot

## Home
Move Home folder content to Home folder!
Chek

## desktop_apps(Linux "desktop" -launchs)
Remember edit paths (change username) in *.desktop files.
Move *.desktops to paht: /usr/share/applications.


## atv_remote ros package:
Move rest of atv_remote (folder) to yours ROS workspace source folder (assuming here ws is ~/catkin_ws/src).
```
$ cd catkin_ws/src
$ catkin_create_pkg atv_remote
$ cd ~/catkin_ws
$ catkin_make
```
