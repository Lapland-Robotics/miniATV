# This is miniATV Robot

## devrules
## dialout group
Add your username to dialout group so you don't need chmod every ttyUSBx port...
```
$ sudo adduser <your user name> dialout
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

## atv_setup ros package:
Move rest of atv_setup (folder) to yours ROS workspace source folder (assuming here ws is ~/catkin_ws/src).
```
$ cd catkin_ws/src
$ catkin_create_pkg atv_setup
$ cd ~/catkin_ws
$ catkin_make
```
