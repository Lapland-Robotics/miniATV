## devrules
### Rename ESP32 USB-port:
1. Read serial number (ATTRS{serial}) 
```
$ udevadm info --name=/dev/ttyUSBx --attribute-walk # USBx = where ESP32 is connected
```
2. Replace ATTRS{serial} with yours ESP32 serial number in 99-esp32-usb.rules
3. Move 99-esp32-usb.rules to path /etc/udev/rules.d/

### Rename RPLidar USB-port:
Move 99-rplidar-usb.rules to path /etc/udev/rules.d/
