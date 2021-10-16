## Scripts:

### atv_control_and_odom.py:
**Script for communication with ATV-interface:**

Subscribe and convert "Twist cmd" to "Ackermann cmd" (ATV_Conrtol) command and publish it for ATV-interface.

Subscribe "ATV_State" from ATV-interface and calculate and publish "odom".

### pub_waypoints.py:
**Script for communication with ATV-interface:**


### waypoint_navigation.py:
**Script for launch/start navigation via waypoint. Waypoints need to be publish first with pub_waypoints.py**


### atv_control_and_odom_simulator.py:
**Script for publish waypoints. Waypoints in file waypoints.csv in folder atv_setup/waypoints**


### atv_control_manual.py:
**Script for send constant steering and speed commands to ATV-interface:**

**Note! Used only for testing when drive motor is disconnected!**



