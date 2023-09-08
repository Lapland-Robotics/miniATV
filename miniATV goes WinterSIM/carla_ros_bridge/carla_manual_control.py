#!/usr/bin/env python2
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

import datetime
import math
import numpy

import carla

import rospy
import rostopic
import tf
#import carla_ros_bridge.transforms as transforms
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from std_msgs.msg import String
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaStatus

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    """
    Handle the rendering
    """

    def __init__(self, role_name, hud):
        self._surface = None
        self.hud = hud
        self.role_name = role_name

        # CS: Test: draw points at spawn point 
        # Connecting to the server
        client = carla.Client('localhost', 2000)
        self.world = client.get_world()
        client.set_timeout(10.0)
        self.map = self.world.get_map()
        print()
        print(self.map.transform_to_geolocation(carla.Location(0,0,0)))
        self.origin_geolocation = self.map.transform_to_geolocation(carla.Location(0,0,0)) 
        print()
        print("map origin in geodetic ")
        print(self.origin_geolocation)


        print("WinterSIM weather: " + str(self.world.get_weather()))

        spawn_points = self.map.get_spawn_points()
        self.spawn_point_location = spawn_points[0].location
        for point in spawn_points:
            self.spawn_point_location = point.location
            life_time = -1 # -1 is forever, TODO: How to erease the last point on arrival of the newest point
            #self.world.debug.draw_point(self.spawn_point_location, size=1, color=carla.Color(0, 255, 0), life_time=0)

        

        self.image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/view/image_color".format(self.role_name),
            Image, self.on_view_image)
        self.collision_subscriber = rospy.Subscriber(
            "/carla/{}/collision".format(self.role_name), CarlaCollisionEvent, self.on_collision)
        self.lane_invasion_subscriber = rospy.Subscriber(
            "/carla/{}/lane_invasion".format(self.role_name),
            CarlaLaneInvasionEvent, self.on_lane_invasion)
        
        # GPS position of the real miniATV
        self.real_gps_pos_subscriber = rospy.Subscriber(
            "/atv_gps", NavSatFix, self.cb_display_real_gps_position)
        # Frequency of gps messages for securtiy 
        # and seamlessly displaying the real miniATV's gps position with draw_point, which has a set live time but cannot be deleted early
        self.real_gps_hz = rostopic.ROSTopicHz(-1)
        self.real_gps_hz_subscriber = rospy.Subscriber(
            "/atv_gps", NavSatFix, self.real_gps_hz.callback_hz, callback_args="/atv_gps")
        
        # GPS position of the digital twin miniATV
        self.dt_gnss_subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/gnss/gnss1/fix", NavSatFix, self.cb_dt_gnss)
        # Monitor rate of the digital twin's GNSS topic for comparisons
        self.dt_gnss_hz = rostopic.ROSTopicHz(-1)
        self.dt_gnss_hz_subscriber = rospy.Subscriber(
            "/carla/ego_vehicle/gnss/gnss1/fix", NavSatFix, self.dt_gnss_hz.callback_hz, callback_args="/carla/ego_vehicle/gnss/gnss1/fix")
        
        # Subscribe to ouster lidar IMU 
        self.ouster_imu_subscriber = rospy.Subscriber("/ouster/imu", Imu, self.cb_pt_imu)
        self.pt_imu_hz = rostopic.ROSTopicHz(-1)
        self.pt_imu_hz_subscriber = rospy.Subscriber(
            "/ouster/imu", Imu, self.pt_imu_hz.callback_hz, callback_args="/ouster/imu")
        
        #TODO:
        # Publish the distances between positions as an easily recordable ros topic
        self.control_info_publisher = rospy.Publisher(
            "distance_info", String, queue_size=1)
        
         # TODO: In case of more than one digital miniATV: Adapt this
         # TODO: world is not yet available, and spawning takes time, wait for world to be ready?
        self.dt_miniATV = self.world.get_actors().filter('vehicle.miniatv.miniatv')[0] # should be spawned by the time manual_control gets launched
        print(self.dt_miniATV)
        # teleport vehicle to starting position of the rosbag
        # TODO: set new spawnpoint for starting position of rosbag(s)
        self.dt_miniATV_start_loc = self.geodetic_gnss_to_carla_location(66.48072052, 25.7217960358, 79.8890151978) #lat, lon, alt
        print("teleport dt miniATV to start position ")
        print(self.dt_miniATV_start_loc)
        self.dt_miniATV.set_location(self.dt_miniATV_start_loc)




    def cb_pt_imu(self, data):
        # Ouster lidar should be using the right-hand coordinate system, also called ENU (East-North-Up)
        # Carla is using ESU or left-hand coordinate system which can be translated to E-NU
        # TODO: When testing this live with the lidar, one of the directions is exactle the other was round
        # TODO: Find out which one and put a minus in front of that, it's probably the missing ENU to E-NU/ESU translation

        #Sources
        #https://carla.readthedocs.io/en/0.9.8/python_api/#carlarotation # roll, pitch yaw should be in degree
        # ROS IMU message: rotational velocity should be in rad/sec

        # Radians to degree: https://www.wikihow.com/Convert-Radians-to-Degrees 
        x_deg_sec = data.angular_velocity.x * (180/math.pi) # [deg/s] and still probably ENU
        y_deg_sec = data.angular_velocity.y * (180/math.pi) # [deg/s] and still probably ENU
        z_deg_sec = data.angular_velocity.z * (180/math.pi) # [deg/s] and still probably ENU

        print("angular velocity in degree per s: " + str(x_deg_sec))

        # How to get from [deg/s] to just degree?
        # If it's x degree in one second and the measurements are y seconds apart, 
        # than it turns x [deg/s] / #measurements/y seconds or x/Hz of topic

        # Get the frequency of the imu topic to transfer the values to just degrees
        try:
            pt_imu_topic_hz = self.pt_imu_hz.get_hz("/ouster/imu")[0]
            print("Hz of pt_imu topic: ")
            print(pt_imu_topic_hz)
        except:
            print()
            pt_imu_topic_hz = 236 #Approximation for whenever a measurement doesn't come through base on Ouster lidar imu topic frequency
        draw_life_time = 1/pt_imu_topic_hz # Only for drawing a shape from world.debug.draw....

        # carla.Rotation
        # pitch (float) Y rotation in degrees
        # yaw (float) Z rotation in degrees.
        # roll (float) X rotation in degrees.

        y_pitch = y_deg_sec / pt_imu_topic_hz # [deg] (hopefully) and definitely still in ENU
        x_roll = x_deg_sec / pt_imu_topic_hz # [deg] (hopefully) and definitely still in ENU
        z_yaw = z_deg_sec / pt_imu_topic_hz # [deg] (hopefully) and definitely still in ENU

        #delta rotation, because it is the change in rotation until the next incoming message
        pt_delta_rotation = carla.Rotation(pitch = y_pitch, yaw = z_yaw, roll = x_roll)
        print(pt_delta_rotation)
        pt_current_rotation = self.dt_miniATV.get_transform().rotation
        pt_current_location = self.dt_miniATV.get_transform().location

        new_pitch = pt_current_rotation.pitch + y_pitch
        new_yaw = pt_current_rotation.yaw + z_yaw
        new_roll = pt_current_rotation.roll + x_roll
        new_rotation = carla.Rotation(pitch = new_pitch, yaw = new_yaw, roll = new_roll)

        # test it on the miniATV, is it changing it's orientation accordingly?
        self.dt_miniATV.set_transform(carla.Transform(pt_current_location, new_rotation))

        # That did not work?
        #self.world.debug.draw_box(carla.BoundingBox(carla.Location(self.dt_miniATV_start_loc), carla.Vector3D(0.5,0.5,0.3)), carla.Rotation(pitch = y_pitch, yaw = z_yaw, roll = x_roll), 0.05, carla.Color(255,0,0,0), -1)



     



    def cb_dt_gnss(self, data):
        """
        Callback for the digital twin miniATV's GNSS sensor
        """
        # Get the header with the timestamp for comparisons
        self.dt_gnss_header = data.header
        # Update the carla position of the digital twin's GNSS sensor
        dt_gnss_lat = data.latitude
        dt_gnss_lon = data.longitude
        dt_gnss_alt = data.altitude
        self.dt_gnss_location = self.geodetic_gnss_to_carla_location(dt_gnss_lat, dt_gnss_lon, dt_gnss_alt)

        # For comparisons with the real miniATV's GNSS topic:
        try:
            gps_topic_hz = self.dt_gnss_hz.get_hz("/carla/ego_vehicle/gnss/gnss1/fix")[0]
            draw_life_time = 1/gps_topic_hz
            print("Hz of dt_gnss topic: ")
            print(draw_life_time)
            print("dt_gnss hz rate " + str(gps_topic_hz))
        except:
            print()
    
    
    def cb_display_real_gps_position(self, data): # CS
        """ 
        Callback for the 
        """
        # Input from the miniATV's GNSS sensor: geodetic,WGS84
        miniATV_gnss_lat = data.latitude
        miniATV_gnss_lon = data.longitude
        # miniATV_gnss_alt = data.altitude # rosbag is using other sea level computation than carla origin, hardcoding that for testing with rosbag
        miniATV_gnss_alt = 79.0000 #[m] Mean Sea level
        miniATV_gnss_hor_acc = data.position_covariance[0]
        
        #lat_lon_convert_to_carla_gps = numpy.array([miniATV_gnss_lat, miniATV_gnss_lon])
        #print("Before GPS: Lat ")
        #print(lat_lon_convert_to_carla_gps[0])
        #print(" Lon ")
        #print(lat_lon_convert_to_carla_gps[1])
        miniATV_location = self.geodetic_gnss_to_carla_location(miniATV_gnss_lat, miniATV_gnss_lon, miniATV_gnss_alt)
        #miniATV_pos = [carla_lat_lon[0], carla_lat_lon[1], miniATV_gnss_alt]
        
        print("miniATV carla coordinates ")
        print(miniATV_location)

        print()
        print("map origin in geodetic ")
        
        print(self.origin_geolocation)
        print("type of origin geolocation ")
        print(type(self.origin_geolocation))

        print()
        print("GNSS horizontal accuracy: ")
        print(miniATV_gnss_hor_acc)
        #carla_gps = 
        # Size ??? Is it radius or diameter???
        # SI unit: probably meter since meter and m/s are the standard units of CARLA
        
        
        #print("list ")
        #print(self.dt_miniATV)
        #print(self.world.get_actors().filter('vehicle.*'))
        dt_miniATV_location = self.dt_miniATV.get_transform().location
        #print()
        #print("digital miniATV pos: ")
        #print(self.dt_miniATV.get_transform().location)
        #print()
        
        try:
            gps_topic_hz = self.hz.get_hz("/atv_gps")[0]
            draw_life_time = 1/gps_topic_hz
            print("Hz of gps topic: ")
            print(draw_life_time)
            print("gps hz rate " + str(gps_topic_hz))
        except:
            draw_life_time = 0.1 # /atv_gps has a 10 Hz rate from the Arduino side, so this is a good approximation when lacking the actual rate
        
        if miniATV_gnss_hor_acc < 2: #initially, the horizontal accuracy can be more than 10 m, in that case do not obstruct the whole screen
            pt_point_size = miniATV_gnss_hor_acc
        else:
            pt_point_size = 2
        #self.world.debug.draw_point(miniATV_location, size=pt_point_size, color=carla.Color(255, 0, 0), life_time=draw_life_time)
        
        #self.dt_miniATV.set_location(miniATV_location) # Just for fun: Teleports dt_miniATV to the gps positions but without regard to orientation
        #self.world.debug.draw_line(miniATV_location, dt_miniATV_location, thickness=0.05, color=carla.Color(255,0,0), life_time=draw_life_time)

        print(miniATV_location - dt_miniATV_location)
        #euclidean = math.dist(miniATV_location, dt_miniATV_location)
        horizontal_euclidean = numpy.linalg.norm(numpy.array((miniATV_location.x, miniATV_location.y)) - numpy.array((dt_miniATV_location.x, dt_miniATV_location.y)))
        vertical_euclidean = numpy.linalg.norm(miniATV_location.z - dt_miniATV_location.z)
        print("Euclidean distance hor " + str(horizontal_euclidean))
        print("Euclidean distance vert " + str(vertical_euclidean))

        # Test if the conversion from geodetic gps to carla location is sucessfull
        #origin_geolocation


    def geodetic_gnss_to_carla_location(self, lat, lon, alt):
        """
        Source: https://github.com/carla-simulator/carla/issues/2737 with thanks to Kait0
        Converts GPS signal (x left/west/latitude, y front/north/longitude, z up/altitude) => right-handed coordinates
        into the standard ??? CARLA coordinate frame (x front/north?/longitude, y right/east, z up) => left-hand system?????

        Carla GeoLocation:
        https://carla.readthedocs.io/en/latest/python_api/#carla.GeoLocation
        

        https://github.com/carla-simulator/carla/issues/2737 # How to convert gps  to carla location


        :param gps: gps from gnss sensor
        :return: gps as numpy array in CARLA coordinates
        """

        # geodetic (GNSS output) to enu (East, North, Up)

        # Origin of Rantavitikka map as reference point:
        O_lat = self.origin_geolocation.latitude
        O_lon = self.origin_geolocation.longitude
        O_alt = self.origin_geolocation.altitude
        print()
        print("Exact lat, lon, alt of Rantavitikka origin ")
        print(O_lat)
        print(O_lon)
        print(O_alt)
        print()
        x_enu, y_enu, z_enu = self.geodetic_to_enu(lat, lon, alt, O_lat, O_lon, O_alt)

        carla_location = carla.Location(x_enu, -y_enu, z_enu)



        #Approximation: #gps = (gps - numpy.array([0.0, 0.0])) * numpy.array([111324.60662786, 111319.490945])
        #gps = numpy.array([gps[1], -gps[0]]) # convert from ENU (east north up) to ESU (east south up) that is used in carla
        return carla_location
    
        # https://gist.github.com/sbarratt/a72bede917b482826192bf34f9ff5d0b # convert different coordinate systems to each other

    def geodetic_to_ecef(self, lat, lon, h):
        #https://github.com/carla-simulator/carla/issues/2737

        a = 6378137 # earth radius for WSG-84
        b = 6356752.3142
        f = (a - b) / a
        e_sq = f * (2-f)

        # (lat, lon) in WSG-84 degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = a / math.sqrt(1 - e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - e_sq) * N) * sin_lambda

        return x, y, z

    def ecef_to_enu(self, x, y, z, lat0, lon0, h0):
        # https://github.com/carla-simulator/carla/issues/2737

        a = 6378137
        b = 6356752.3142
        f = (a - b) / a
        e_sq = f * (2-f)

        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = a / math.sqrt(1 - e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - e_sq) * N) * sin_lambda

        xd = x - x0
        yd = y - y0
        zd = z - z0

        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

        return xEast, yNorth, zUp

    def geodetic_to_enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
        # https://github.com/carla-simulator/carla/issues/2737

        x, y, z = self.geodetic_to_ecef(lat, lon, h)
        
        return self.ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)



    def on_collision(self, data):
        """
        Callback on collision event
        """
        intensity = math.sqrt(data.normal_impulse.x**2 +
                              data.normal_impulse.y**2 + data.normal_impulse.z**2)
        self.hud.notification('Collision with {} (impulse {})'.format(
            data.other_actor_id, intensity))

    def on_lane_invasion(self, data):
        """
        Callback on lane invasion event
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown ")
        self.hud.notification('Crossed line %s' % ' and '.join(text))

    def on_view_image(self, image):
        """
        Callback when receiving a camera image
        """
        array = numpy.frombuffer(image.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        """
        render the current image
        """
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
        self.hud.render(display)

    def destroy(self):
        """
        destroy all objects
        """
        self.image_subscriber.unregister()
        self.collision_subscriber.unregister()
        self.lane_invasion_subscriber.unregister()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, hud):
        self.role_name = role_name
        self.hud = hud

        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

        self.vehicle_control_manual_override_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, queue_size=1, latch=True)
        self.vehicle_control_manual_override = False
        self.auto_pilot_enable_publisher = rospy.Publisher(
            "/carla/{}/enable_autopilot".format(self.role_name), Bool, queue_size=1)
        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            CarlaEgoVehicleControl, queue_size=1)
        self.carla_status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self._on_new_carla_frame)

        self.set_autopilot(self._autopilot_enabled)

        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override)  # disable manual override

    def __del__(self):
        self.auto_pilot_enable_publisher.unregister()
        self.vehicle_control_publisher.unregister()
        self.vehicle_control_manual_override_publisher.unregister()

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.hud.notification('Set vehicle control manual override to: {}'.format(enable))
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    # pylint: disable=too-many-branches
    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and
                                          pygame.key.get_mods() & KMOD_SHIFT):
                    self.hud.help.toggle()
                elif event.key == K_b:
                    self.vehicle_control_manual_override = not self.vehicle_control_manual_override
                    self.set_vehicle_control_manual_override(self.vehicle_control_manual_override)
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self.hud.notification('%s Transmission' % (
                        'Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.set_autopilot(self._autopilot_enabled)
                    self.hud.notification('Autopilot %s' % (
                        'On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
        return False

    def _on_new_carla_frame(self, _):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            try:
                self.vehicle_control_publisher.publish(self._control)
            except rospy.ROSException as error:
                rospy.logwarn("Could not send vehicle control: {}".format(error))

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """
    Handle the info display
    """

    def __init__(self, role_name, width, height):
        self.role_name = role_name
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self._show_info = True
        self._info_text = []
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.tf_listener = tf.TransformListener()
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(self.role_name),
            CarlaEgoVehicleStatus, self.vehicle_status_updated)
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle_info_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_info".format(self.role_name),
            CarlaEgoVehicleInfo, self.vehicle_info_updated)
        self.latitude = 0
        self.longitude = 0
        self.manual_control = False
        self.gnss_subscriber = rospy.Subscriber(
            "/carla/{}/gnss/gnss1/fix".format(self.role_name), NavSatFix, self.gnss_updated)
        self.manual_control_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, self.manual_control_override_updated)

        self.carla_status = CarlaStatus()
        self.status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self.carla_status_updated)

    def __del__(self):
        self.gnss_subscriber.unregister()
        self.vehicle_status_subscriber.unregister()
        self.vehicle_info_subscriber.unregister()

    def tick(self, clock):
        """
        tick method
        """
        self._notifications.tick(clock)

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data
        self.update_info_text()

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle status updates
        """
        self.manual_control = data.data
        self.update_info_text()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.vehicle_status = vehicle_status
        self.update_info_text()

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback on vehicle info updates
        """
        self.vehicle_info = vehicle_info
        self.update_info_text()

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.update_info_text()

    def update_info_text(self):
        """
        update the displayed info text
        """
        if not self._show_info:
            return
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(
                '/map', self.role_name, rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -math.degrees(yaw)
            x = position[0]
            y = -position[1]
            z = position[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            z = 0
            yaw = 0
        heading = 'N' if abs(yaw) < 89.5 else ''
        heading += 'S' if abs(yaw) > 90.5 else ''
        heading += 'E' if 179.5 > yaw > 0.5 else ''
        heading += 'W' if -0.5 > yaw > -179.5 else ''
        fps = 0
        if self.carla_status.fixed_delta_seconds:
            fps = 1 / self.carla_status.fixed_delta_seconds
        self._info_text = [
            'Frame: % 22s' % self.carla_status.frame,
            'Simulation time: % 12s' % datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec())),
            'FPS: % 24.1f' % fps,
            '',
            'Vehicle: % 20s' % ' '.join(self.vehicle_info.type.title().split('.')[1:]),
            'Speed:   % 15.0f km/h' % (3.6 * self.vehicle_status.velocity),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (x, y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (self.latitude, self.longitude)),
            'Height:  % 18.0f m' % z,
            '']
        self._info_text += [
            ('Throttle:', self.vehicle_status.control.throttle, 0.0, 1.0),
            ('Steer:', self.vehicle_status.control.steer, -1.0, 1.0),
            ('Brake:', self.vehicle_status.control.brake, 0.0, 1.0),
            ('Reverse:', self.vehicle_status.control.reverse),
            ('Hand brake:', self.vehicle_status.control.hand_brake),
            ('Manual:', self.vehicle_status.control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(self.vehicle_status.control.gear,
                                                      self.vehicle_status.control.gear),
            '']
        self._info_text += [('Manual ctrl:', self.manual_control)]
        if self.carla_status.synchronous_mode:
            self._info_text += [('Sync mode running:', self.carla_status.synchronous_mode_running)]
        self._info_text += ['', '', 'Press <H> for help']

    def toggle_info(self):
        """
        show/hide the info text
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        display a notification for x seconds
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        display an error
        """
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """
        render the display
        """
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1])) # pylint: disable=too-many-function-args
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30)
                                  for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset + 50, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            f = 0.0
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """
    Support Class for info display, fade out text
    """

    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim) # pylint: disable=too-many-function-args

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        set the text
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim) # pylint: disable=too-many-function-args
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        """
        tick for fading
        """
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """
        render the fading
        """
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """
    Show the help text
    """

    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim) # pylint: disable=too-many-function-args
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """
        Show/hide the help
        """
        self._render = not self._render

    def render(self, display):
        """
        render the help
        """
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node('carla_manual_control', anonymous=True)

    role_name = rospy.get_param("~role_name", "ego_vehicle")

    # resolution should be similar to spawned camera with role-name 'view'
    resolution = {"width": 800, "height": 600}

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("CARLA ROS manual control")
    world = None
    try:
        display = pygame.display.set_mode(
            (resolution['width'], resolution['height']),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(role_name, resolution['width'], resolution['height'])
        world = World(role_name, hud)
        controller = KeyboardControl(role_name, hud)

        clock = pygame.time.Clock()

        while not rospy.core.is_shutdown():
            clock.tick_busy_loop(60)
            if controller.parse_events(clock):
                return
            hud.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()
        pygame.quit()


if __name__ == '__main__':

    main()
