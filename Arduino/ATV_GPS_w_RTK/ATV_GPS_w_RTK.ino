/*
  Get the high precision geodetic solution for latitude and longitude
  By: Nathan Seidle
  Modified by: Steven Rowland and Paul Clark
  SparkFun Electronics
  Date: April 17th, 2020
  License: MIT. See license file for more information.

  This example shows how to inspect the accuracy of the high-precision
  positional solution. Please see below for information about the units.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS
#include <WiFi.h> // ??
#include "secrets.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

// ROS
// ROS Headers for ROSserial
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
// ROS Node Hande
ros::NodeHandle nh;

// The String msg that should later become the debug message
std_msgs::String str_msg;
ros::Publisher atv_gps_debug("atv_gps_debug", &str_msg);

// The ros gps NavSatFix message:
sensor_msgs::NavSatFix gps_msg;
ros::Publisher atv_gps("atv_gps", &gps_msg);

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 100000; //If we fail to get a complete RTCM frame after 100s, then disconnect from caster
String debug_msg = ""; // String for debugging
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void setup()
{
  delay(1000);
  
  debug_msg += "Beginn: \n";

  // ROS setup
  nh.initNode();
  //nh.advertise(atv_gps_debug);
  nh.advertise(atv_gps);

  Wire.begin();

  //myGNSS.enableDebugging(Serial);

  if (myGNSS.begin(Wire) == false) //Connect to the u-blox module using Wire port
  {
    debug_msg += "u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.";
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5, VAL_LAYER_RAM); //Set output to 5 times a second. Change the RAM layer only - not BBR

  byte rate;
  if(myGNSS.getNavigationFrequency(&rate)) //Get the update rate of this module
  {
    debug_msg += "Current update rate: ";
    String upd_rate_str = String(rate) + "\n";
    debug_msg += upd_rate_str;
  }
  else
  {
    //Serial.println("getNavigationFrequency failed!");
    debug_msg += "getNavigationFrequency failed! \n";
  }

  // WiFi connection for NTRIP correction data
  //Serial.print(F("Connecting to local WiFi"));
  debug_msg += "Connecting to local WiFi \n";
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(F("."));
    debug_msg += ".";
  }
  //Serial.println();
  //debug_msg += "\n";

  debug_msg += "\n WiFi connected with IP: ";
  String wifi_ip_str = String(WiFi.localIP()) + "\n";
  debug_msg += wifi_ip_str + "\n";
}

// Pretty-print the fractional part with leading zeros - without using printf
// (Only works with positive numbers)
String printFractional(int32_t fractional, uint8_t places)
{
  String return_str = "";
  if (places > 1)
  {
    for (uint8_t place = places - 1; place > 0; place--)
    {
      if (fractional < pow(10, place))
      {
        return_str += "0";
      }
    }
  }
  String fract_str = String(fractional);
  return_str += fract_str;
  return return_str;
}

void publish_ros_msgs()
{
  if (myGNSS.getHPPOSLLH())
  {
    // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // If you want to use the high precision latitude and longitude with the full 9 decimal places
    // you will need to use a 64-bit double - which is not supported on all platforms

    // To allow this example to run on standard platforms, we cheat by converting lat and lon to integer and fractional degrees

    // The high resolution altitudes can be converted into standard 32-bit float

    // First, let's collect the position data
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t horizontal_accuracy = myGNSS.getHorizontalAccuracy();
    uint32_t vertical_accuracy = myGNSS.getVerticalAccuracy();

    // Defines storage for the lat and lon units integer and fractional parts
    int32_t lat_int; // Integer part of the latitude in degrees
    int32_t lat_frac; // Fractional part of the latitude
    int32_t lon_int; // Integer part of the longitude in degrees
    int32_t lon_frac; // Fractional part of the longitude

    // Calculate the latitude and longitude integer and fractional parts
    lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
    lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
    if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lat_frac = 0 - lat_frac;
    }
    lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
    lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
    if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lon_frac = 0 - lon_frac;
    }
    
    //Assemble the the lat and lon strings
    String latitude_msg = ""; //Latitude string accumulating the different parts of the latitude
    String longitude_msg = ""; //Longitude string "
    String lat_str = String(lat_int);
    latitude_msg += lat_str;
    latitude_msg += ".";    
    latitude_msg += printFractional(lat_frac, 9);  // For printing the fractional part of the latitude with leading zeros
    float f_latitude = latitude_msg.toFloat();
    String lon_str = String(lon_int);
    longitude_msg += lon_str;
    longitude_msg += ".";
    longitude_msg += printFractional(lon_frac, 9);
    float f_longitude = longitude_msg.toFloat();

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    //float f_msl;
    float f_horizontal_accuracy;
    float f_vertical_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1 and then convert from mm * 10^-1 to m
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    f_ellipsoid = f_ellipsoid / 10000.0;

    // ROS NavSatFix requires the height above the ellipsoid, but the msl seems more accurate?
    // Calculate the height above mean sea level in mm * 10^-1 and then convert from mm * 10^-1 to m
    //f_msl = (msl * 10) + mslHp;
    //f_msl = f_msl / 10000.0;

    // Convert the horizontal accuracy (mm * 10^-1) to a float and then convert from mm * 10^-1 to m
    f_horizontal_accuracy = horizontal_accuracy;
    f_vertical_accuracy = vertical_accuracy;
    f_horizontal_accuracy = f_horizontal_accuracy / 10000.0;
    f_vertical_accuracy = f_vertical_accuracy / 10000.0;


    // Try to put accuracy in array for NavSatFix msg
    // The msg requires East, North and Height - all in [m²]
    // But for comparisons in other parts of the system, I just need unsqared horizontal accuraca
    // so the unsquared accuracies are smuggled into the gps message
    float acc_cov[9] = {f_horizontal_accuracy, 0.0, 0.0, 0.0, f_horizontal_accuracy, 0.0, 0.0, 0.0, f_vertical_accuracy};

    // Assemble the gps message and publish it
    gps_msg.header.stamp = nh.now();
    gps_msg.header.frame_id = "map";
    gps_msg.latitude = f_latitude;
    gps_msg.longitude = f_longitude;
    gps_msg.altitude = f_ellipsoid;
    gps_msg.position_covariance[0] = acc_cov[0];
    gps_msg.position_covariance[4] = acc_cov[4];
    gps_msg.position_covariance[8] = acc_cov[8];
    gps_msg.position_covariance_type = 1;
    atv_gps.publish(&gps_msg);

    // Publish the debug message
    //str_msg.data = debug_msg.c_str();
    //atv_gps_debug.publish(&str_msg);
    
    nh.spinOnce();
    delay(500);
  }
}

void beginClient()
{
  WiFiClient ntripClient;
  long rtcmCount = 0;
  
  while (WiFi.status() == WL_CONNECTED)
  {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      debug_msg += "Opening socket to ";
      String caster_host_str = String(casterHost);
      debug_msg += caster_host_str;

      if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
      {
        debug_msg += "Connection to caster failed";
        return;
      }
      else
      {
        debug_msg += "Connected to ";
        String caster_port_str = String(casterPort);
        debug_msg += caster_host_str + ": " + caster_port_str + "\n";
        debug_msg += "Requesting NTRIP Data from mount point ";
        String mount_pt_str = String(mountPoint);
        debug_msg += mount_pt_str + "\n";

        const int SERVER_BUFFER_SIZE  = 512;
        char serverRequest[SERVER_BUFFER_SIZE + 1];

        //snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
        //          mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0)
        {
          //strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
          debug_msg += "Accept: */*\r\nConnection: close\r\n";
        }
        else
        {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
          //snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);
          String user_cred_str = String(userCredentials);
          debug_msg += "Sending credentials " + user_cred_str + "\n";

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
          //snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
        }
        //strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        //strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            debug_msg += "\n Caster timed out! \n";
            ntripClient.stop();
            //Serial.println(debug_msg);
            //delay(10);
            debug_msg = "";
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available())
        {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") != nullptr) //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
          {
            debug_msg += "\n Hey - your credentials look bad! Check you caster username and password. \n";
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        String resp_str = String(response);
        debug_msg += "Caster responded with: " + resp_str + "\n";

        if (connectionSuccess == false)
        {
          debug_msg += "Failed to connect to " + caster_host_str + ": " + resp_str + "\n";
          return;
        }
        else
        {
          debug_msg += "Connected to " + caster_host_str + "\n";
          lastReceivedRTCM_ms = millis(); //Reset timeout
        }
      } //End attempt to connect
    }//End ntripClient.connected == false

    if (ntripClient.connected() == true)
    {
      uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available())
      {
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0)
      {
        lastReceivedRTCM_ms = millis();

        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        String rtcm_str = String(rtcmCount);
        debug_msg += "RTCM pushed to ZED: " + rtcm_str + "\n";
        
        publish_ros_msgs();
      }
    } // End ntripClient.connected() == true

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      debug_msg += "RTCM timeout. Disconnecting... \n";
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    } // End close socket in case of no new data

    delay(10);
  }
  debug_msg += "Disconnecting...";
  ntripClient.stop();

}

void loop()
{ 
  beginClient();
  
  // TODO: Only if publish_ros_msg() is in beginClient, there are no error messages. If it is here, there might be a storage problem?
  // TODO: Move publish_ros_msg() somewhere more time-efficient than once every second
  //Send the gps message
  //publish_ros_msgs();
  //delay(500);
  //atv_gps.publish(&gps_msg);
  //str_msg.data = debug_msg.c_str();
  //debug_msg = "";
  //atv_gps_debug.publish(&str_msg); 
  //nh.spinOnce();

  // delay(500) makes everything slower, with delay(1000) a new msg is published roughly every second
  delay(1000);
}
