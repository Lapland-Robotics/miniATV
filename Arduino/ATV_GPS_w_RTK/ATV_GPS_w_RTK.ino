/*
By C. Stubenvoll

For the SparkFun MicroMod ESP32 with an SparkFun GNSS ZED-F9P Carrier Board and a via I2C cable attached hight precision GNSS antenna by TAOGLAS.

This code establishes an RTK correction data stream as a NTRIP Client using the ESP32's WiFi connection (with the input of your local 
WiFi's credentials in secrets.h) and pushes the obtained RTK correction data to the ZED-F9P via I2C. 

Those two processes are looped, the frequency of the loop is determined by myGNSS.setNavigationFrequency(x) which determines how often per second a navigation
solution is available and forces this rythm onto the loop, since getting positon and accuracy data is waiting for the newest navigation solution.
Therefore, also the ROS messages are forced into the frequency of x-times per second, so if a higher frequency in ROS messages is desired, increase x.
In Example_15, the programm waits for the next RTCM frame to be available for reading and then pushing to the GNSS sensor, but since that takes usually
somewhere between 400 and 800 ms (via NLS with mountpoint ROV2, around 1000ms with rtk2go and mountpoint Taroniemi) with this code, 
it is not possible to wait for that, especially if a shorter frequency than that is desired for the ROS messages
If rosserial is giving a sync error, check if the Wifi is connected and that no loop in the program runs longer than a few seconds, 
since ROS assumes a lost connection in that case.

Two RTK correction data services can be used around Rovaniemi:
RTK2go provides the RTK data for free and without registration, their closest mountpoint for the LapinAMK in Rovaniemi is Taroniemi near Ylitornio, approximately 100km 
from Rovaniemi. Usually, a mountpoint should be as close as possible and should not be further away than approximately 30km. Surprisingly, the accuracy is still ok
with that distance. Unfortunately, it takes a bit longer to get a full dataframe.
The NLS (https://www.maanmittauslaitos.fi/en/finpos/rtk) has a base station in Rovaniemi and allows it's use for research (on registration and request), 
but it closes the connection after every RTK data delivery unexpectedly. Even with that, it is a bit faster in getting a full dataframe than RTK2go
After pushing RTK correction data to the ZED-F9P, position and accuracy data are obtained from the sensor and then send out via rosserial
as ROS messages.

Board: SparkFun ESP32 MicroMod
BoardManager: esp32 by Espressif Systems, version 2.0.9. (version 2.0.10 came out during the programming of this code but was not tested with the setup)
Libraries: SparkFun u-blox GNSS v3 (version 3.0.16 - newest at that time), Rosserial Arduino Library (version 0.9.1 - newest at that time)
Important: After the Installation of Rosserial in the Arduino IDE, open ros.h and replace "#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)"
with "#if defined(ROSSERIAL_ARDUINO_TCP)"

Based on the Example15_NTRIPClient and the Example10_GetHighPrecisionPositionAndAccuracy of the SparkFun_u-blox_GNSS_v3 library 
(https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3/tree/main/examples/ZED-F9P), see license paragraph below.

####################################################################################################################################################

SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).

The MIT License (MIT)

Copyright (c) 2016 SparkFun Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
####################################################################################################################################################
*/

#include <WiFi.h>
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

// ROS
// ROS Headers for ROSserial
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
// ROS Node Hande
ros::NodeHandle nh;

// The ros gps NavSatFix message:
sensor_msgs::NavSatFix gps_msg;
ros::Publisher atv_gps("atv_gps", &gps_msg);

// The String msg that should later become the debug message
std_msgs::String str_msg;
ros::Publisher atv_gps_debug("atv_gps_debug", &str_msg);


//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 60000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
WiFiClient ntripClient;
long rtcmCount = 0;
String debug_msg = ""; // String for debugging, also send via rosserial
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);

  // ROS setup: After established WiFi connection! Or rosserial is prone to losing sync!
  nh.initNode();
  nh.advertise(atv_gps_debug);
  nh.advertise(atv_gps);

  Wire.begin(); //Start I2C

  if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
  {
    debug_msg += "u-blox GPS not detected at default I2C address. Check wiring. Freezing.";
    // Publish the debug message
    str_msg.data = debug_msg.c_str();
    atv_gps_debug.publish(&str_msg);
    nh.spinOnce();
    delay(10);
    debug_msg = "";
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
  myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  myGNSS.setNavigationFrequency(10); //Set output in Hz. => Really important! With 1 you only get one navigation message every second!



  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    //nh.spinOnce();
    debug_msg += "Trying to connect to WiFi...";
    // Publish the debug message
    str_msg.data = debug_msg.c_str();
    atv_gps_debug.publish(&str_msg);
    nh.spinOnce();
    delay(500);
    debug_msg = "";
  }

  debug_msg += " WiFi connected with IP: ";
  debug_msg += String(WiFi.localIP());

}

void loop()
{
  
  //Connect if we are not already. Limit to 5s between attempts.
  if (ntripClient.connected() == false)
  {
    debug_msg += " Opening socket to ";
    debug_msg += String(casterHost);

    if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
    {
      debug_msg += " - Connection to caster failed";
      str_msg.data = debug_msg.c_str();
      atv_gps_debug.publish(&str_msg);
      nh.spinOnce();
      delay(10);
      debug_msg = "";
      return;
    }
    else
    {
      debug_msg += " - Connected!";
      debug_msg += " - Requesting NTRIP Data from mount point ";
      debug_msg += String(mountPoint);

      const int SERVER_BUFFER_SIZE  = 512; 
      char serverRequest[SERVER_BUFFER_SIZE + 1];

      snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n", mountPoint);

      char credentials[512];
      if (strlen(casterUser) == 0)
      {
        strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
      }
      else
      {
        //Pass base64 encoded user:pw
        char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
        snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);
        debug_msg += " - Sending credentials: ";
        debug_msg += userCredentials;

#if defined(ARDUINO_ARCH_ESP32)
        //Encode with ESP32 built-in library
        base64 b;
        String strEncodedCredentials = b.encode(userCredentials);
        char encodedCredentials[strEncodedCredentials.length() + 1];
        strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
        snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
        //Encode with nfriendly library
        int encodedLen = base64_enc_len(strlen(userCredentials));
        char encodedCredentiaserverRequestls[encodedLen]; //Create array large enough to house encoded data
        base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
      }
      //Sending server request     
      strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
      strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

      debug_msg += " - serverRequest size: ";
      debug_msg += strlen(serverRequest);
      debug_msg += " of ";
      debug_msg += String(sizeof(serverRequest));
      debug_msg += " bytes available ";
      
      ntripClient.write(serverRequest, strlen(serverRequest));

      // Publish debug msg
      str_msg.data = debug_msg.c_str();
      atv_gps_debug.publish(&str_msg);
      nh.spinOnce();
      delay(10);
      debug_msg = "";
      
      //Do not wait until the RTCM frame is available,
      //since that takes usually between 400 and 800 ms 


      //Check reply
      bool connectionSuccess = false;
      char response[512];
      int responseSpot = 0;
      while (ntripClient.available())
      {
        nh.spinOnce();
        if (responseSpot == sizeof(response) - 1) break;

        response[responseSpot++] = ntripClient.read();
        if (strstr(response, "200") != nullptr) //Look for 'ICY 200 OK'
          connectionSuccess = true;
        if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
        {
          debug_msg += " Hey - your credentials look bad! Check you caster username and password.";
          connectionSuccess = false;
        }
        debug_msg += " - Caster responded! "; // respose itself is to big to publish via rosserial
      }
      response[responseSpot] = '\0';

    } //End attempt to connect
  } //End if connected == false


  if (ntripClient.connected() == true)
  {
    uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    rtcmCount = 0;

    //Print any available RTCM data
    while (ntripClient.available())
    {
      nh.spinOnce();
      rtcmData[rtcmCount++] = ntripClient.read();
      if (rtcmCount == sizeof(rtcmData)) break;
    }

    if (rtcmCount > 0)
    {
      debug_msg += " - Last received full RTCM frame [ms]: ";
      debug_msg += String(millis() - lastReceivedRTCM_ms);
      lastReceivedRTCM_ms = millis();
      
      //Push RTCM to GNSS module over I2C
      myGNSS.pushRawData(rtcmData, rtcmCount, false);
      debug_msg += " - RTCM pushed to ZED: ";
      debug_msg += String(rtcmCount);
    }
  }

  //Close socket if we don't have new data for 60s
  if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
  {
    debug_msg += "No new RTCM data in 60s. Timeout. Disconnecting...";
    str_msg.data = debug_msg.c_str();
    atv_gps_debug.publish(&str_msg);
    nh.spinOnce();
    delay(10);
    debug_msg = "";
    if (ntripClient.connected() == true)
    ntripClient.stop();
    return;
  }

  

  if (myGNSS.getHPPOSLLH()) //This takes as much time as set with setNavigationFrequency
  {
  // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
  // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
  // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
  // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
  // getElipsoid: returns the height above ellipsoid as an int32_t in mm
  // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
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
  
    // Convert the horizontal accuracy (mm * 10^-1) to a float and then convert from mm * 10^-1 to m
    f_horizontal_accuracy = horizontal_accuracy;
    f_vertical_accuracy = vertical_accuracy;
    f_horizontal_accuracy = f_horizontal_accuracy / 10000.0;
    f_vertical_accuracy = f_vertical_accuracy / 10000.0;
  
  
    // Try to put accuracy in array for NavSatFix msg
    // The msg requires East, North and Height - all in [m²]
    // But for comparisons in other parts of the system, I just need unsqared horizontal accuracy
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
  
    //nh.spinOnce();
  }
  else
  {
    debug_msg += "Not able to get position data from sensor!";
  }

  // Publish the debug message
  str_msg.data = debug_msg.c_str();
  atv_gps_debug.publish(&str_msg);

  nh.spinOnce();
  delay(10);
  debug_msg = "";
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
