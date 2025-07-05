
const char* APP = "AmbientAP ";
const char* VERSION = "2025 v0704 ";

/////////////////////////////////////////////////////////////////////////////////////
//
// AmbientAP is a flexible, multi-featured sensor platform.  It can:
//  1. expose sensor values via http GET command requests from a client.
//  2. expose sensor values via lighter footprint ESPNOW protocol which pushes data to a peer/client periodically without receiving a request.
//  3. expose sensor values via MQTT to Home Assistant or other MQTT broker.
//  3. operate using either an ESP32 or ESP8266 (including d1 mini) controller.
//  4. accommodate the following sensors (simultaneusly if an ESP32 is used):
//      temperature
//      humidity          
//      atmospheric pressure          
//      light intensity
//      water leaks/flooding
//      door/window state
//      pir motion sensors 
//      ultrasonic sensors measure distance (car parking aid or car presence detector in garage for example) 
//  6. sleep and wake up via a timer, an interrupt (such as a window being opened) or motion, or it can stay awake for continuous monitoring.
//  7. sleep immediately after sending a successfully received ESPNOW message.
//  8. operate with or without an OLED display.
//  9. operate as one of many such devices using this software by assigning unique SensorID (1, 2, 3, etc.) to each sensor platform.
//
/*////////////////////////////////////////////////////////////////////////////////////

  Copyright 2022 Bob Jessup  MIT License:
  https://github.com/Bobbo117/Cellular-IoT-Monitor/blob/main/LICENSE

  Certain code within the #ifdef ESPNOW compiler directives is adapted from the following:
  https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
   
*////////////////////////////////////////////////////////////////////////////////////
/*
NOTES -  1. IF using MELife for ESP32 ESP-32S Development Board, use Arduino IDE "esp32 Dev Module" or "MH ET LIVE ESP32 Devkit"
         2. IF using d1 Mini 8266, use Arduino IDE "LOLIN(WEMOS) d1 R2 & Mini"

            For d1 Mini 8266, connect pins D0 and RST pins w/jumper 
            to wake up from sleep; disconnect jumper to load program

         3. Be sure to include all libraries shown in #include statements below for the selected processor in step 1 or 2
         
         4. Onboard LED states for debug/verification purposes when DEBUG_ is on:
            -->ON at setup Start, off at setup end.
            -->ON at GET temperature
            -->OFF at GET humidity
            -->OFF at  sleep     
*/
   
//////////////////////////////////////////////////////////////  
//*******     Setup secrets.h file               ***********//
//               
//////////////////////////////////////////////////////////////

  #include <secrets.h>   // secrets.h file contents follow:

  //#define SECRET_WIFI_SSID           "your wifi ssid"
  //#define SECRET_WIFI_PASSWORD       "your wifi password" 

  // Home Assistant or other MQTT broker
  //#define SECRET_MQTT_HOST "your mqtt host"  //12/29/2022
  //#define SECRET_MQTT_PORT 1883
  //#define SECRET_MQTT_USER_NAME "your mqtt user name"
  //#define SECRET_MQTT_USER_PWD  "your mqtt password" 

  //  HUB MAC Address (if you want to transmit to a cellular hub using ESPNOW):
  //  uint8_t SECRET_broadcastAddress[] = {0x8C, 0xAA, 0xB3, 0x85, 0x6A, 0x67};  //example for Mac Address 8C:AA:B3:85:6A:67

 
//////////////////////////////////////////////////////////////  
//*******         Compile-time Options           ***********//
//        (Disable unwanted options with leading //)
////////////////////////////////////////////////////////////////#define OTA_             // Enable over the air software updates

//#define HA_              // Enable direct ommunication with Home Assistant for enhanced real time sensor updates and monitoring via local wifi.

//#define GATEWAY_         //optional internet gateway (router) in use by destination device (Hub).
                           //software will determine the wifi channel being used to communicate with the hub
                           ///NOTE - Enable gateway if using ESPNOW and hub is reporting to Home Assistant!!

  #define DEBUG_           //omment out if no printout to Serial monitor 
  
  //Select one:
  //*****************
  //#define HTTPGET_       //set up as wifi server w/o router to transfer data upon receiving a http GET message
  #define ESPNOW_1TO1_     //send data to one peer: enter MAC address of receiver in secrets.h file  
                           //ESPNOW_1to1 is preferred; NOTE: HTTPGET is a memory hog.

//////////////////////////////////////////////////////////////
                         
  //*****************************
  // 1. Select one CASA (reporting site which may have unique configuration):
  // IMPORTANT: CHANGE SECRETS.H FILE WHEN CHANGING CASA !!!
  //*****************************
  //#define CASA_1            // Site #1 LillyGo (FL)
  //#define CASA_1a           // Site# 1 Botletics
  //  #define CASA_2            // Site #2 LillyGo
  #define CASA_2a           // Site# 2 Botletics

  //*****************
  // 2. Select unique sensorID
  //*****************
  #define ID1     //FL gar door  ME basement
//  #define ID2     //FL car sonic ME 2 bedroom 2a gur=est bath wh-BL
//  #define ID3     //FL car hatch ME kitchen
//  #define ID4     //FL Kitchen ME bathroom
//  #define ID5     //FL bath ME #13 bath
//  #define ID6     //FL bath2 / lanai

  //*****************
  // 3. Misc flags/variables
  //*****************
  bool reportFlag = true;      //issue report to HUB after startup, also when new data exists 
  bool haPublishFlag = true;   //issue report to HA after startup, also when new data exists 
  bool printSensorDataFlag = true;
  int loopStatus = 0;
  char topic[100];             //mqtt topic
  char topTopic[20];           //top level mqtt topic
  uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
  uint8_t wakeupID;                   //reason sensor woke up; see readWakeupID 
  String hum="--",tem="--",pres="--"; //adjusted values sent to hub if wifi used: "--" if nan or current reading if valid

  //*****************                                                                                    
  // 4. Timing Parameters
  //    NOTE: Wemos D1 Mini 8266 max sleep time is 71 minutes = 4260 seconds
  //*****************
  RTC_DATA_ATTR int bootCount = 0;    //increment by 1 each time the processor wakes up OR experiences chillCount
  #define bootResetCount 6*24         //10 min/wakeup period x6 = 60 min x 24 = 24 hrs; i.e., reset once per day

  //*****************
  //  5. Set prameters for each device
  //*****************

  //Select one temperature sensor for each sensor ID below if applicable:
  //#define AHT10_        // Adafruit AHT10 
  //#define BME_          // BME280 FL
  //#define DHT_          // DHT11,21,22, etc.
  //#define SHT20_        // DFRobot SHT20

  //////////////////////////////////////////////////////////////
  #ifdef CASA_1

    //*****************
    // Timing Parameters
    //***************** 
    #ifndef ID2
      const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
      const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
      const long chillSeconds = 10;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
    #endif
    #ifdef ID2
      const int sleepSeconds = 0;          //8*60 default sleep time in seconds; 0 if no sleep                                              
      const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
      const long chillSeconds = 10;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
    #endif

    //*****************
    #ifdef ID1  
      uint8_t sensorID = 1;
      #define displayTitle " ~AMBIENT 1~"
      const char* ssid = "AMBIENT_1";
      #define BME_ 
      
      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range 
      #define TOPTOPIC_ "garage/"                                         
      #define OLED_               //option: comment out if no OLED display 128x64
      #ifdef OLED_
        //If enabled, choose one of the following OLED formats:
        //#define oledFormat1       // displays only temp humidity pressure on OLED display
        #define oledFormat2         // displays all sensors on optional OLED display
        #define h2oThreshhold  1    // wet - dry threshhold % for OLED display  
      #endif   
    #endif

    //*****************
    #ifdef ID2
      uint8_t sensorID = 2; 
      const char* ssid = "AMBIENT_2";
      #define displayTitle " ~AMBIENT 2~"
      //#define DHT_   

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range      
       #define TOPTOPIC_ "sonic/"                                         
    #endif
  
    //*****************
    #ifdef ID3
      uint8_t sensorID = 3;
      #define displayTitle " ~AMBIENT 3~"
      const char* ssid = "AMBIENT_3";
      #define DHT_ 

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range   
       #define TOPTOPIC_ "hatch/"                                                       
    #endif
  
    //*****************
    #ifdef ID4
      //FL kitchen used for temp hum and pir motion to cause watermain standby
      uint8_t sensorID = 4;
      #define displayTitle "~AMBIENT4~"
      const char* ssid = "AMBIENT_4";
      #define AHT10_  
      
      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
       #define TOPTOPIC_ "kitchen/"                                                
    #endif
  
    //*****************
    #ifdef ID5
      //FL  bath used for temp hum and pir motion to cause watermain standby
      uint8_t sensorID = 5;
      #define displayTitle "~AMBIENT5~"
      const char* ssid = "AMBIENT_5";
      #define BME_ 

      #define ESP32_           //Recommended choice is esp32
      #define d1MiniESP32_     //select both ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
       #define TOPTOPIC_ "bathroom/"                                         
    #endif
  
    //*****************
    #ifdef ID6
      //FL bath 2 used for temp hum and pir motion to cause watermain standby
      uint8_t sensorID = 6;
   
      #define displayTitle "~AMBIENT6~"
      const char* ssid = "AMBIENT_6";
      #define AHT10_ 

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                                 
       #define TOPTOPIC_ "bathroom2/"                                         
      #define OLED_               //option: comment out if no OLED display 128x64
      #ifdef OLED_
        //If enabled, choose one of the following OLED formats:
        //#define oledFormat1       // displays only temp humidity pressure on OLED display
        #define oledFormat2         // displays all sensors on optional OLED display
        #define h2oThreshhold  1    // wet - dry threshhold % for OLED display  
      #endif   
    #endif

  #endif

  //////////////////////////////////////////////////////////////  
    #ifdef CASA_1a
    char dataTag[] = "F2";   // <<< unique prefix to identify source of data >
     
    //Data Inputs - select one of these two if receiving data wirelessly from sensors:
//#define ESPNOW      // to receive or send by espnow, A lightweight communication protocol by which sensors push data to the hub based on MAC address
    //#define HTTPGET   // A slower system by which the HUB polls the sensor via http GET request to pull data; WIFI is a memory hog.

    #ifdef cellularMode
#define simSleepMode   // shut off sim modem between readings 
    #endif
    
 //   #define alertMode          //  enables postAlert when a measurement crosses threshhold; // = no alert
//  #define GARAGE

    //*****************
    // Timing Parameters
    //***************** 
    const int espSleepSeconds = 58*60;     // or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
    const long espAwakeSeconds = 2*60;    // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
    int heartbeatMinutes = 0;             // (default 10) heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less than espAwakeSeconds
                                           // IF espSleepSeconds - 0, then this is the heartbeat frequency.
    const long chillSeconds = 10;          // (not used)interval between readings if sleepSeconds = 0 slows serial monitor updates
 const long sensorSeconds = 10*60;        // interval in seconds to refresh time sensitive sensor readings 
  
    const long serialMonitorSeconds = 5*60; // (growth) interval in seconds to refresh serial monitor sensor readings (enhances readability)            
    const long samplingRateSeconds = 10;    // (growth) interval in seconds between sensor readings in alertMode
  
    #define bootResetCount 24         //reset ESP once per day

    //*****************
    // Board selection 
    //*****************
    
    #define BOTLETICS  //Use for SIM7000A boards including Botletics shield connected to ME LIFE ESP32 or equivalent
    //#define LILLYGO  //Use for Lillygo TT Go SIM7000G board with on-board wrover ESP32 
  
    //*****************
    // Temperature sensor 
    //****************
    //#define AHT10_    // Adafruit AHT10  <--GARAGE
    #define BME_        // BME280 temperature, humidity, pressure sensor
    //#define DHT_      // DHT11,21,22, etc. temperature, humidity sensors
    //#define MCP9808_  // botletics shield temperature sensor
    //#define SHT20_    // DFRobot SHT20
  #endif  //CASA_1a
  
  //////////////////////////////////////////////////////////////
  #ifdef CASA_2
    const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
    const long awakeSeconds = 0; //2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
    const long chillSeconds = 5*60;      //interval between readings if sleepSeconds = 0 slows serial monitor updates     
    const long sensorSeconds = 10*60;        // interval in seconds to refresh sensor readings 
    
    //*****************
    #ifdef ID1
      uint8_t sensorID = 1;
      #define displayTitle "CASA_2 #1 "   

      const char* ssid = "AMBIENT_1";
      #define DHT_ 

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range 
      #define TOPTOPIC_ "2basement/"                                         
  
      #define OLED_               //option: comment out if no OLED display 128x64
      #ifdef OLED_
        //If enabled, choose one of the following OLED formats:
        //#define oledFormat1       // displays only temp humidity pressure on OLED display
        #define oledFormat2         // displays all sensors on optional OLED display
        #define h2oThreshhold  1    // wet - dry threshhold % for OLED display  
      #endif          
    #endif

    //*****************
    #ifdef ID2
      uint8_t sensorID = 2; 
      const char* ssid = "AMBIENT_2";
      #define displayTitle "CASA_2 #2"
      #define DHT_            // DHT11,21,22, etc.

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
      #define TOPTOPIC_ "2bedroom/"
    #endif
      
    //*****************
    #ifdef ID3
      uint8_t sensorID = 3;
      #define displayTitle "CASA_2 #3"
      const char* ssid = "AMBIENT_3";
      #define DHT_            // DHT11,21,22, etc.

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
      #define TOPTOPIC_ "2kitchen/"
    #endif

    //***************** 
    #ifdef ID4
      uint8_t sensorID = 4;
      #define displayTitle "CASA_2 #4"
      const char* ssid = "AMBIENT_4";
      #define AHT10_         // DHT11,21,22, etc.

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
      #define TOPTOPIC_ "2bathroom/"

    #endif

  #endif

  //////////////////////////////////////////////////////////////

  #ifdef CASA_2a
    const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
    const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
    const long chillSeconds = 5;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
    const long sensorSeconds = 10*60;        // interval in seconds to refresh sensor readings 

    //*****************
    #ifdef ID1
      uint8_t sensorID = 1;
      #define displayTitle "CASA_2a #1 "      
      const char* ssid = "AMBIENT_1a";
      #define AHT10_ 

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_   //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
      #define TOPTOPIC_ "2abasement/"
      #define OLED_               //option: comment out if no OLED display 128x64
      #ifdef OLED_
        //If enabled, choose one of the following OLED formats:
        //#define oledFormat1       // displays only temp humidity pressure on OLED display
        #define oledFormat2         // displays all sensors on optional OLED display
        #define h2oThreshhold  1    // wet - dry threshhold % for OLED display  
      #endif   
    #endif

    //*****************
    #ifdef ID2
      uint8_t sensorID = 2;
      const char* ssid = "AMBIENT_2a";
      #define displayTitle "CASA_2a #2"
      #define AHT10_ 

      #define ESP32_           //Recommended choice is esp32
      //#define d1MiniESP32_     //select ESP32 and d1MiniESP32 if diMiniESP32 is used
      //#define ESP8266_       //use for wemos D1 Mini.  
                               //NOTE Multiple D1 Minis seem to interfere with one another and have limited wireless range                 
      #define TOPTOPIC_ "2abathroom/"
    #endif

  #endif
  
  //*****************
  //  6. sensor inventory for each device
  //*****************
  // arrays to indicate types of sensors aboard each device (1=presnt, 0 = absent)
  // HUB  id=0; boards are 1,2,3.. example: temps[]={1,0,1,0} indicates hub and sensor #2 have temp sensors, sensor #1 and #3 do not.

  #ifdef CASA_1
    //const char* location[] = {"hub","garage","sonic","hatch",      "kitchen","bathroom","bathroom2,"spare"};
    uint8_t temps[] = {1,1,0,0, 1,1,1,0}, hums[]=  {1,1,0,0, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, press[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,0,0,0, 0,0,0,0}, h2os[] = {0,0,0,0, 0,0,0,0}, doors[]={0,1,0,1, 0,0,0,0}, pirs[]={0,1,0,0, 1,1,1,0}, sonics[]={0,0,1,0, 0,0,0,0};
  #endif
  
  #ifdef CASA_2:
    //const char* location[] = {"hub","basement","bedroom","kitchen","bathroom","spare","spare","spare"};
    uint8_t temps[] = {1,1,1,1, 1,1,1,0}, hums[]=  {1,1,1,1, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, press[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,0,1, 0,0,0,0}, h2os[] = {0,1,0,0, 0,0,0,0}, doors[]={0,0,1,1, 0,0,0,0}, pirs[]={0,1,0,1, 0,0,0,0}, sonics[]={0,0,0,0, 0,0,0,0};
  #endif

  #ifdef CASA_2a:
    //const char* location[] = {"hub","basement","bedroom","kitchen","bathroom","spare","spare","spare"};
    uint8_t temps[] = {1,1,1,1, 1,1,1,0}, hums[]=  {1,1,1,1, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, press[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,1,1, 0,0,0,0}, h2os[] = {0,1,0,0, 0,0,0,0}, doors[]={0,0,0,0, 0,0,0,0}, pirs[]={0,0,0,0, 0,0,0,0}, sonics[]={0,0,0,0, 0,0,0,0};
  #endif  
 
  //*****************                                                                                    //*****************
  // 7. ESPNOW Parameters (if ESPNOW is selected)
  //*****************
  int espNowAttempts=0;            //number of ESPNOW transmission attempts counter
  #define espNowAttemptsAllowed 30 //number of unsuccessful attempts allowed before sleeping if sleepSeconds > 0
  int espNowDelay = 50;            //delay in msec between espnow attempts

  //*****************
  // 8. ESP32 PIN DEFINITIONS
  //    note: esp32 set pin 15 low prevents startup log on Serial monitor - 
  //          good to know for battery operation
  //    From 363-ESP32 Pri 1 pins - The guy with the swiss accent:
  //      Pri1: GPIO 4*,5,16,17,18,19,23,32,33; (16,17 not available on Wrover)
  //      Pri2: (input only, no internal pullup) GPIO 34,35,36,39.
  //      Do not use ADC2 (GPIO 2,4*,12-15, 25-27) if using wifi.
  //      Reserve GPIO 21,22 for I2C bus
  //      GPIO's 14,16,17,18,19,21-23 have internal 10K pullup resisters
  //      Can use 1,2,4,12-15, 25-27, 32-39 as wakeup (interrupt) source; avoid conflict w/ wifi by using only 32-39 
  //*****************
  #ifdef ESP32_
    #define pinBoardLED 2               //onboard LED
    #ifndef d1MiniESP32_
      #define pinPir 39                 //HC-SR501 PIR sensor       for full size esp32    
      #define pinPhotoCell 34 
    #endif     
    #ifdef d1MiniESP32_
      #define pinPir 34                 //HC-SR501 PIR sensor     for d1 mini esp32 
      #define pinPhotoCell 33           
    #endif  
    #define pinSonicEcho 33                                       
    #define pinSonicTrigger 23          
    #define pinDoor 35                  //door wired to magnetic reed switch, NO & C connected 
                                        //  to pinDoor and ground; 1K pullup R to pinDoor & 3.3v
                                        //  pinDoor = 1 = closed, 0 = open.
                                        //NOTE if this pin is changed, 
                                        //  change setupWakeupConditions as well.
    //#define pinDh2o 18                 //digital moisture digital indicater *                         
    //#define pinAh2o 36                 //analog moisture analog measure *
                                        //   * Hiletgo LM393 FC37 moisture monitor
                                        
    #define pinAh2o 32                  // touchpin                                
    //#define pinPhotoCell 34             // analog input 
                                        // esp32 analog input; photocell connected to GPIO pin & gnd, 
                                        // 10K pullup resister, i.e., connected to GPIO pin & 3.3v
    #define pinSDA 21                   // ESP 12C Bus SDA for BME temp sensor, OLED, etc
    #define pinSCL 22                   // ESP 12C Bus SCL for BME temp sensor, OLED, etc
    
  #endif
 
  //*****************
  // 9. ESP8266 D1 Mini PIN DEFINITIONS
  //***************** 
  #ifdef ESP8266_
    #define pinBoardLED 2               //onboard LED
    #define pinDoor 14                  //door switch D5
    #define pinDh2o 13                  //digital flood sensor input D7                             
    //#define pinAh2o                     //analog flood sensor not connected
    #define pinPhotoCell A0             //analog input 
  #endif

  //*****************
  // 10. Temp Sensor Libraries
  //*****************
  #ifdef AHT10_
    #include <Adafruit_AHT10.h>
    Adafruit_AHT10 aht;
  #endif
  #ifdef DHT_
    #include <Adafruit_Sensor.h>
    #define pinDHT 5                 
    #include "DHT.h"
    #define DHTTYPE DHT11  //or DHT21, DHT22
    DHT dht(pinDHT,DHTTYPE);
  #endif  
  #ifdef BME_
    #include <Adafruit_Sensor.h>
    #include <Wire.h>               //12C bus library
    #include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
    Adafruit_BME280 bme;            // I2C
  #endif
  #ifdef SHT20_  
    #include "DFRobot_SHT20.h"
    DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
  #endif    


////////////////////////////////////////////////////////////////////////////
//*********            End of Compile-time Options           ***************
////////////////////////////////////////////////////////////////////////////

  // Platform sensor structure
  // Structure to send data must match the receiver structure if espNow is used:
  //*****************
   typedef struct platforms {  // create an definition of platformm sensors
      uint8_t id;              // id must be unique for each sender; HUB  id=0; boards are 1,2,3..
      float temperature;       // F
      float humidity;          // %
      float pressure;          // mb
      uint8_t lux;             // 0-99 % of full illumination
      uint8_t aH2o;            // 0-99 % of full sensor level detection
      uint8_t dH2o;            // 0 or 1 digital readout 0=dry  (normal state)
      uint8_t doorCount;       // # times door opened and closed after previous successful report to HUB
      uint8_t door;            // door = 0 when closed, 1 when open
      uint8_t pir;             // pir = 0 no motion, 1 for motion 
      uint16_t sonic;          // 0=absent 1=present
      uint8_t sendFailures;    // # failed attempts to send via espNow
      uint8_t pirCount;        // # times pir detected motion after previous successful report to HUB
  } platforms;
  
  #ifdef ESP32_  //store the reaadings persistently if esp32
    RTC_DATA_ATTR platforms sensorData ={sensorID,0,0,0,0,0,0,0,0,0,0,0,0};     
  #endif
  #ifdef ESP8266_
    platforms sensorData = {sensorID,0,0,0,0,0,0,0,0,0,0,0,0};   to initialize  
  #endif  
  uint8_t aH2oMeasured = 0; // sensor measurement prior to preocessing
  RTC_DATA_ATTR uint8_t priorDoor = 0; //last door status = 0 (closed) or 1 (open)
 
  //*****************
  // OLED DISPLAY Libraries
  //*****************
  #ifdef OLED_
    #include <Wire.h>
    #include "SSD1306Ascii.h"
    #include "SSD1306AsciiWire.h"
    #define I2C_ADDRESS 0x3C            // 0X3C+SA0 - 0x3C or 0x3D
    #define OLED_RESET     -1           // Reset pin # (or -1 if sharing Arduino reset pin)
    SSD1306AsciiWire oled;
  #endif

  //*****************
  // WIFI Server Libraries
  //*****************
  #ifdef HTTPGET_
    #ifdef ESP32_
      #ifndef WIFI_H
        #define WIFI_H
        #include <WiFi.h>        
      #endif    
    #endif
    #ifdef ESP8266_
      #include <ESP8266WiFi.h>           // wifi libr for esp8266 if you must
      #include <ESPAsyncTCP.h>           //added for esp8266
    #endif                
    #include "ESPAsyncWebServer.h"
    AsyncWebServer server(80);           //Create AsyncWebServer object on port 80
    const char* password = "123456789";
  #endif

  //*****************
  // ESP NOW One to One Libraries
  //*****************
  uint8_t sendFailureCount = 0;       // # failed attempts to send via espNow; if >0, data not resent to HA; data resent to ESPNow 
  long espNowTimeOut = 1;          // espNOW timeout in seconds
  uint8_t espNowStatus = 2;         // 0=not started; 1 = in process; 2=completed

  #ifdef ESPNOW_1TO1_ 
    #include <esp_now.h>
    #ifdef ESP32_
      #ifndef WIFI_H
        #define WIFI_H
        #include <WiFi.h>        
      #endif    
    #include <esp_wifi.h>
    #endif
    #ifdef ESP8266_
      #include <ESP8266WiFi.h>                // wifi libr for esp8266
      #include <ESPAsyncTCP.h>           //added for esp8266
    #endif  
    esp_now_peer_info_t peerInfo;   // Create peer interface if sending
  #endif

  //*******************
  // OTA Libraries
  //******************
  #ifdef OTA_
    #ifndef WIFI_H
      #define WIFI_H
      #include <WiFi.h>
    #endif
  #endif
  
  //*******************************   
  // 16. Home Assistant setup
  //*******************************
  #ifdef HA_ 
    TimerHandle_t mqttReconnectTimer; //Create two Timer Objects
    TimerHandle_t wifiReconnectTimer;
    //#define mqTTMode
    //****************wifi stuff
    #ifndef WIFI_H
      #define WIFI_H
      #include <WiFi.h>        
    #endif    
    #include "WiFiMulti.h"
    #include <HTTPClient.h>   

    //***************mqtt stuff
    #include <AsyncMqttClient.h>
    AsyncMqttClient mqttClient;  // Create a MQTT client Object
  #endif
  
  /************************************
  void IRAM_ATTR doorChangeInterrupt() {  //disabled can poll door fast in loop
    //door callback function counts door changes
    sensorData.door ++; 
    #ifdef DEBUG_
      Serial.println(F("*doorChangeInterrupt*"));
      printSensorData();
    #endif  
  }
  */
  /************************************
  void IRAM_ATTR pirInterrupt() {  //disabled due to excessive false interrupts
    //PIR callback function counts pir changes
    sensorData.pir ++; //= sensorData + digitalRead(pinPir);
    #ifdef DEBUG_
      Serial.println(F("*pirInterrupt*"));
      printSensorData();
    #endif  
  }
  */

//*********************************
void blinkBoardLED(int sec){      // Blink board LED for sec seconds at .5 second intervals
  #ifdef DEBUG_ 
    Serial.println(F("*****blinkBoardLED*****"));
  #endif  
  for (int i=0;i<sec;i++){
   turnOnBoardLED();            // Turn on boaard LED .5 seconds
   delay(500);
   turnOffBoardLED();           // Turn off boaard LED .5 seconds
   delay(500);
  }
}

//*********************************

void displayOLED(){  
  //display sensor readings on oled display if #define OLED_ is enabled
  
  #ifdef OLED_  
    #ifdef DEBUG_
      Serial.println(F("*****displayOLED*****"));
    #endif  
    char buf0[6],buf1[6];
    //oled.clear(); 
    oled.setFont(fixed_bold10x15);
    oled.setCursor(0,0); 
    oled.println(displayTitle);       //top line is display title
    
    #ifdef oledFormat1
      oled.print("  ");oled.print(sensorData.temperature);oled.println(" F    ");
      oled.print("  ");oled.print(sensorData.humidity);oled.println(" %    ");
      oled.print("  ");oled.print(sensorData.pressure);oled.println("mb    "); 
    #endif
    
    #ifdef oledFormat2
      int iTemperature=sensorData.temperature + .5;
      oled.print(iTemperature);
      #ifdef DEBUG_
        //Serial.print(" tem, Temperature: ");Serial.print(tem);Serial.print(", ");Serial.print(iTemperature);Serial.println(" *F");
      #endif 
      if(tem=="--"){
        oled.print (" F?");
      }else{ 
        oled.print(" F ");
      }

      #ifdef DEBUG_
        oled.print(bootCount);  //0901: to make sure add is rebooting
        oled.print(" ");
      #else
        oled.print("  ");
      #endif
      
      int iHumidity=sensorData.humidity +.5;
      oled.print(iHumidity);
      if(hum=="--"){
        oled.println("?    ");
      }else{
        oled.println("%    ");
      }
      #ifdef DEBUG_
       // Serial.print(" hum, Humidity: ");Serial.print(hum);Serial.print(F(", "));Serial.print(iHumidity);Serial.println(F("%"));
      #endif  

      if(luxs[sensorID]==1){
        oled.print ("LIGHT   ");
        #ifdef DEBUG_
          // Serial.print(" lux: ");Serial.println(sensorData.lux);
        #endif  
        oled.print(sensorData.lux);
        oled.println("%      ");
      }  
    
      if(h2os[sensorID]==1){
        if(sensorData.aH2o - h2oThreshhold > 0){  //display leak status in real time
          oled.print("WET");
          oled.print(aH2oMeasured);
          #ifdef DEBUG_
            Serial.println(" FLOOR aH2o WET");
          #endif  
        }else{
          oled.print("DRY");
          oled.print(aH2oMeasured);
          #ifdef DEBUG_
            Serial.print(" FLOOR aH2o DRY; aH2oMeasured = ");Serial.println(aH2oMeasured);
          #endif  
        }
      }
      
      if(pirs[sensorID]==1){
          oled.print(" m");
          oled.print(sensorData.pir);
          #ifdef DEBUG_
            Serial.print(" MOTION = "); Serial.println(sensorData.pir);
          #endif  
      }
      
      if(doors[sensorID]==1){
        if(digitalRead(pinDoor)==1) {
          oled.print(" SH");
          #ifdef DEBUG_
            Serial.println(" DOOR SHUT");
          #endif  
        }else{
          oled.print(" OP");
          #ifdef DEBUG_
            Serial.println(" DOOR OPEN");
          #endif  
        }
        oled.print(sensorData.doorCount);
      }
    #endif  // oledFormat2
    
    oled.println("  ");  
  #endif    //OLED_
}

//*********************************
static int displayTimeIsUp(long msec){    //Read data at timed intervals  
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//*********************************
void displayVersion(){  
  //display version and blink onboard led 5 sec
  if (bootCount ==1) {
    #ifdef OLED_
      //char buf0[6],buf1[6];
      oled.clear(); 
      //oled.setFont(fixed_bold10x15);
      oled.setFont(Arial_14);
      oled.println(VERSION);      
    #endif
    #ifndef BATTERY
      blinkBoardLED(5);    // blink LED for 5 sec for feedback and to give user time to activate serial console   
    #endif
  }
}  

//*********************************
static int espNowTimeIsUp(long msec){    
  //espNow send timeout timer
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
}

//*********************************
//***ESP NOW One to One Callback function***************  
#ifdef ESPNOW_1TO1_  
  void ESPNOW_1to1_OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //callback verifies successful receipt after message was sent (see loop below for sendEspNow_1to1 function)
    #ifdef DEBUG_
      Serial.println(F("*****ESPNOW_1to1_OnDataSent***** "));
      Serial.print(F("     Last Packet Send Status: "));
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif
    if(status==0)
    {
      //data sent success; reset count parameters
      sensorData.doorCount = 0;
      sensorData.pir = 0;
      sensorData.pirCount = 0;
      
      sensorData.sendFailures = sendFailureCount;
      sendFailureCount = 0;

      espNowStatus=2; // 0=not started; 1 = in process; 2=completed
      if(awakeSeconds==0){
        napTime();
      } 
    
    } else {
       
      sendFailureCount ++;
      Serial.print("     sendFailureCount = ");Serial.println(sendFailureCount);
      if (sendFailureCount >= espNowAttemptsAllowed)
      { 
        sensorData.sendFailures = sendFailureCount;  
        espNowStatus=2;  // 0=not started; 1 = in process; 2=completed
      
        if(awakeSeconds==0){
          napTime();
        }             
      }else{
        sendEspNow_1to1();
      }   
    } 
  }
#endif

//*********************************
String getAh2o(){
  #ifdef DEBUG_
    Serial.println(F("*****getAh2o*****"));
  #endif  
  return String(sensorData.aH2o);
}

//***********************************
String getDh2o(){
  #ifdef DEBUG_
    Serial.println(F("****getDh2o*****"));
  #endif  
  return String(sensorData.dH2o);
}

//***********************************
String getTemperature() {  
  //http GET retrieves latest valid reading
  #ifdef DEBUG_
    Serial.println(F("*****getTemperature*****"));
    Serial.println(tem);
  #endif  
  //urnOnBoardLED();  
  return tem;
}

//***********************************
String getDoor(){
  #ifdef DEBUG_
    Serial.println(F("*****getDoor*****"));
    Serial.print("     door = ");Serial.println(sensorData.door);
  #endif  
  return String(sensorData.door);
}

//*********************************
String getPhotoCell(){ 
  #ifdef DEBUG_
    Serial.print(F("****getPhotoCell*****")); 
    Serial.print("     lux: ");Serial.println(sensorData.lux);
  #endif                
// if (door==0){
//    turnOffBoardLED(); 
//  }else{
//    turnOnBoardLED();
//  }
  return String(sensorData.lux);
}

//***********************************
String getHumidity() {  
  //http GET retrieves last reading
  #ifdef DEBUG_
    Serial.println(F("*****getHumidity*****"));
    Serial.println(hum);
  #endif  
//  turnOffBoardLED();
  return hum;
}

//***********************************
String getPressure() {   
  //http GET retrieves last reading
  #ifdef DEBUG_
    Serial.println(F("*****getPressure*****"));
    Serial.println(pres);
  #endif  
  return pres;
}

//**************************************
#ifdef GATEWAY_
//Determine the active wifi channel being used by the wifi gateway.
//Credits: https://github.com/m1cr0lab-esp32/esp-now-network-and-wifi-gateway
//
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
            #ifdef DEBUG_  
              Serial.print(F("*****getWifiChannel***** = "));Serial.println(WiFi.channel(i));
            #endif
            return WiFi.channel(i);
          }
      }
  }
  return 0;
}
#endif

//*******************************
void napTime(){
  setupWakeupConditions();  //set up interrupt timer or if door or PIR state changes
  #ifdef DEBUG_             // issue sleep message if #define DEBUG_ is enabled
    delay(2000);            // allow time for prior msgs to screen
    Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
    delay(2000);
  #endif
  ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);  //go to sleep for sleepSeconds 
}
      
//************************************
void printSensorData(){
  if (printSensorDataFlag){
    #ifdef DEBUG_  
      Serial.println(F("*****printSensorData*****"));
      Serial.println(F("id      temp    hum    pres    lux    aH2o    dH2o    door   doorCount  sonic   pir  pir#   Fails"));    
      Serial.print(sensorData.id);Serial.print(F("\t"));
      Serial.print(sensorData.temperature);Serial.print(F("\t"));
      Serial.print(sensorData.humidity);Serial.print(F("\t"));
      Serial.print(sensorData.pressure);Serial.print(F("\t"));
      Serial.print(sensorData.lux);Serial.print(F("\t"));
      Serial.print(sensorData.aH2o);Serial.print(F("\t"));
      Serial.print(sensorData.dH2o);Serial.print(F("\t"));
      Serial.print(sensorData.door);Serial.print(F("\t"));
      Serial.print(sensorData.doorCount);Serial.print(F("\t"));
      Serial.print(sensorData.sonic);Serial.print(F("\t"));
      Serial.print(sensorData.pir);Serial.print(F("\t"));
      Serial.print(sensorData.pirCount);Serial.print(F("\t"));
      Serial.print(sensorData.sendFailures);Serial.println(F("\t"));
      printSensorDataFlag = 0;
    #endif
  }  
}

//********************************
void printWakeupID(uint8_t wakeupID){
  #ifdef DEBUG_ 
    Serial.print(F("*****printWakeupID***** ---> ")); 
    switch(wakeupID){
      case 0 : Serial.println(F("RTC_IO = door")); break;
      case 1 : Serial.println(F("RTC_CNTL = PIR")); break;
      case 2 : Serial.println(F("timer")); break;
      case 3 : Serial.println(F("touchpad")); break;
      case 4 : Serial.println(F("ULP program")); break;
      default : Serial.println(F("default = restart")); break;
    }
  #endif    
}

//***********************************
void readAh2o(){
  //Read analog value and convert to whole number 0-5
  if(h2os[sensorID]==1){
    #ifdef DEBUG_
      Serial.print(F("*****readAh2o***** "));
    #endif  

    //sensorData.aH2o = 2.44*(4095-analogRead(pinAh2o))/100;   //whole number 1-99 %
    aH2oMeasured =  touchRead(pinAh2o);
    if(aH2oMeasured > 5){
      aH2oMeasured = 5;  
    }
    
    sensorData.aH2o = 5 - aH2oMeasured;
    reportFlag = true;
    haPublishFlag = true; 
    printSensorDataFlag = true;
  
    #ifdef DEBUG_
      Serial.println(sensorData.aH2o);//Serial.println(F("% "));
    #endif
  }
}

//***********************************
void readDh2o(){
  if(h2os[sensorID]==1){
    #ifdef DEBUG_
      Serial.print(F("*****readDh2o***** "));
    #endif  

    //sensor reads 1 when dry, 0 when wet
    //we want 0=dry (normal status)
    //sensorData.dH2o = !digitalRead(pinDh2o);

    #ifdef DEBUG_
      Serial.println(sensorData.dH2o);
    #endif
  }  
}

//***********************************
void readDoor(){
  if (doors[sensorID]==1){

    #ifdef DEBUG_
      Serial.print(F("*****readDoor*****"));
      Serial.print(F("     pinDoor : "));Serial.print(digitalRead(pinDoor));
      //Serial.print(F("     priorDoor : "));Serial.print(sensorData.door);
      Serial.print(F("     doorCount: "));Serial.println(sensorData.doorCount);
    #endif
    
    //door = 1 when closed, 0 when open due to pullup resister
    //we invert to report 0 = closed so report csv shows all 0 for normal state
    if(sensorData.door == digitalRead(pinDoor)){  //if door state changed
        sensorData.door = !digitalRead(pinDoor); //report door state 0 if closed, 1 if open
        sensorData.doorCount++;                  //increment doorCount
        reportFlag = true;                       //report right away 
        haPublishFlag = true;
        printSensorDataFlag = true;
    }
  }  
  #ifdef DEBUG_
    //Serial.print("Exit door: ");Serial.println(sensorData.door);
  #endif
}

//*********************************
void readHumidity() {  
  //valid data updates sensorData.humidity and hum; invalid updates only hum with  "--"
  if (hums[sensorID]==1){  
    #ifdef DEBUG_
      Serial.print(F("*****readHumidity***** "));
    #endif  
    float x=0;
    #ifdef AHT10_
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
      x=humidity.relative_humidity;  
    #endif  
    #ifdef DHT_
      x = (dht.readHumidity());
    #endif
    #ifdef BME_
      x = (bme.readHumidity());
    #endif
    #ifdef SHT20_
      x = (sht20.readHumidity());  
    #endif
    hum=String(x);
    if (hum == "nan") {hum = "--";
     }else{ 
      sensorData.humidity = x;
      reportFlag=true;
      haPublishFlag = true;
      printSensorDataFlag = true;
    }
    #ifdef DEBUG_
      Serial.print(F("humidity, hum: "));Serial.print(sensorData.humidity);Serial.print(F(", "));Serial.println(hum);
    #endif
  }  
}

//***********************************
void readPhotoCell(){
  //Read analog value and convert to % as whole number 0-99
  if (luxs[sensorID]==1){
    #ifdef DEBUG_
      Serial.print(F("*****readPhotoCell***** "));
    #endif  
    sensorData.lux = 2.44*((4095-analogRead(pinPhotoCell))/100);  //whole number 1-99 %
    reportFlag = true;
    haPublishFlag = true;
    printSensorDataFlag = true;
  
    #ifdef DEBUG_
      Serial.print(F("     lux%, A/D: "));Serial.print(sensorData.lux); Serial.println(F("% "));
    #endif  
  }  
}

//***********************************
void readPir(){
  if (pirs[sensorID]==1){
    #ifdef DEBUG_
      Serial.print(F("*****readPir***** "));
    #endif

    //pir = 0 when no motion (normal state)
    sensorData.pir = digitalRead(pinPir); //+ sensorData.pir ;
    sensorData.pirCount=sensorData.pirCount+sensorData.pir;    
  }  
  #ifdef DEBUG_
    Serial.println(sensorData.pir);
  #endif
}

//*************************************
void readSonic(){
  if(sonics[sensorID]==1){
    #ifdef DEBUG_
      Serial.println(F("*****reasdSonic*****"));Serial.print(F("     entry reportFlag: "));Serial.println(reportFlag);
    #endif  
       
    //Send trigger pulse
    digitalWrite(pinSonicTrigger, HIGH);
    unsigned long time_now = micros();
    while (micros() < time_now + 10);
    digitalWrite(pinSonicTrigger, LOW);

    //measure how long for the pulse to return and calculate the distance in cm
    float duration = pulseIn(pinSonicEcho, HIGH);
    float sensorDistance = ((duration / 2) * 0.0344)/2.54;  //calculate distance in inches
    uint16_t distance = round(sensorDistance+1); //rounding up
    //priorDistance = sensorData.sonic;
    #ifdef DEBUG_
      Serial.printf("\n\nDistance = %d cm\n", distance);  Serial.printf("\n\nPrior Distance = %d cm\n", sensorData.sonic);
    #endif
    if (abs(distance-sensorData.sonic)>=6 ) {
      sensorData.sonic=distance;       
      reportFlag = true;
      haPublishFlag = true;
      printSensorDataFlag = true;
    }    
    
    #ifdef DEBUG_
      Serial.print(F("exit reportFlag: "));Serial.println(reportFlag);
    #endif
  }
}

//***********************************
void readPressure(){   
  //valid data updates sensorData.pressure and pres; invalid updates only pres with  "--"
  if (press[sensorID]==1){
    #ifdef DEBUG_
      Serial.print(F("*****readPressure***** "));
    #endif 
    float x = 0; 
    pres="--"; //indicate no reading
    #ifdef BME_ 
      x = bme.readPressure()/100.0;    //millibars
      pres = String(x);
      if (pres == "nan") {pres = "--";
        }else{ 
        sensorData.pressure = x;
        reportFlag=true;
        haPublishFlag = true;
        printSensorDataFlag = true;
      }
    #endif                                                                    
    #ifdef DEBUG_
      Serial.print(F("pressure, pres: "));Serial.print(sensorData.pressure);Serial.print(F(", "));Serial.println(pres);
    #endif  
  }  
}

//***********************************
void readSensors()
{
  #ifdef DEBUG_
    Serial.println(F("*****readSensors()***** "));
  #endif   
  readDoor();                   //read door status. NOTE door is wakeup trigger, also polled during loop
  //readPir();                  //read pir status. NOTE pir is wakeup trigger; updated during readWakeupID
  readAh2o();                   //read analog flood level value
  readDh2o();                   //read digital flood indicater
  readPhotoCell();              //read photocell value
  readSonic();
  readTemperature();            //valid data updates temperature and temp; invalid updates only temp with  "--"
  readHumidity();               //valid data updates humidity and hum; invalid updates only hum with  "--"
  readPressure();               //valid data updates pressure and pres; invalid updates only pres with  "--" 
}
  
//***********************************
void readTemperature() {  
  //valid data updates sensorData.temperature and temp; invalid updates only temp with  "--"
  if (temps[sensorID]==1){  
    #ifdef DEBUG_
      Serial.print(F("*****readTemperature***** "));
    #endif 
    float x = 0; 
    #ifdef AHT10_
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
      //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
      //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
      x=temp.temperature;           // centigrade
    #endif 
    #ifdef DHT_
      x=dht.readTemperature(false);  // C
    #endif
    #ifdef BME_
      x=bme.readTemperature();      // C
    #endif 
    #ifdef SHT20_
      x = sht20.readTemperature();  // C
    #endif

     x=1.8*x+32.0;                 // convert to Fahrenheit; use leading // for Centigrade,  

    tem=String(x);
    if (tem=="nan"){tem = "--";
    }else{
      sensorData.temperature = x;
      reportFlag=true;
      haPublishFlag = true;
      printSensorDataFlag = true;
    }
  
    #ifdef DEBUG_ 
      Serial.print(F("temperature, tem: "));Serial.print(sensorData.temperature);Serial.print(F(", "));Serial.println(tem);
    #endif  
  }    
}

//*************************************
uint8_t readWakeupID(){
  //read wakeup reason and return code
  #ifdef DEBUG_ 
    Serial.println(F("*****readWakupID*****"));
  #endif   

//  reportFlag=true;
//  haPublishFlag = true;
//  printSensorDataFlag = true;
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
       sensorData.door = !digitalRead(pinDoor);
       sensorData.doorCount ++ ;// tracks # changes between reports
       #ifdef DEBUG_ 
         Serial.print(F("     Door: "));Serial.print(sensorData.door);Serial.print(F(" doorCount: "));Serial.println(sensorData.doorCount);
       #endif
       return 0;
    case ESP_SLEEP_WAKEUP_EXT1 :      
       sensorData.pir = digitalRead(pinPir);  //in case we need to reset the interrupt
       sensorData.pir = 1;                    //in case we we are too slow reading the interrupt and it no longer is active
       sensorData.pirCount ++;
       return 1;
    case ESP_SLEEP_WAKEUP_TIMER : return 2;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return 3;
    case ESP_SLEEP_WAKEUP_ULP : return 4;
    default : return 5;
  }    
}

//***********************************
  void restartIfTime(){
  #ifdef DEBUG_ 
    Serial.print(F("*****restartIfTime***** bootCount = "));Serial.print(bootCount);Serial.print(F(" \t\pir = "));
    Serial.println(sensorData.pir);  
  #endif     
  if (wakeupID==2){               //timer caused wakeup
    bootCount++; 
  }   
  if (bootCount>=bootResetCount){  //if bootCount meets threshhold AND:
    if(sensorData.pir<=3){        //no unreported pir data(allowing for a couple false hits)
      if(sensorData.door==0){     //no unreported door data
        #ifdef DEBUG_
          Serial.println("");delay (5000);
          Serial.println(F("     Rebooting.............*************.............")); 
        #endif                    
        ESP.restart();            //Reboot the esp32 to prevent memory issues
      }  
    } 
  }  
}  

//***********************************
void sendEspNow_1to1(){
  //format data and send to single ESPNOW peer if #define espnow_1to1 is enabled
  #ifdef ESPNOW_1TO1_
    #ifdef DEBUG_
      Serial.println(F("*****sendEspNow1to1***** "));
    #endif    // 
    espNowStatus=1;   // 0=not started; 1 = in process; 2=completed
    sensorData.id = sensorID;

    esp_err_t espResult = esp_now_send(SECRET_broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData)); // Send message via ESP-NOW
    #ifdef DEBUG_
      if (espResult == ESP_OK) { 
        Serial.println(F("     Sent with success"));
        delay(espNowDelay);  //delay to prevent resending prior to response
      }else{
        Serial.println(F("     Error sending the data"));  
      }
    #endif
    delay(espNowDelay);  
  #endif
}

//***********************************
static int sensorNapTime(long msec){    
  //Counter to determine if processor should go to sleep
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//*********************************
static int sensorTimeIsUp(long msec){    
  //loop() timing to read data at timed intervals  
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
}

//*********************************
void serialPrintVersion(){      //Show startup info to serial monitor if DEBUG_ enabled
  #ifdef DEBUG_              //print the following to status monitor if DEBUG_ defined:
    Serial.begin(115200);
//    turnOnBoardLED();           //turn on board LED during setup
    delay(5000);
    Serial.println(F(" "));
    Serial.print(F(APP)); Serial.print(F(VERSION));  Serial.print(F(" "));
    Serial.println(displayTitle);
    Serial.println(F("----------------------------")); 
    printWakeupID(wakeupID);
  #endif
}
  
//*********************************
void setupESPNOW_1to1(){
  //initializes espnow, registers the peer's MAC address, and registers the peer
  #ifdef DEBUG_
    Serial.println(F("*****setupESPNOW_1to1*****"));
  #endif
  #ifdef ESPNOW_1TO1_
    sensorData.id = sensorID;
    WiFi.mode(WIFI_STA);

    #ifdef GATEWAY_
      //Use the active wifi channel being used by the wifi gateway.
      //Credits: https://github.com/m1cr0lab-esp32/esp-now-network-and-wifi-gateway
      
      int32_t channel = getWiFiChannel(SECRET_WIFI_SSID);
      //WiFi.printDiag(Serial); // Verify channel number before
      if(channel >= 1){
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
      }
      //WiFi.printDiag(Serial); // verify channel change
    #endif
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      #ifdef DEBUG_
        Serial.println(F("Error initializing ESP-NOW"));
      #endif  
      return;
    }
  
    // register for Send CB to get the status of Trasnmitted packet
    esp_now_register_send_cb(ESPNOW_1to1_OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, SECRET_broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      #ifdef DEBUG_
        Serial.println(F("Failed to add peer"));
      #endif
      return;
    }
  #endif
}

//*********************************
void setupOledDisplay(){
  //setup OLED display on I2C bus
  #ifdef DEBUG_
    Serial.println(F("*****setupOledDisplay*****"));
  #endif
  #ifdef OLED_
    Wire.begin();  
    #if OLED_RESET  >= 0
      oled.begin(&Adafruit128x64, I2C_ADDRESS, OLED_RESET);
    #else 
      oled.begin(&Adafruit128x64, I2C_ADDRESS);
    #endif
  #endif 
}

//*********************************
void setupOTA() {
  #ifdef OTA_

  #endif
}

//*********************************
void setupPinModes(){                
  //Set Pin Modes INPUT / OUTPUT / PULLUP
  #ifdef DEBUG_
    Serial.println(F("*****setupPinModes*****"));
  #endif   
  pinMode(pinBoardLED,OUTPUT);
  if (doors[sensorID]==1){
    pinMode (pinDoor, INPUT);
  }  
  if (pirs[sensorID]==1){
      pinMode (pinPir, INPUT);
  }
  if (luxs[sensorID]==1){
    pinMode (pinPhotoCell, INPUT);
  } 

  if (sonics[sensorID]==1){
    pinMode (pinSonicTrigger, OUTPUT);
    pinMode (pinSonicEcho, INPUT);
  }   
/*  
  if (h2os[sensorID]==1){  
    #ifdef ESP32_
      pinMode (pinAh2o, INPUT);
      pinMode (pinDh2o, INPUT_PULLUP);
    #endif
    #ifdef ESP8266_
      pinMode (pinDh2o, INPUT);
    #endif  
  }
*/    
}

//*********************************
void setupSensors(){
  //initializes sensors
  #ifdef DEBUG_
    Serial.println(F("*****setupSensors*****"));
  #endif  
  #ifdef AHT10_
    aht.begin();
    delay(100);
  #endif 
  #ifdef BME_
    #define SEALEVELPRESSURE_HPA (1013.25)
    bool bme280=bme.begin(0x76);
    delay(100);
    if (!bme280){
       #ifdef DEBUG_
         Serial.println(F("Failed to initiate bme280"));
       #endif
    }
  #endif
  #ifdef DHT_
    dht.begin();
    delay(1000);
  #endif
  #ifdef MCP9808_      
    if (!tempsensor.begin()) {
      #ifdef DEBUG_
         Serial.println(F(" Couldn't find the MCP9808!"));
      #endif  
    } 
  #endif
  #ifdef SHT20_
    sht20.initSHT20();
    delay(100);
    //Status: End of battery, Heater enabled, Disable OTP reload result: no,no,yes
    #ifdef testMode
      sht20.checkSHT20();
    #endif  
 #endif   
}

//*********************************
void setupWakeupConditions(){
  //set wakeup conditions based on door & PIR sensor
  #ifdef DEBUG_
    Serial.println(F("*****setupWakeupConditions*****"));
    if(doors[sensorID]==1){          //if this sensor platform has a door sensor:    
      Serial.print(F("     door --> "));Serial.println(!digitalRead(pinDoor));
    }
    if(pirs[sensorID]==1){                  //if this sensor platform has a pir sensor:
      Serial.println(F("    pir --> 1"));//Serial.println(sensorData.pir);
    }  
  #endif 

  //Set up door to cause wakeup when opened or closeed
  if(doors[sensorID]==1){          //if this sensor platform has a door sensor:
    if (digitalRead(pinDoor)==1){  //determine wakeup condition based on door closed or open
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0); //0 = High OPEN 1 = Low CLOSED
     }else{
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,1);  
    } 
  }  

  //set up pir to cause wakeup when movement detected
  if(pirs[sensorID]==1){                  //if this sensor platform has a pir sensor:
    #ifndef d1MiniESP32_
      #ifdef ESP32_
        //#define wakeupBitmask 0x4000000000  //wakeup on pir gpio 39 full esp32 ?? 4 sb 8?
        #define wakeupBitmask 0x8000000000  //wakeup on pir gpio 39 full esp32 
      #endif
    #endif
    #ifdef d1MiniESP32_      
      #define wakeupBitmask 0x400000000    //wakeup on pir gpio 34 d1 mini esp32
    #endif  
    esp_sleep_enable_ext1_wakeup(wakeupBitmask,ESP_EXT1_WAKEUP_ANY_HIGH);
  }  
}      

//*************************************
void setupWifiServer(){
  //Initializes WIFI server for access to clients that issue GET commands to retrieve sensor data
  #ifdef HTTPGET_
    #ifdef DEBUG_
      Serial.println(F("*****setupWifiServer*****"));
      Serial.print(F("     sensorID: "));Serial.println(sensorID);
      Serial.print(F("     SSID: "));Serial.println(ssid);
    #endif
    //WiFi.softAP(ssid, password);   //use this if you want a passsword
    WiFi.softAP(ssid);   // OMIT password for open AP 
    delay(100);
    //IPAddress Ip(192,168,4,sensorID);
    IPAddress Ip(192,168,4,1);
    IPAddress NMask(255,255,255,0);
    WiFi.softAPConfig(Ip,Ip,NMask);
    
    IPAddress IP = WiFi.softAPIP();
    #ifdef DEBUG_
      Serial.print(F("AP IP address: "));Serial.println(IP);
    #endif 
    //server.on("/payload", HTTP_GET, [](AsyncWebServerRequest *request){
    //  request->send_P(200, "text/plain", getPayload().c_str());  
    //});  
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getTemperature().c_str());
    });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getHumidity().c_str());
    });
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getPressure().c_str());
    });
    server.on("/lux", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getPhotoCell().c_str());
    });
    server.on("/ah2o", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getAh2o().c_str());
    });
    server.on("/dh2o", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getDh2o().c_str());
    });
    server.on("/door", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getDoor().c_str());
    });
    //server.on("/displayStatus1", HTTP_GET, [](AsyncWebServerRequest *request){
    //  request->send_P(200, "text/plain", displayStatus().c_str());
    //});
    server.begin();       // Start server
  #endif
}

//*************************************
void turnOnBoardLED(){              
  // Turn Board LED on
  #ifdef testMode
    Serial.println(F("*****turnOnBoardLED*****"));
  #endif    
  #ifdef ESP8266_
    digitalWrite(pinBoardLED, LOW);
  #endif  
  #ifdef ESP32_
    digitalWrite(pinBoardLED, HIGH);
  #endif  
} 

//*********************************    
void turnOffBoardLED(){             
  // Turn board LED off
  #ifdef testMode
    Serial.println(F("*****turnOffBoardLED*****"));
  #endif  
  #ifdef ESP8266_
    digitalWrite(pinBoardLED, HIGH);
  #endif  
  #ifdef ESP32_
    digitalWrite(pinBoardLED, LOW);
  #endif 
}

//HA stuff:
//************************************************************WiFi Stuff

//*********************************   
void setupWifi(){
  #ifdef HA_
    #ifdef DEBUG_
      Serial.println(F("*****setupWiFi*****"));
    #endif
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      #ifdef DEBUG_
        Serial.print(F("."));
      #endif
    }
    #ifdef DEBUG_
      Serial.print(F("     WiFi connected..  IP Address: "));
      Serial.println(WiFi.localIP());
    #endif
  #endif  
}

//*********************************   
void connectToWifi() {
  #ifdef HA_
    #ifdef DEBUG_
      Serial.println(F("*****ConnectToWiFi*****"));
    #endif
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  #endif
}

//*********************************   
 #ifdef HA_

//*********************************   
void WiFiEvent(WiFiEvent_t event) {
    #ifdef DEBUG_
      Serial.print("     WiFiEvent Event: "); Serial.print(event);
    #endif   
  switch(event) {
    case 0:
      #ifdef DEBUG_
        Serial.println(F("     WiFi ready"));
      #endif
      break;
    case 1:
      #ifdef DEBUG_
        Serial.println(F("     Finished scanning AP"));
      #endif
      break;
    case 2:
      #ifdef DEBUG_
        Serial.println(F("     WiFi Station Start"));
      #endif
      break;
    case 3:
      #ifdef DEBUG_
        Serial.println(F("     WiFi Station stop"));
      #endif
      break; 
    case 4:
      #ifdef DEBUG_
        Serial.println(F("     Wifi Station connnected to AP"));
      #endif
      break;       
    case SYSTEM_EVENT_STA_DISCONNECTED:  // code 5
      #ifdef DEBUG_
        Serial.println(F("     WiFi lost connection"));
      #endif
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
    case 6:
      #ifdef DEBUG_
        Serial.println(F("     WiFi STA auth mode changed"));
      #endif
      break;  
    case SYSTEM_EVENT_STA_GOT_IP:  //code 7
      #ifdef DEBUG_
        Serial.print(F("     WiFi connected..  IP Address: "));
        Serial.println(WiFi.localIP());
      #endif
      connectToMqtt();
      break;
    case 8:
      #ifdef DEBUG_
        Serial.println(F("     WiFi STA lost IP"));
      #endif
      break;
    default:
      #ifdef DEBUG_
        Serial.println(F(" "));
      #endif
      break;
  }
}
  #endif

//**************************************************************************MQTT Stuff  

//*********************************   
static int mqttTimeIsUp(long msec){    //Read data at timed intervals  
  static unsigned long previousMillis = millis();   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//*********************************   
static int mqttTimeIsUp_(long msec){    //Read data at timed intervals  
  static unsigned long previousMillis = millis();   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//*********************************   
void setupMQTT(){
    #ifdef HA_
      #ifdef DEBUG_
        Serial.println(F("*****setupMQTT*****"));
      #endif
      mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
      wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
      //event functions
      WiFi.onEvent(WiFiEvent);
      mqttClient.onConnect(onMqttConnect);
      mqttClient.onDisconnect(onMqttDisconnect);
      mqttClient.onSubscribe(onMqttSubscribe);
      mqttClient.onUnsubscribe(onMqttUnsubscribe);
      mqttClient.onMessage(onMqttMessage);
      mqttClient.onPublish(onMqttPublish);
      //MQTT Client login settings
      mqttClient.setServer(SECRET_MQTT_HOST, SECRET_MQTT_PORT);
      mqttClient.setCredentials(SECRET_MQTT_USER_NAME, SECRET_MQTT_USER_PWD);
      connectToWifi();//???
      
    const long mqttSeconds_ = 10;   //Allow 10 seconds for MQTT timeout
    while (!mqttTimeIsUp_(mqttSeconds_ * 1000)) {
    } 
    #endif
}

//*********************************   
void connectToMqtt() {
  #ifdef HA_
      #ifdef DEBUG_
        Serial.println(F("*****connectToMqtt*****"));
        Serial.println(F("     MQTT not connected yet..."));
      #endif
    mqttClient.connect();
  #endif
}

//*********************************   
void onMqttConnect(bool sessionPresent) {
  #ifdef HA_
    #ifdef DEBUG_
      Serial.println(F("*****onMqttConnect*****"));
      Serial.print("     Connected to MQTT* Session: "); Serial.println(sessionPresent);
    #endif
//publishMQTT(sensorID);

    //Serial.println("Connected to MQTT."); 
    // subscribe ESP32 to following topics
    //mqttClient.subscribe(BUTTON_TOPIC, 0); 
    //mqttClient.subscribe(TEMPERATURE_TOPIC, 0); 
    //mqttClient.subscribe(HUMIDITY_TOPIC, 0); 
    //mqttClient.subscribe(PRESSURE_TOPIC, 0);
    //mqttClient.subscribe(LUX_TOPIC, 0); 
    //mqttClient.subscribe(TEMPTHRESHHOLD_TOPIC, 0); 
  #endif
}

//*********************************   
void publishMQTT(uint8_t i){
  #ifdef HA_
  
  if (haPublishFlag){
    #ifdef DEBUG_
      Serial.print(F("*****publishMQTT***** "));Serial.println(i);
    #endif
    haPublishFlag = false;

    if (temps[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"temperature");
      #ifdef DEBUG_
        Serial.print("    ");Serial.print(topic);Serial.print("/");Serial.println(String(sensorData.temperature));
      #endif  
      mqttClient.publish(topic, 0, false, (String(sensorData.temperature)).c_str());
    }
    if (hums[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"humidity");
      mqttClient.publish(topic, 0, false, (String(sensorData.humidity)).c_str());
    }  
    if (press[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"pressure");
      mqttClient.publish(topic, 0, false, (String(sensorData.pressure)).c_str());
    }  
    if (doors[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"door");
      mqttClient.publish(topic, 0, false, (String(sensorData.door)).c_str());
    }  
    if (doors[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"doorcount");
      mqttClient.publish(topic, 0, false, (String(sensorData.doorCount)).c_str());
    }  
    if (pirs[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"pir");
      mqttClient.publish(topic, 0, false, (String(sensorData.pir)).c_str());
    }  
    if (pirs[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"pir");
      mqttClient.publish(topic, 0, false, (String(sensorData.pirCount)).c_str());
    }  
    if (luxs[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"lux");
      mqttClient.publish(topic, 0, false, (String(sensorData.lux)).c_str());
    }  
    if (h2os[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"ah2o");
      mqttClient.publish(topic, 0, false, (String(sensorData.aH2o)).c_str());
    }  
    if (h2os[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"dh2o");
      mqttClient.publish(topic, 0, false, (String(sensorData.dH2o)).c_str());
    }  
    if (press[sensorID]==1){
      strcpy(topic,TOPTOPIC_); 
      strcat(topic,"sonic");
      mqttClient.publish(topic, 0, false, (String(sensorData.sonic)).c_str());
    }
    strcpy(topic,TOPTOPIC_); 
    strcat(topic,"sendfailures");
    mqttClient.publish(topic, 0, false, (String(sensorData.sendFailures)).c_str());

    #ifdef DEBUG_
      Serial.println(F("     publishMQTT msg completed"));
    #endif
  }
  #endif
}

  #ifdef HA_

//*********************************   
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef DEBUG_
    Serial.println(F("     Disconnected from MQTT."));
  #endif
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

//*********************************   
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  #ifdef DEBUG_
    Serial.println(F("     Subscribe acknowledged."));
  #endif
}

//*********************************   
void onMqttUnsubscribe(uint16_t packetId) {
  #ifdef DEBUG_
    Serial.println(F("     Unsubscribe acknowledged."));
  #endif  
}

//*********************************   
void onMqttPublish(uint16_t packetId) {  //does not get called
  #ifdef DEBUG_
    Serial.print("     onMqttPublish packet ID = ");Serial.println(packetId);
  #endif
  haPublishFlag = false;
}

//*********************************   
// Modify this function to handle what happens when you receive a push message on a specific topic
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
/*  
  static float reptTempF = 0;
  static float reptHumidity = 0;
  static float reptPressure = 0;
  static float reptLight = 0;
  static int reptLedState = 0;  
  String messageTemp;
  
  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }
  
  if (strcmp(topic, BUTTON_TOPIC) == 0) {
    reptLedState = messageTemp.toInt();
    //displayButtonState(reptLedState); 
  }
  else if (strcmp(topic, TEMPERATURE_TOPIC) == 0){
  reptTempF = messageTemp.toFloat();
  }
  else if (strcmp(topic, HUMIDITY_TOPIC) == 0){
     reptHumidity = messageTemp.toFloat();
  }
  else if (strcmp(topic, PRESSURE_TOPIC) == 0){
     reptPressure = messageTemp.toFloat();
  }
  else if (strcmp(topic, LUX_TOPIC) == 0){
      reptLight = messageTemp.toFloat();
      //setLightAlarm(reptLight);
  }
*/
 
}
#endif
/////////////////////////////////////
//*****       Setup()      *********
////////////////////////////////////
void setup()
{
  delay(200);                   //ESP32 Bug workaround -- don't reference rtc ram too soon!!
  setupPinModes();              //sset up pin assignments based on board in use
  wakeupID = readWakeupID();    //process wakeup sources & return wakeup indicator
  serialPrintVersion();         //Show version and startup/wakeup info to serial monitor if DEBUG_ enabled
  #ifdef DEBUG_
    Serial.println();           // Create a blank line
    Serial.println(F("_____setup()_____ "));
  #endif
  restartIfTime();              //restart ea bootCount>bootResetCount if timer wakeup else increment bootCount 
  setupOledDisplay();           //prepare the oled display if #define includeOled is enabled
  displayVersion();             //display version on OLED and blink onboard led 5 sec on bootCount=1
  turnOnBoardLED();             //illuminate the on-board LED
  //attachInterrupt(digitalPinToInterrupt(pinPir), pirInterrupt, RISING);  //in case you want to try it!
  //attachInterrupt(digitalPinToInterrupt(pinDoor), doorChangeInterrupt, CHANGE); //in case you want to try it!
  setupSensors();               //initialize sensors 
  setupMQTT();                  //if HA_ is set; delay after setup
  setupESPNOW_1to1();           //format data and send espnow msg to a single peer if #define ESPNOW_1TO1_ is enabled; 
                                //successful data transfer will cause the system to sleep if sleepSeconds > 0
  setupWifiServer();            //initialize wifi server if #define HTTPGET is enabled
  turnOffBoardLED();            //turn off the board LED when setup complete
  loopStatus=0;                 //0 - loop() print loop title and read sensors; 1; read sensors 2; do not read sensors
}

//////////////////////////////////
//*****      Loop()      *********** 
/////////////////////////////////
void loop()
{ 
  if(loopStatus ==0){         // Display loop header if this is the firsst time thru since wakeup
    #ifdef DEBUG_  
      Serial.println();       // Create a blank line
      Serial.println(F("_____loop()_____"));
    #endif
  }
  
  if(loopStatus < 2){         // Sensor measurement and reporting
    readSensors();            // read all the sensors for this device NOTE - PIR and doors arrive by interrupt!
    displayOLED();            // display latest valid sensor readings on oled display if #define OLED_ is enabled   
    printSensorData();        // print sensor data if #define DEBUG_ enabled
    publishMQTT(sensorID);    // publish to HA if enabled AND haPublishFlag = true
    sendEspNow_1to1();        // format data and send espnow msg to a single peer if #define ESPNOW_1TO1_ is enabled; 
    while (espNowStatus<=1){  // Wait for sendEspNow process to complete  // 0=not started; 1 = in process; 2=completed
      if(espNowTimeIsUp(espNowTimeOut*1000)==1){  //wait for espNow processing
        break;                // move on if timeout
      }
    }
  }
  loopStatus = 2;

  if (sleepSeconds >0 && awakeSeconds >0 && sensorNapTime(awakeSeconds*1000) ==1 ){
    napTime();                // Go to sleep if awake time has elapsed
  }

  if(sensorTimeIsUp(sensorSeconds*1000)==1){
    loopStatus=1;
  }
}                             // repeat at top of the loop
