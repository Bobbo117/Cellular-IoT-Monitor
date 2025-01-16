
const char* APP = "AmbientHUB ";
const char* VERSION = "2023 v1101";

/* 
 *  set up FL Casa 1 for HA
 *  
 */
/////////////////////////////////////////////////////////////////////////////////////
//
// AmbientHUB is a sensor HUB which collects data from up to seven sensor platforms and submits reports via cellular connection.
//   It can report to Home Assistant as well.
//
//   Application example: You have sensor platforms in different rooms, each monitoring perhaps, temperature, humidty, flood, door, 
//   and movement reporting the data to an AMBIENT_HUB. 
//
//  Cellular communication is accomplished via adafruit MQTT, Dweet.io, and/or IFTTT.com to receive reports at up to 20 email accounts, Google Sheets, and more.
//
/*////////////////////////////////////////////////////////////////////////////////////

  Copyright 2022 Robert Jessup  MIT License:
  https://github.com/Bobbo117/Cellular-IoT-Monitor/blob/main/LICENSE

  Certain code within the #ifdef ESPNOW compiler directives is adapted from the following:causes crash
  https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
   
*/////////////////////////////////////////////////////////////////////////////////////
//NOTES - 1. IF using MELife ESP32 ESP-32S Development Board, use Arduino IDE "esp32 Dev Module" or "MH ET LIVE ESP32 Devkit"
//              (tested with Botletics arduino SIM7000A shield and and-global SIM7000A boards - select BOTLETICS hardware device below)
//
//        2. IF using LillyGo SIM7000G (Recommended!), select ESP32 Wrover Module, remove on board SD card for software upload
//        
//        3. Be sure to include all libraries shown in active #include statements below. 
//           <xxx> are searched in library directories, "xxx" are searched in sketch local directory
//        
//        4. ESP LED states for debug/verification purposes:
//           -->Blinks 5 seconds at first time startup
//           -->ON at setup WiFi with a sensor
//           --OFF at kill WiFi with a ssensor
   
//////////////////////////////////////////////////////////////  
//*******     Setup secrets.h file for:        ***********//
//               a. IFTTT webhook API Key 
//               b. adafruit mqtt username & key (Optional)
//               c. wifi credentials (Optional - necessary only if wifi, home assistant, or telegram are to be used)
//               d. home assistant credentials (Optional) 
//               e. Telegram credentials (Optional)
//               f. HUB MAC Address
//////////////////////////////////////////////////////////////

      #include <secrets.h>   // secrets.h file contents follow:
  
  //  IFTTT:     
  //  #define SECRET_IFTTT_HB_URL "http://maker.ifttt.com/trigger/HB/with/key/YOUR_IFTTT_KEY_GOES_HERE"  //heartbeat event HB
 
  //  AdaFruitIO MQTT:
  //  #define SECRET_adaMQTT_USERNAME    "YOUR ADAFRUIT.IO USERNAME HERE"
  //  #define SECRET_adaMQTT_KEY         "YOUR ADAFRUIT.IO MQTT KEY HERE"
  
  //  WIFI:
  //  #define SECRET_WIFI_SSID           "Your SSID here"
  //  #define SECRET_WIFI_PASSWORD       "Your WIFI password here" 
  
  //  Home Assistant:
  //  #define SECRET_MQTT_HOST "YOUR HOME ASSISTANT URL HERE" 
  //  #define SECRET_MQTT_PORT "YOUR HOME ASSISTANT PORT HERE"
  //  #define SECRET_MQTT_USER_NAME "YOUR USERNAME HERE"
  //  #define SECRET_MQTT_USER_PWD  "YOUR PASSWORD HERE"
  
  //  Telegram BOT:
  //  #define SECRET_BOTTOKEN      "your Bot Token here (Get from Botfather)"  // your Bot Token (Get from Botfather)
  //  #define SECRET_CHAT_ID       "your chatID here"

  //  HUB MAC Address:
  //  uint8_t SECRET_broadcastAddress[] = {0x8C, 0xAA, 0xB3, 0x85, 0x6A, 0x67};  //example for Mac Address 8C:AA:B3:85:6A:67
#define ESP32_
//////////////////////////////////////////////////////////////  
//*******         Compile-time Options           ***********//
//        (Disable unwanted options with leading //)
//////////////////////////////////////////////////////////////
  #define iFTTTMode           // enables use of ifttt; get a free account at www.ifttt.com.  This will enable passing
                              // sensor values to multiple gmail accounts and to google sheets.
  #define adaMQTT             // enables use of adafruit mqtt publish; get a free account at www.adafruit.io, then 
                              // use this account to access ifttt for a more reliable experience than ifttt direct.
  #define HA                  // Enable ommunication with Home Assistant for enhanced real time sensor updates and monitoring via local wifi.
  //#define TELEGRAM          // Enable telegram if you pass comands to ESP32 via WIFI (for exxample, to close the garage door from away)
  #define dweetMode           // enables use of dweet as a backup to ifttt and adafruit.  www.Dweet.com is free.  The cellular SIM imei is used as a key to
                              // log and monitor sensor values.
   
  //*******************************   
  // 1. Select debug aids/monitors:
  //*******************************
  #define OLED_               // 64x128 pixel OLED display if hub uses one
  #define printMode           // comment out if you don't want brief status display of sensor readings, threshholds and cellular interactions to pc
  //#define testMode          // uncomment for verbose test messages to status monitor for heavy debug / verification
  #define cellularMode        //  enables a cellular connection; // = no connection (for debugging code without racking up cellular costs)
  //#define testCellular      // comment out if you don't want to enable test transmission to dweet, ifttt at initial startup

  //*****************************
  // 2. Select one CASA (reporting site which may have unique configuration):
  //*****************************
//  #define CASA_1            // Site #1 FL LillyGo
  //#define CASA_1a           // Site# 1a FL Botletics
  #define CASA_2            // Site #2 ME LillyGo2
//  #define CASA_2a           // Site# 2a ME Botletics

  #ifdef CASA_1
    char dataTag[] = "FL";   // <<< unique prefix to identify source of data >
     
    //Data Inputs - select one of these two:
    #define ESPNOW      // to receive or send by espnow, A lightweight communication protocol by which sensors push data to the hub based on MAC address
    //#define HTTPGET   // A slower system by which the HUB polls the sensor via http GET request to pull data; WIFI is a memory hog.

    #ifdef cellularMode
      //#define simSleepMode   // shut off sim modem between readings 
    #endif
    
    #define alertMode          //  enables postAlert when a measurement crosses threshhold; // = no alert
//  #define GARAGE

    //*****************
    // Timing Parameters
    //***************** 
    const int espSleepSeconds = 0; //48*60;     // or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
    const long espAwakeSeconds = 12*60;    // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
    int heartbeatMinutes = 60; //10;             // (default 10) heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less than espAwakeSeconds
                                           // IF espSleepSeconds - 0, then this is the heartbeat frequency.
    const long chillSeconds = 10;          // (not used)interval between readings if sleepSeconds = 0 slows serial monitor updates
    const long sensorSeconds = 1*60;        // interval in seconds to refresh time sensitive sensor readings 
  
    const long serialMonitorSeconds = 5*60; // (growth) interval in seconds to refresh serial monitor sensor readings (enhances readability)            
    const long samplingRateSeconds = 10;    // (growth) interval in seconds between sensor readings in alertMode
  
    #define bootResetCount 24         //reset ESP once per day

    //*****************
    // Board selection 
    //*****************
    
    // #define BOTLETICS  //Use for SIM7000A boards including Botletics shield connected to ME LIFE ESP32 or equivalent
    #define LILLYGO  //Use for Lillygo TT Go SIM7000G board with on-board wrover ESP32 
  
    //*****************
    // Temperature sensor 
    //****************
    //#define AHT10_    // Adafruit AHT10  <--GARAGE
    #define BME_        // BME280 temperature, humidity, pressure sensor
    //#define DHT_      // DHT11,21,22, etc. temperature, humidity sensors
    //#define MCP9808_  // botletics shield temperature sensor
    //#define SHT20_    // DFRobot SHT20
  #endif  //CASA_1

//***************************************************************************************
  #ifdef CASA_2
    char dataTag[] = "ME";   // <<< unique prefix to identify source of data >
   // #define BOTLETICS  //Use for SIM7000A boards including Botletics shield connected to ME LIFE ESP32 or equivalent
    #define LILLYGO  //Use for Lillygo TT Go SIM7000G board with on-board wrover ESP32 
         
    //Data Inputs:
    #define ESPNOW      // to receive or send by espnow, A lightweight communication protocol by which sensors push data to the hub based on MAC address
    //#define HTTPGET   // A slower system by which the HUB polls the sensor via http GET request to pull data; WIFI is a memory hog.

    #ifdef cellularMode
      //#define simSleepMode        // shut off sim modem between readings 
    #endif
    
//  #define alertMode           //  enables postAlert when a measurement crosses threshhold; // = no alert
//  #define GARAGE

    //*****************
    // Timing Parameters
    //*****************
    
    const int espSleepSeconds = 0; //48*60;     // or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
    const long espAwakeSeconds = 12*60;    // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
    int heartbeatMinutes = 60; // 10;           // (default 10)  heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less than espAwakeSeconds
                                           // IF espSleepSeconds = 0, then this is the time between scheduled reports over the cellular network
    const long chillSeconds = 10;          // interval between readings if sleepSeconds = 0 slows serial monitor updates
    const long sensorSeconds = 1*60; //15*60;        // interval in seconds to refresh time sensitive sensor readings 
  
    const long serialMonitorSeconds = 5*60; //1*60; // (growth) interval in seconds to refresh serial monitor sensor readings (enhances readability)            
    const long samplingRateSeconds = 10;    // (growth) interval in seconds between sensor readings in alertMode
  
    #define bootResetCount 24         //reset ESP once per day (24 wakeups)

    //*****************
    // Temperature sensor 
    //****************
    #define AHT10_    // Adafruit AHT10 
    //#define BME_        // BME280 temperature, humidity, pressure sensor
    //#define DHT_      // DHT11,21,22, etc. temperature, humidity sensors
    //#define MCP9808_  // botletics shield temperature sensor
    //#define SHT20_    // DFRobot SHT20

  #endif  //CASA_2

//*****************************************************************************************
  #ifdef CASA_2a
    char dataTag[] = "M2";   // <<< unique prefix to identify source of data >
    #define BOTLETICS  //Use for SIM7000A boards including Botletics shield connected to ME LIFE ESP32 or equivalent
    //#define LILLYGO  //Use for Lillygo TT Go SIM7000G board with on-board wrover ESP32 
         
    //Data Inputs:
    #define ESPNOW      // to receive or send by espnow, A lightweight communication protocol by which sensors push data to the hub based on MAC address
    //#define HTTPGET   // A slower system by which the HUB polls the sensor via http GET request to pull data; WIFI is a memory hog.

    #ifdef cellularMode
    //#define simSleepMode        // shut off sim modem between readings 
    #endif
    
//  #define alertMode           //  enables postAlert when a measurement crosses threshhold; // = no alert
//  #define GARAGE

    //*****************
    // Timing Parameters
    //*****************
    
    const int espSleepSeconds = 0; //48*60;     // or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
    const long espAwakeSeconds = 12*60;    // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
    int heartbeatMinutes =  60;//10;           // 10 heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less than espAwakeSeconds
                                           // IF espSleepSeconds = 0, then this is the time between scheduled reports over the cellular network
    const long chillSeconds = 10;          //interval between readings if sleepSeconds = 0 slows serial monitor updates
    const long sensorSeconds = 1*60; //15*60;        // interval in seconds to refresh time sensitive sensor readings 
  
    const long serialMonitorSeconds = 5*60; // (growth) interval in seconds to refresh serial monitor sensor readings (enhances readability)            
    const long samplingRateSeconds = 10;    // (growth) interval in seconds between sensor readings in alertMode
  
    #define bootResetCount 24         //reset ESP once per day (24 wakeups)

    //*****************
    // Temperature sensor 
    //****************
    //  #define AHT10_    // Adafruit AHT10  <--GARAGE
    #define BME_        // BME280 temperature, humidity, pressure sensor
    //#define DHT_      // DHT11,21,22, etc. temperature, humidity sensors
    //#define MCP9808_  // botletics shield temperature sensor
    //#define SHT20_    // DFRobot SHT20
  

  #endif  //CASA_2a

//*****************************************************************************************

  //*****************    
  // 3. Sensor Inventory
  //*****************
  uint8_t sensorID = 0;               //HUB identifier = 0 see next comment  
  // arrays to indicate types of sensors aboard each sensor platform (1=presnt, 0 = absent)
  // HUB  sensorID=0; boards are 1,2,3.. example: temps[]={1,0,1,0} indicates hub and sensor #2 have temp sensors, sensor #1 and #3 do not.

  #ifdef CASA_1
    #define numberOfPlatforms 7         // number of sensor platforms available  + 1 for HUB
                                        // example: if hub has sensors, and there are 3 wireless platforms, numberOfPlatforms = 4
                                        //const char* location[] = {"hub","garage","sonic","hatch", "kitchen","bathroom","bathroom2,"spare"};
    uint8_t temps[] = {1,1,0,0, 1,1,1,0}, hums[]=  {1,1,0,0, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, pres[]={1,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,0,0, 0,0,0,0}, h2os[] = {0,0,0,0, 0,0,0,0}, doors[]={0,1,0,1, 0,0,0,0}, pirs[]={0,1,0,0, 1,1,1,0}, sonics[]={0,0,1,0, 0,0,0,0};
  #endif
  
  #ifdef CASA_2
    #define numberOfPlatforms 5         // number of sensor platforms available  + 1 for HUB
                                        // example: if hub has sensors, and there are 3 wireless platforms, numberOfPlatforms = 4
                                        //const char* location[] = {"hub","basement","bedroom","kitchen","bathroom","spare","spare","spare"};
    uint8_t temps[] = {1,1,1,1, 1,1,1,0}, hums[]=  {1,1,1,1, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, pres[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,0,1, 0,0,0,0}, h2os[] = {0,1,0,0, 0,0,0,0}, doors[]={0,0,1,1, 0,0,0,0}, pirs[]={0,1,0,1, 0,0,0,0}, sonics[]={0,0,0,0, 0,0,0,0};
  #endif
  #ifdef CASA_2a
    #define numberOfPlatforms 3         // number of sensor platforms available  + 1 for HUB
                                        // example: if hub has sensors, and there are 3 wireless platforms, numberOfPlatforms = 4
                                        //const char* location[] = {"hub","basement","bedroom","kitchen","bathroom","spare","spare","spare"};
    uint8_t temps[] = {1,1,1,0, 0,0,0,0}, hums[]=  {1,1,1,0, 0,0,0,0}, dbms[]= {1,0,0,0, 0,0,0,0}, pres[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,1,1, 1,0,0,0}, h2os[] = {0,1,0,0, 0,0,0,0}, doors[]={0,0,0,0, 0,0,0,0}, pirs[]={0,0,0,0, 0,0,0,0}, sonics[]={0,0,0,0, 0,0,0,0};
  #endif
  
  //*****************
  // 4. sensor data structure
  //*****************
   typedef struct platforms {  // create a definition of platformm sensors
      uint8_t id;              // id must be unique for each sender; HUB  id=0; sensor platforms are 1,2,3
      float temperature;       // F
      float humidity;          // %
      float pressure;          // mb
      uint8_t lux;             // 0-99 % of full illumination
      uint8_t aH2o;            // 0-99 % of full sensor level detection
      uint8_t dH2o;            // 0 or 1 digital readout 1=dry
      uint8_t doorCount;       // # times door opened and closed after previous cellular report
      uint8_t door;            // 0=door is closed 1=door is open  
      uint8_t pir;             // 0= no motion 1 = motion 
      uint16_t sonic;          // 0=absent 1=present
      uint8_t sendFailures;    // # failed attempts to send via espNow
      uint8_t pirCount;        // # times pir detected motion after previous cellular report
  } platforms;
  
  platforms sensorData = {0,0,0,0,0,0,0,0,0,0,0,0,0}; // Creates a structure called sensorData to receive data from sensor platforms
  platforms dataOut = {1,0,0,0,0,0,0,0,0,0,0,0,0};    // structure for sending data out
  RTC_DATA_ATTR platforms platform[numberOfPlatforms]; //stores the reaadings persistently in an array of structures
  uint8_t aH2oMeasured = 0; // sensor measurement prior to preocessing
  RTC_DATA_ATTR uint8_t priorDoor = 0; //last door status = 0 (closed) or 1 (open)
  RTC_DATA_ATTR int dbm[] = {-00,0,0,0, 0,0,0,0};      //hub signal strenth not included in the platform structure
  RTC_DATA_ATTR int16_t vbat[] = {0,0,0,0, 0,0,0,0}; //hub battery voltage (mv) not included in the platform structure
  //**********************
  uint8_t platformStatus[numberOfPlatforms];
  uint8_t sensorStatus[numberOfPlatforms];  //wifi success 1 or 0 to display : or ?? on oled header in displayStatus() or to stop at 1 reading if limited awake time
  const char* sensors[]={"local","wifi","wifi","wifi","wifi","wifi","wifi","wifi"};
     
  uint8_t postTries = 3;  //allowed data post attempts
  
  uint8_t alert[numberOfPlatforms]; // used to accumulate # sensors that have alert condition
  #define h2oThreshhold  20         // water threshhold % which will cause display or IFTTT alert if enabled
  #define luxThreshhold  45         // light threshhold % which will cause display or IFTTT alert if enabled
  #define sonicThreshhold 75        // floor = 85 
  
  #ifdef alertMode 
    #ifdef CASA_1 
      //Arrays to indicate which sensors can be used to generate an alert over the cellular network
      uint8_t iftTemps[] = {0,0,0,0,0,0,0,0}, iftHums[]={0,0,0,0,0,0,0,0}, iftLuxs[]={0,0,0,0,0,0,0,0}, iftH2os[]={0,0,0,0,0,0,0,0}, iftDoors[]={0,1,0,0,0,0,0,0}, iftPirs[]={0,0,0,0,0,0,0,0};
      
      #define tempHighLimit 90            // temperature upper limit which will cause alert
      #define tempLowLimit 45             // temperature lower limit which will cause alert
      #define tempIncrement 5             // Increment by which temp threshholds are moved to prevent multiple alarms caused by temp fluctuations around the threshhold
    #endif  //CASA_1
    #ifdef CASA_2 
      //Arrays to indicate which sensors can be used to generate an alert over the cellular network
      uint8_t iftTemps[] = {0,0,0,0, 0,0,0,0}, iftHums[]={0,0,0,0, 0,0,0,0}, iftLuxs[]={0,0,0,0, 0,0,0,0}, iftH2os[]={0,0,0,0, 0,0,0,0}, iftDoors[]={0,1,0,0, 0,0,0,0}, iftPirs[]={0,0,0,0, 0,0,0,0};
       
      #define tempHighLimit 90            // temperature upper limit which will cause alert
      #define tempLowLimit 45             // temperature lower limit which will cause alert
      #define tempIncrement 5             // Increment by which temp threshholds are moved to prevent multiple alarms caused by temp fluctuations around the threshhold
    #endif  //CASA_2
    #ifdef CASA_2a 
      //Arrays to indicate which sensors can be used to generate an alert over the cellular network
      uint8_t iftTemps[] = {0,0,0,0, 0,0,0,0}, iftHums[]={0,0,0,0, 0,0,0,0}, iftLuxs[]={0,0,0,0, 0,0,0,0}, iftH2os[]={0,0,0,0, 0,0,0,0}, iftDoors[]={0,1,0,0, 0,0,0,0}, iftPirs[]={0,0,0,0, 0,0,0,0};
       
      #define tempHighLimit 90            // temperature upper limit which will cause alert
      #define tempLowLimit 45             // temperature lower limit which will cause alert
      #define tempIncrement 5             // Increment by which temp threshholds are moved to prevent multiple alarms caused by temp fluctuations around the threshhold
    #endif  //CASA_2a
        
    //Set initial conditions
    RTC_DATA_ATTR int tempHighThresh[]={tempHighLimit,tempHighLimit,tempHighLimit,tempHighLimit};  //persistently keeps track of adaptive high temp threshhold
    RTC_DATA_ATTR int tempLowThresh []={tempLowLimit,tempLowLimit,tempLowLimit,tempLowLimit};      //persistently keeps track of adaptive low temp threshhold
    int tempHighThreshReset[numberOfPlatforms];
    int tempLowThreshReset [numberOfPlatforms];

    RTC_DATA_ATTR int luxReportedLow[]={1,1,1,1, 1,1,1,1};            
    RTC_DATA_ATTR int luxReportedHigh[]={0,0,0,0, 0,0,0,0};
    int luxReportedHighReset[numberOfPlatforms];
    int luxReportedLowReset[numberOfPlatforms];
    
    RTC_DATA_ATTR int h2oReportedLow[]={1,1,1,1, 1,1,1,1};
    RTC_DATA_ATTR int h2oReportedHigh[]={0,0,0,0, 0,0,0,0};
    int h2oReportedHighReset[numberOfPlatforms];
    int h2oReportedLowReset[numberOfPlatforms];
    
    
    RTC_DATA_ATTR int doorReportedLow[]={1,1,1,1, 1,1,1,1};
    RTC_DATA_ATTR int doorReportedHigh[]={0,0,0,0, 0,0,0,0};
    int doorReportedHighReset[numberOfPlatforms];
    int doorReportedLowReset[numberOfPlatforms];

    RTC_DATA_ATTR int pirReportedLow[]={1,1,1,1, 1,1,1,1};
    RTC_DATA_ATTR int pirReportedHigh[]={0,0,0,0, 0,0,0,0};
    int pirReportedHighReset[numberOfPlatforms];
    int pirReportedLowReset[numberOfPlatforms];   

  #endif  

  //*****************
  // 5. Miscellaneous
  //*****************

  uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds  
  RTC_DATA_ATTR int bootCount = 0;    //Incremented after first setup so delay only happen on initial startup gives time for user to open Status monitor 
  char buf[200],buf2[20];             //formatData() sets up buf with data to be sent to the cloud. Buf2 is working buffer.
  char prelude[20];                   //included after datatag in formatData() for special msgs such as "GARAGE OPEN!")
  
  uint8_t type;                       //fona.type
  char imei[16] = {0};                // 16 character buffer for IMEI
  char URL[255];                      // buffer for request URL
  char body[255];                     // buffer for POST 
  uint8_t wakeupID;                   //reason sensor wode up; see readWakeupID 
  bool reportFlag = true;             //????

  #define OPEN 1
  #define PRESENT 1
  #define MOTION 1

  //*****************
  // 6. Timer stuff
  //*****************
  
  extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h" 
  }
  unsigned long currentMillis =0;
  unsigned long priorMillis=0; 
  
  //*****************
  // 7.  ESP32 PIN DEFINITIONS
  //    note: esp32 set pin 15 low prevents startup log on Serial monitor - 
  //          good to know for battery operation
  //    From youtube #363 - ESP Pri 1 pins - The guy with the swiss accent:
  //      Pri1: GPIO 4*,5,16,17,18,19,23,32,33; (16,17 not available on Wrover)
  //      Pri2 (input only, no internal pullup) GPIO 34,35,36,39.
  //      Do not use ADC2 (GPIO 2,4*,12-15, 25-27 if using wifi
  //      Reserve GPIO 21,22 for I2C bus
  //      Can use 1,2,4,12-15, 25-27, 32-39 as wakeup source; avoid conflict w/ wifi by using only 32-39 
  //
  //*****************
  // Botletics SIM7000A Arduino Shield Connections to ESP32
  //*****************

  #ifdef BOTLETICS                      // Botletics shield or and-global module with esp32
    #define UART_BAUD   9600
    #define pinBoardLED 2               // ESP32 on board led
    #define pinFONA_RST 5               // BOTLETICS shield D7  global=S
    #define pinFONA_TX 16               // ESP32 hardware serial RX2 to shield 10 TX
    #define pinFONA_RX 17               // ESP32 hardware serial TX2 to shield 11 RX
    #define pinFONA_PWRKEY 18           // BOTLETICS shield 6
  #endif

  //*****************
  // LillyGo SIM7000G Connections to on-board Wrover ESP32
  //*****************
  
  #ifdef LILLYGO       
    #define UART_BAUD   9600
    #define pinBoardLED 12    
    #define PIN_DTR     25   
    #define pinFONA_TX  26 
    #define pinFONA_RX  27     
    #define pinFONA_PWRKEY 4 
    #define SD_MISO     2               // SD Card
    #define SD_MOSI     15              // SD Card
    #define SD_SCLK     14              // SD Card
    #define SD_CS       13              // SD Card
  #endif

  //*****************
  // 8. Pin connections used for sensors
  //*****************
  
  #define pinSDA 21                   // ESP 12C Bus SDA for temp sensor, OLED, etc
  #define pinSCL 22                   // ESP 12C Bus SCL for temp sensor, OLED, etc
  #define pinGarage 19                // garage door relay controller                             
  #define pinAh2o 32                  // analog flood sensor touchpin
  #define pinPhotoCell 34             // esp32 analog input; photocell pulled high, 
                                      // 100K reistor pulled low, node to GPIO
  #define pinPir 39                   //HC-SR501 PIR sensor                                      
  #ifdef DHT_
    #define pinDHT 32                 // DHT temperature & Humidity sensor
  #endif
  #define pinDoor 35                  //door wired to magnetic reed switch, NO & C connected 
                                      //  to pinDoor and ground; 1K pullup R to pinDoor & 3.3v
                                      //  pinDoor = 1 = closed, 0 = open.
                                      //NOTE if this pin is changed, 
                                      //  change setupWakeupConditions as well.
  #define pinSonicTrigger 23          // useful to deteermine whether car is in garage
  #define pinSonicEcho 33
  
  //*****************
  // 9. Temperature Sensor Libraries
  //*****************

  #include <Wire.h>                 // 12C
  #include <Adafruit_Sensor.h>
  #ifdef AHT10_
    #include <Adafruit_AHT10.h>
    Adafruit_AHT10 aht;
  #endif
  #ifdef BME_
    #include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
    Adafruit_BME280 bme;            // temp hum pressure sensor
  #endif  
  #ifdef MCP9808_
    #include "Adafruit_MCP9808.h"   // shield temp sensor (on 12C bus)
    Adafruit_MCP9808 tempsensor = 
         Adafruit_MCP9808();        // Create the MCP9808 temperature sensor object
  #endif
  #ifdef DHT_                       
    #include "DHT.h"                // library for DHTxx temp sensor
    #define DHTTYPE DHT11           // can also be DHT22, etc
    DHT dht(pinDHT,DHTTYPE);
  #endif 
  #ifdef SHT20_  
    #include "DFRobot_SHT20.h"
    DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
  #endif     
  
  //*****************
  // 10. OLED Libraries
  //*****************
  #ifdef OLED_
    //#define FORMAT1  //HUB only format with temp hum pres dbm sensors
    #include "SSD1306Ascii.h"     // low overhead library text only
    #include "SSD1306AsciiWire.h"
    #define I2C_ADDRESS 0x3C      // 0x3C+SA0 - 0x3C or 0x3D
    #define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
    SSD1306AsciiWire oled;
  #endif   

  //*****************
  // 11. HTTP GET Libraries
  //*****************
  
  #ifdef HTTPGET
    #ifndef WIFI_H
      #include <WiFi.h>
    #endif
    #include <HTTPClient.h>
    const char* wifiSSID[] = {"n/a","AMBIENT_1","AMBIENT_2","AMBIENT_3"};
    const char* wifiPassword[] = {"","","",""};
    
    //don't need all this, but it is convenient for debug purposes:
    const char* serverNameTemperature = "http://192.168.4.1/temperature";
    const char* serverNameHumidity = "http://192.168.4.1/humidity";
    const char* serverNamePressure = "http://192.168.4.1/pressure";
    const char* serverNameLux = "http://192.168.4.1/lux";
    const char* serverNameAh2o = "http://192.168.4.1/ah2o";
    const char* serverNameDh2o = "http://192.168.4.1/dh2o";
    const char* serverNameDoor = "http://192.168.4.1/door";
    const char* serverNamedisplayStatus = "http://192.168.4.1/displayStatus"; 
  #endif  
  
  //*****************
  // 12. FONA Library
  //*****************

  #include "Adafruit_FONA.h"  //IMPORTANT! get it from https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
  //#include <Adafruit_FONA.h>
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
  #include <HardwareSerial.h>
  HardwareSerial fonaSS(1);
  #define SIMCOM_7000                // SIM7000A/C/E/G

  //*****************
  // 13. ESPNow Libraries
  //*****************
  
  #ifdef ESPNOW
    #include <esp_now.h>
    #ifndef WIFI_H
      #include <WiFi.h>        //espNow uses its own wifi; no external wifi router is used
    #endif    
    //espNow uses its own wifi; no external wifi router is used
    #include <esp_wifi.h>
    
    #ifdef ESPNOW_1to1
      esp_now_peer_info_t peerInfo;   // Create peer interface if sending     
      int espNowAttempts=0;            //number of ESPNOW transmission attempts so far (do not adjust)
      #define espNowAttemptsAllowed 30  //number of unsuccessful attempts allowed before sleeping if sleepSeconds > 0
      int espNowDelay = 50;         //delay in msec between espnow attempts
    #endif
  #endif

  //*****************
  // 14. MQTT PARAMETERS
  //*****************

  #ifdef adaMQTT
    #include "Adafruit_MQTT.h"
    #include "Adafruit_MQTT_FONA.h"

    #define adaMQTT_SERVER      "io.adafruit.com"    
    #define adaMQTT_PORT        1883    //insecure port
    //IMPORTANT - See Secrets file setup instructions above for mqtt credential options                  
  
    // Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
    Adafruit_MQTT_FONA mqtt(&fona, adaMQTT_SERVER, adaMQTT_PORT, SECRET_adaMQTT_USERNAME, SECRET_adaMQTT_KEY);
    Adafruit_MQTT_Publish feed_temp = Adafruit_MQTT_Publish(&mqtt, SECRET_adaMQTT_USERNAME "/f/t");  
    Adafruit_MQTT_Publish feed_hum = Adafruit_MQTT_Publish(&mqtt, SECRET_adaMQTT_USERNAME "/f/h");
    Adafruit_MQTT_Publish feed_csv = Adafruit_MQTT_Publish(&mqtt, SECRET_adaMQTT_USERNAME "/f/csv"); 
    Adafruit_MQTT_Publish feed_cmd = Adafruit_MQTT_Publish(&mqtt, SECRET_adaMQTT_USERNAME "/f/cmd");  
    Adafruit_MQTT_Subscribe feed_command = Adafruit_MQTT_Subscribe(&mqtt, SECRET_adaMQTT_USERNAME "/feeds/command");

   // Adafruit_MQTT_Publish feed_gar = Adafruit_MQTT_Publish(&mqtt, SECRET_adaMQTT_USERNAME "/f/gar"); 
  #endif

  //*******************************   
  // 15. Telegram setup
  //*******************************
    #ifdef TELEGRAM
      #ifndef WIFI_H
        #include <WiFi.h>        
      #endif    
      #include <WiFiClientSecure.h>
      #include <UniversalTelegramBot.h> // Universal Telegram Bot Library written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
      #include <ArduinoJson.h>
      WiFiClientSecure client;
      UniversalTelegramBot bot(SECRET_BOTTOKEN, client);
      //Checks for new messages every 1 second.
      int botRequestDelay = 1000;
      unsigned long lastTimeBotRan;      
    #endif
    
  //*******************************   
  // 16. Home Assistant setup
  //*******************************
  #ifdef HA 
    TimerHandle_t mqttReconnectTimer; //Create two Timer Objects
    TimerHandle_t wifiReconnectTimer;
    //#define mqTTMode
    //****************wifi stuff
    #ifndef WIFI_H
      #include <WiFi.h>        
    #endif    
    #include "WiFiMulti.h"
    #include <HTTPClient.h>   
    //#define WIFI_SSID SECRET_WIFI_SSID
    //#define WIFI_PASSWORD SECRET_WIFI_PASSWORD

    //***************mqtt stuff
    #include <AsyncMqttClient.h>
    AsyncMqttClient mqttClient;  // Create a MQTT client Object
    
    #ifdef CASA_1
      //garage topics
      //#define BUTTON_TOPIC          "garage/button"
      
      #define GARAGE_TEMPERATURE_TOPIC     "garage/temperature"
      #define GARAGE_HUMIDITY_TOPIC        "garage/humidity"
      #define GARAGE_PRESSURE_TOPIC        "garage/pressure"
      #define GARAGE_DOOR_TOPIC            "garage/door"
      #define GARAGE_DOOR_COUNT_TOPIC      "garage/doorcount"
      #define GARAGE_PIR_TOPIC             "garage/pir"
      #define GARAGE_LUX_TOPIC             "garage/lux"
      #define GARAGE_AH2O_TOPIC            "garage/ah2o"
      #define GARAGE_DH2O_TOPIC            "garage/dh2o"
      #define GARAGE_SEND_FAILURES_TOPIC   "garage/sendfailures"

      #define SONIC_TOPIC           "garage/sonic"
      #define SONIC_SEND_FAILURES_TOPIC   "sonic/sendfailures"

      #define HATCH_TOPIC           "garage/hatch"
      #define HATCH_SEND_FAILURES_TOPIC   "hatch/sendfailures"

      //hub topics
      #define HUB_TEMPERATURE_TOPIC "hub/temperature"
      #define HUB_HUMIDITY_TOPIC    "hub/humidity"
      #define HUB_PRESSURE_TOPIC    "hub/pressure"
      #define HUB_SEND_FAILURES_TOPIC "hub/sendfailures"

      //kitchen topics
      #define KITCHEN_TEMPERATURE_TOPIC     "kitchen/temperature"
      #define KITCHEN_HUMIDITY_TOPIC        "kitchen/humidity"
      #define KITCHEN_PRESSURE_TOPIC        "kitchen/pressure"
      #define KITCHEN_PIR_TOPIC             "kitchen/pir"
      #define KITCHEN_SEND_FAILURES_TOPIC   "kitchen/sendfailures"

      //bathroom topics
      #define BATHROOM_TEMPERATURE_TOPIC     "bathroom/temperature"
      #define BATHROOM_HUMIDITY_TOPIC        "bathroom/humidity"
      #define BATHROOM_PRESSURE_TOPIC        "bathroom/pressure"
      #define BATHROOM_PIR_TOPIC             "bathroom/pir"      
      #define BATHROOM_SEND_FAILURES_TOPIC   "bathroom/sendfailures"

      //bathroom2 topics
      #define BATHROOM2_TEMPERATURE_TOPIC     "bathroom2/temperature"
      #define BATHROOM2_HUMIDITY_TOPIC        "bathroom2/humidity"
      #define BATHROOM2_PRESSURE_TOPIC        "bathroom2/pressure"
      #define BATHROOM2_PIR_TOPIC             "bathroom2/pir"      
      #define BATHROOM2_SEND_FAILURES_TOPIC   "bathroom2/sendfailures"
    #endif //CASA_1
    
    #ifdef CASA_2
      //hub topics
      #define HUB_TEMPERATURE_TOPIC "2hub/temperature"
      #define HUB_HUMIDITY_TOPIC    "2hub/humidity"
      #define HUB_PRESSURE_TOPIC    "2hub/pressure"
      #define HUB_SEND_FAILURES_TOPIC "2hub/sendfailures"
           
     //basement topics
     // #define BUTTON_TOPIC          "2basement/button"
      #define BASEMENT_TEMPERATURE_TOPIC     "2basement/temperature"
      #define BASEMENT_HUMIDITY_TOPIC        "2basement/humidity"
      //#define PRESSURE_TOPIC        "2basement/pressure"
      //#define DOOR_TOPIC            "2basement/door"
      //#define DOOR_COUNT_TOPIC      "2basement/doorcount"
      #define BASEMENT_PIR_TOPIC             "2basement/pir"
      #define BASEMENT_PIR_COUNT_TOPIC        "2basement/pircount"
      #define BASEMENT_LUX_TOPIC             "2basement/lux"
      #define BASEMENT_AH2O_TOPIC            "2basement/ah2o"
      #define BASEMENT_DH2O_TOPIC            "2basement/dh2o"
     // #define SONIC_TOPIC           "2basement/sonic"
      #define BASEMENT_SEND_FAILURES_TOPIC   "2basement/sendfailures"
      //#define HATCH_TOPIC           "2basement/hatch"

      //kitchen topics
      #define KITCHEN_TEMPERATURE_TOPIC     "2kitchen/temperature"
      #define KITCHEN_HUMIDITY_TOPIC        "2kitchen/humidity"
      #define KITCHEN_PRESSURE_TOPIC        "2kitchen/pressure"
      #define KITCHEN_PIR_TOPIC             "2kitchen/pir"
      #define KITCHEN_PIR_COUNT_TOPIC        "2kitchen/pircount"
      #define KITCHEN_LUX_TOPIC             "2kitchen/lux"
      #define KITCHEN_DOOR_TOPIC            "2kitchen/door"
      #define KITCHEN_DOOR_COUNT_TOPIC       "2kitchen/doorcount"
      #define KITCHEN_SEND_FAILURES_TOPIC   "2kitchen/sendfailures"
       
      //bedroom topics
      #define BEDROOM_TEMPERATURE_TOPIC     "2bedroom/temperature"
      #define BEDROOM_HUMIDITY_TOPIC        "2bedroom/humidity"
      //#define BEDROOM_LUX_TOPIC             "2bedroom/lux"
      #define BEDROOM_DOOR_TOPIC            "2bedroom/door"
      #define BEDROOM_DOOR_COUNT_TOPIC       "2bedroom/doorcount"
      #define BEDROOM_SEND_FAILURES_TOPIC   "2bedroom/sendfailures"

      //bathroom topics
      #define BATHROOM_TEMPERATURE_TOPIC     "2bathroom/temperature"
      #define BATHROOM_HUMIDITY_TOPIC        "2bathroom/humidity"
      //#define BATHROOM_LUX_TOPIC             "2bathroom/lux"
      #define BATHROOM_SEND_FAILURES_TOPIC    "2bathroom/sendfailures"
      //#define BATHROOM_PIR_TOPIC             "2bathroom/pir"   

    #endif //CASA_2

    #ifdef CASA_2a
      //hub topics
      #define HUBa_TEMPERATURE_TOPIC "2ahub/temperature"
      #define HUBa_HUMIDITY_TOPIC    "2ahub/humidity"
     //#define HUBa_PRESSURE_TOPIC    "2ahub/pressure"
      #define HUBa_SEND_FAILURES_TOPIC "2ahub/sendfailures"
           
     //basement topics
     // #define BUTTON_TOPIC          "2basement/button"
      #define BASEMENTa_TEMPERATURE_TOPIC     "2abasement/temperature"
      #define BASEMENTa_HUMIDITY_TOPIC        "2abasement/humidity"
      //#define PRESSURE_TOPIC        "2abasement/pressure"
      //#define DOOR_TOPIC            "2abasement/door"
      //#define DOOR_COUNT_TOPIC      "2abasement/doorcount"
      #define BASEMENTa_PIR_TOPIC             "2abasement/pir"
      #define BASEMENTa_PIR_COUNT_TOPIC        "2abasement/pircount"
      #define BASEMENTa_LUX_TOPIC             "2abasement/lux"
      #define BASEMENTa_AH2O_TOPIC            "2abasement/ah2o"
      #define BASEMENTa_DH2O_TOPIC            "2abasement/dh2o"
     // #define SONIC_TOPIC           "2abasement/sonic"
      #define BASEMENTa_SEND_FAILURES_TOPIC   "2abasement/sendfailures"
      //#define HATCH_TOPIC           "2abasement/hatch"

      //bathroom topics
      #define BATHROOMa_TEMPERATURE_TOPIC     "2abathroom/temperature"
      #define BATHROOMa_HUMIDITY_TOPIC        "2abathroom/humidity"
      #define BATHROOMa_LUX_TOPIC             "2abathroom/lux"
      #define BATHROOMa_SEND_FAILURES_TOPIC    "2abathroom/sendfailures"
      //#define BATHROOM_PIR_TOPIC             "2abathroom/pir"     
          
    #endif //CASA_2a  

  #endif

  /************************************
  void IRAM_ATTR doorChangeInterrupt() {  //disabled can poll door fast in loop
    //door callback function counts door changes
    sensorData.door ++; 
    #ifdef printMode
      Serial.println(F("*doorChangeInterrupt*"));
      printSensorData();
    #endif  
  }
  */
  /************************************
  void IRAM_ATTR pirInterrupt() {  //disabled due to excessive false interrupts
    //PIR callback function counts pir changes
    sensorData.pir ++; //= sensorData + digitalRead(pinPir);
    #ifdef printMode
      Serial.println(F("*pirInterrupt*"));
      printSensorData();
    #endif  
  }
  */

//*********************************
int activateCellular(){
  #ifdef printMode
    Serial.println(F("*activateCellular*"));
  #endif
  #ifdef cellularMode
    for (int i = 0;i<2;i++){       // try again if unsuccessful
      powerUpSimModule();          // Power on the SIM7000 module by goosing the PWR pin 
      delay (1000); //9/19/2000 
      if(setupSimModule()==0){       // Establish serial communication and fetch IMEI
        activateModem();             // Set modem to full functionality ready for Hologram network
//      disableGPS();

        if(activateGPRS()==0){       // Activate General Packet Radio Service
          //disableGPS();              // disable GPS to minimize power
          connectToCellularNetwork();  // Connect to cell network
          uint8_t SS = fona.getRSSI();  //read sig strength
          #ifdef printMode
            Serial.print(F("Sig Str = "));Serial.println(SS);
          #endif
          delay (5000);
          if (SS>0){                    // exit if good signal
  delay(2000);      //5000    
  readBatteryVoltage (sensorID);

            return (0);
            
          }else { 
            //Serial.println(F("bad sig str - trying again to activate Cellular****"));
          }
        }
      }
    }
  #endif  
}

//******************************
int activateGPRS(){             // Activate General Packet Radio Service
  #ifdef printMode
    Serial.println(F("*activateGPRS*")); 
    Serial.println(F("fona.enableGPRS(true)"));
  #endif
  // Turn on GPRS
  int i = 0;
  while (!fona.enableGPRS(true)) {
    //failed to turn on; delay and retry
    #ifdef printMode
        Serial.print(F("."));
    #endif
    delay(2000); // Retry every 2s
    i ++;
    if (i>3){
      #ifdef printMode
          Serial.println(F("Failed to enable GPRS"));
      #endif
      return -1;  //failure exit
    }
  }

  #ifdef printMode
      Serial.println(F("Enabled GPRS!"));
  #endif
  return 0;  //success exit
}

//*********************************
void activateModem(){              // Set modem to full functionality
  #ifdef printMode
    Serial.println(F("*activateModem*"));
    Serial.println(F("fona.setFuntionality(1)")); 
    Serial.println(F("fona.setNetworkSettings('hologram')"));
    Serial.println(F("fona.setPreferredMode(38)"));
    Serial.println(F("fona.setPreferredLTEMode(1)"));
    //Serial.println(F("setOperatingBand('CAT-M', 12)"));
  #endif  
  /*
    0 - Minimum functionality
    1 - Full functionality
    4 - Disable RF
    5 - Factory test mode
    6 - Restarts module
    7 - Offline mode
  */
  fona.setFunctionality(1);        // AT+CFUN=1
  fona.setNetworkSettings(F("hologram"));
  /*
    2 Automatic
    13 GSM only
    38 LTE only
    51 GSM and LTE only
  */
  fona.setPreferredMode(38);
  /*
    1 CAT-M  //requires least power
    2 NB-Iot
    3 CAT-M and NB-IoT
  */
  fona.setPreferredLTEMode(1);
  //fona.setOperatingBand("CAT-M", 12); // AT&T
  //fona.setOperatingBand("CAT-M", 13); // Verizon does not work FL ??
}

//*********************************
void blinkBoardLED(int sec){      // Blink board LED for sec seconds at .5 second intervals
  #ifdef testMode 
    Serial.println(F("*blinkBoardLED*"));
  #endif  
  for (int i=0;i<sec;i++){
     turnOnBoardLED();            // Turn on boaard LED .5 seconds
     delay(500);
     turnOffBoardLED();           // Turn off boaard LED .5 seconds
     delay(500);
  }
}


//*********************************
static int chillTimeIsUp(long msec){    
  //Read data at timed intervals  
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
int connectToCellularNetwork() {  
  // Connect to cell network and verify connection every 2s until a connection is made
  #ifdef printMode
    Serial.println(F("*connectToCellularNetwork*"));
  #endif
  int i=0;
  while (!readNetStatus()) {
    Serial.print(F("."));
    delay(2000); // Retry every 2s
    i++;
    if (i==5){
      #ifdef printMode
          Serial.println(F("Failed"));
      #endif    
      return -1;
    }
  }
  #ifdef printMode
    Serial.println(F("Connected to cell network!"));
  #endif
  return 0;
    
}
//*************************
void displayPlatforms(){  
  // Display each sensor platform newest readings on OLED display 
  for (uint8_t i=0; i< numberOfPlatforms; i++){  // for each sensor:
    displayStatus(i);             // update OLED display with new or prior readings      
  }
}  
//*********************************
void displayStatus(uint8_t i){
  //updates OLED display with newest sensor platform i
  #ifdef testMode 
    Serial.print(F("*displayStatus "));Serial.print(i);Serial.println(F("*"));
  #endif  

  #ifdef OLED_

      if (numberOfPlatforms==1){
        #ifdef FORMAT1  //HUB only format with temp hum pres dbm sensors
          oled.clear(); 
          //oled.setFont(Arial_bold_14);
          oled.setFont(fixed_bold10x15);
          oled.setCursor(0,0);
          oled.print("Temp: ");oled.print(int(platform[i].temperature+.5));oled.println(" F");
          oled.print("Hum:  ");oled.print(int(platform[i].humidity+.5));oled.println(" %");
          oled.print("Pres: ");oled.print(int(platform[i].pressure+.5));oled.println(" mb");
          oled.print("dbm:  ");oled.println(dbm[i]);
        #endif
      }else{  
        oled.setFont(Arial_14);
        //oled.setCursor(0,16*i);
        //oled.setCursor(i,0);
   /*     //clear the row causes flicker
        oled.setCursor(0,i*2);  //col = 0, row = 8-panel rows; we are using 16 panels
        for(uint8_t i=0;i<24;i++){
          oled.print(F(" "));
        }
        oled.println(F(""));
  */      
        oled.setCursor(0,i*2); 
        oled.print(int(platform[i].temperature + .5));
        oled.print(" ");  
        oled.print(int(platform[i].humidity + .5));
        if (sensorStatus[i] ==0){
          oled.print("?");  //indicates old or no measurement
        }else{  
          oled.print("%");  //indicates current measurement
        }  
  
        if (luxs[i]==0){//oled.print(" L"); 
        }else{
          oled.print(" L");
          oled.print( (int (platform[i].lux) )/10); 
          //oled.print(" ");
        }
        
        if (doors[i]==1){  //read current door status and count of changes
          //oled.print(" d");
          //oled.print(digitalRead(pinDoor));
          //if(digitalRead(pinDoor)==1) {
          if(platform[i].door==0){  
            oled.print(F(" s"));
          }else{
            oled.print(F(" o"));
          }      
          oled.print(platform[i].doorCount);   //can be more than 1
        } 
    
        if (h2os[i]==1){
            if (platform[i].aH2o - h2oThreshhold > 0){
              oled.print(F(" w"));
            }else{
              oled.print(F(" d"));
            }
            //oled.print(F(" "));
            oled.print(platform[i].aH2o); 
            //oled.print(F(" "));
        }
            
        if (pirs[i]==1){//oled.println("M");
          oled.print(F(" m")); 
          //oled.print(platform[i].pir);  
          oled.print(platform[i].pirCount);
        }

        if (sonics[i]==1){
          if(platform[i].sonic==1){
            oled.print(F(" p"));
          }else{          
            oled.print(F(" a"));
          }  
        }
        
        if (dbms[i]==0){//oled.print("-");
          //oled.println(""); 
        }else{
          oled.print(F(" "));
          oled.print(dbm[i]);
          oled.print(F(" dbm "));  //dont need 'dbm' because sig str is negative, ie, -85dbm, etc, but looks nicer
          //oled.print(F(" "));
        }

        if (bat[i]==0){
        }else{
          //oled.print(F(" "));
          oled.print(vbat[i]);
          oled.print(F("mv "));     
        }

       if(i==0){
        oled.print (bootCount);   
       }
        oled.println(F("    "));  //overwrite prior data on this row
      }
  #endif 
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
  //display version and blink onboard LED 5 sec
  if (bootCount ==1) {
    #ifdef OLED_
      //char buf0[6],buf1[6];
      oled.clear(); 
      //oled.setFont(fixed_bold10x15);
      oled.setFont(Arial_14);
      oled.println(VERSION);      
    #endif
    blinkBoardLED(5);    // blink LED for 5 sec for feedback and to give user time to activate serial console   
  }
}  

//***********************************
static int espAwakeTimeIsUp(long msec){      
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//***********************************
void espNowOnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // ESPNOW callback function that will transfer incoming data to array
  #ifdef ESPNOW
      char macStr[18];
      #ifdef printMode
        Serial.println(F("*espNowOnDataRecv*"));
        Serial.print(F("Packet received from: "));
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.println(macStr);
      #endif 
       
      memcpy(&sensorData, incomingData, sizeof(sensorData));
      #ifdef printMode
        Serial.printf("Board ID %u: %u bytes\n", sensorData.id, len);
      #endif

      //transfer incoming data to array
      platform[sensorData.id].id = sensorData.id;
      platform[sensorData.id].temperature = sensorData.temperature;
      platform[sensorData.id].humidity = sensorData.humidity;
      platform[sensorData.id].pressure = sensorData.pressure;
      platform[sensorData.id].lux = sensorData.lux;
      platform[sensorData.id].aH2o = sensorData.aH2o;
      platform[sensorData.id].dH2o = sensorData.dH2o;
      platform[sensorData.id].doorCount =  platform[sensorData.id].doorCount + sensorData.doorCount;
      platform[sensorData.id].door =  sensorData.door;
      platform[sensorData.id].pir = sensorData.pir;
      platform[sensorData.id].sonic = sensorData.sonic;
      platform[sensorData.id].sendFailures = sensorData.sendFailures;
      platform[sensorData.id].pirCount = platform[sensorData.id].pirCount  + sensorData.pirCount;

      publishMQTT(platform[sensorData.id].id);  // publish new data to HA if enabled
      publishMQTT(sensorID);                    // publish hub data as well
   
      #ifdef printMode    
        Serial.println(F("id      temp    hum    pres    lux    aH2o    dH2o    door   doorCount  sonic   PIR  PIR#  Fails"));
        Serial.print(sensorData.id);Serial.print("\t");
        Serial.print(sensorData.temperature);Serial.print("\t");
        Serial.print(sensorData.humidity);Serial.print("\t");
        Serial.print(sensorData.pressure);Serial.print("\t");  
        Serial.print(sensorData.lux);Serial.print("\t");     
        Serial.print(sensorData.aH2o);Serial.print("\t"); 
        Serial.print(sensorData.dH2o);Serial.print("\t"); 
        Serial.print(sensorData.door);Serial.print("\t"); 
        Serial.print(sensorData.doorCount);Serial.print("\t"); 
        Serial.print(sensorData.sonic);Serial.print("\t"); 
        Serial.print(sensorData.pir);Serial.print("\t");
        Serial.print(sensorData.pirCount);Serial.print("\t");
        Serial.println(sensorData.sendFailures);
      #endif 
      
      //printSensorData();
  #endif
}


//*********************************
//***ESP NOW One to One Callback function***************  
#ifdef ESPNOW_1to1  
  void ESPNOW_1to1_OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //callback verifies successful receipt after message was sent (see loop below for sendEspNow_1to1 function)
    #ifdef printMode
      Serial.print(F("*ESPNOW_1to1_OnDataSent* "));
      //Serial.print(F("\r\nLast Packet Send Status:\t"));
      Serial.print(F("Last Packet Send Status: "));
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? " Delivery Success" : " Delivery Fail");
    #endif
    if(status==0){
      platform[1].doorCount=0;
      //platform[0].pir=0;
      platform[1].pir=0;
      //platform[1].sendFailures=0;
      if(espSleepSeconds==0){
      }else{

        //sensorData.doorCount = 0;
        //priorDoor = 0;
        //sensorData.pir = 0;   
        //displayStatus();   

        /*  
        setupWakeupConditions();
        #ifdef printMode
          Serial.print(F(" sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif 
 //displayStatus;
        ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);
        */
      }  
    }else{
      espNowAttempts++;
      platform[1].sendFailures = espNowAttempts;

      if (espNowAttempts >= espNowAttemptsAllowed){ 
        /*
        setupWakeupConditions();  //interrupt if door state changes
        #ifdef printMode
          Serial.print(F(" sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif  
        ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);  
        */      
      }
    }
  }
#endif

//*********************************
void formatData(){                   
  //transfer comma separated sensor data to buf for transfer to the cloud
  #ifdef testMode 
    Serial.println(F("*formatData*"));
  #endif
  
  char prefix[2] = {'_',0};  //separator used between groups of similar data
//  char prefix[2] = {' ',0};  //separator used between groups of similar data
  char separator[2] = {',',0};
//  char comma[2]={',',0};  //somewhere between ifttt and gmail, commas are ignored
  char comma[2]={':',0};  
  strcpy (buf,dataTag); //lead with unique HUB data stream identifier

  int len = strlen(prelude);
  if(len>0){
    strcat (buf,prefix);    //Add prefix (identifier if the location being reported)
    strcat(buf,prelude);    //Alert hightlight such as GAR OPEN or blank
  }
   
  //process temperature
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(temps[i]==1){
      strcat(buf,separator);
      dtostrf(platform[i].temperature, 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }
  strcat(buf,"F");
  
  //process humidity
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(hums[i]==1){
      strcat(buf,separator);
      dtostrf(platform[i].humidity, 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }
//  strcat(buf,"%");
  
  //process photocell lux
  strcpy(separator,prefix);  //separator used between groups of similar data  
  strcat(buf,separator);
  for(int i=0;i<numberOfPlatforms;i++){
    if(luxs[i]==1){
      //strcat(buf,separator);
      dtostrf((platform[i].lux/10), 1, 0, buf2); 
      strcat(buf, buf2);
      //strcpy(separator,comma);
      //strcpy(separator,null);
    }
  }

  //process door incoming 1 = closed door
  strcpy(separator,prefix);  //separator used between groups of similar data  
  strcat(buf,separator);
  for(int i=0;i<numberOfPlatforms;i++){
    if(doors[i]==1){
      //strcat(buf,separator); 
      dtostrf(platform[i].door, 1, 0, buf2); 
      strcat(buf, buf2);
      //strcpy(separator,comma);
      //strcpy(separator,null);
    }    
  }

  //process door counts
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(doors[i]==1){
      strcat(buf,separator);
      dtostrf(platform[i].doorCount, 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }
  
  //process ah2o,dH2o
  strcpy(separator,prefix);  //separator used between groups of similar data  
  strcat(buf,separator);
  for(int i=0;i<numberOfPlatforms;i++){
    if(h2os[i]==1){
      //strcat(buf,separator);
      dtostrf(platform[i].aH2o, 1, 0, buf2); 
      strcat(buf, buf2);
      //strcpy(separator,comma);
      //strcat(buf,",");
      //dtostrf(platform[i].dH2o, 1, 0, buf2); 
      //strcat(buf, buf2);
      //strcpy(separator,null);    
    }
  }

  //process pir incoming 1 = person detected
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(pirs[i]==1){
      strcat(buf,separator);
      dtostrf(platform[i].pirCount, 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }

  //process signal strength
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){  
    if(dbms[i]==1){
      //strcat(buf,separator);  //let minus sogn of dbm be separator
      dtostrf(dbm[i], 1, 0, buf2); 
      strcat(buf, buf2);
       strcpy(separator,comma);
    }
  }

  //process sonic = car detection
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(sonics[i]==1){
      strcat(buf,separator);
      dtostrf(platform[i].sonic, 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }

  //process battery
  strcpy(separator,prefix);  //separator used between groups of similar data  
  for(int i=0;i<numberOfPlatforms;i++){
    if(bat[i]==1){
      strcat(buf,separator);
      dtostrf(vbat[i], 1, 0, buf2); 
      strcat(buf, buf2);
      strcpy(separator,comma);
    }
  }
  
  //append bootCount
  strcpy(separator,prefix);  //separator used between groups of similar data 
  strcat(buf,separator); 
  dtostrf(bootCount, 1, 0, buf2); 
  strcat(buf,buf2);

  #ifdef printMode
    Serial.print(F("buf: "));Serial.println(buf);
  #endif
}
//***********************************
static int heartbeatTimeIsUp(long msec){    //Read data at timed intervals  
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    return(0);
  }
} 

//****************************************
#ifdef gateway
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}
#endif


//****************************************
String httpGETRequest(const char* serverName) {
  #ifdef printMode 
    Serial.println(F("*httpGETRequest*"));Serial.println(serverName);
  #endif  
  #ifdef HTTPGET  
    HTTPClient http;
      
    // Your IP address with path or Domain name with URL path 
    http.begin(serverName);
    
    // Send HTTP POST request
    int httpResponseCode = http.GET();
    
    String payload = "--" ;
    
    if (httpResponseCode>0) {
      #ifdef printMode 
        Serial.print(F(" HTTP Response Code: "));
        Serial.print(httpResponseCode);
      #endif
      payload = http.getString();
      //x.toCharArray(payload,6);
Serial.println(payload);      
    }
    else {
      #ifdef printtMode 
        Serial.print(F(" HTTP Response (error) Code: "));
        Serial.print(httpResponseCode);
      #endif
    }
    #ifdef testMode 
      Serial.print(F(" Payload: "));Serial.println(payload);
    #endif  
    
    http.end();  //release resources
    return payload;
  #endif 
}

//************************************
void initializeArrays(){
  #ifdef testMode 
    Serial.println(F("*initializeArrays*"));
  #endif  
  for (uint8_t i=0; i<numberOfPlatforms; i++){ 
    #ifdef alertMode
      tempHighThreshReset[i] = tempHighThresh[i];  //do it this way to preserve sleep vars
      tempLowThreshReset[i] = tempLowThresh[i];
    #endif  
    sensorStatus[i]=0;  //This causes "?" to display on OLED, indicating no real data
    platformStatus[i]=0;
    platform[i].id = numberOfPlatforms+1;
  }
}

//***************************************
void killWifi(){
  #ifdef WIFI
    WiFi.disconnect();
delay(500);
    turnOffBoardLED();
    //if (testMode==1) {Serial.println("*WiFi killed*");}
  #endif  
}   

//*********************************
int8_t MQTT_connect() {
  int8_t retn = -1;  //returns 0 if connection success
  // Connect and reconnect as necessary to the MQTT server.
  #ifdef adaMQTT
    #ifdef printMode
      Serial.println(F("*MQTT_connect*"));
    #endif
    int8_t ret;
  
    // Exit if already connected.
    if (mqtt.connected()) {
      return 0;
    }
  
    #ifdef printMode
      Serial.println("Connecting to MQTT... ");
    #endif

    int8_t i = 0;
    while (i<postTries &&((ret = mqtt.connect()) != 0)) { // connect will return 0 for connected
      #ifdef printMode
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
      #endif
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
      i++;
    }
    if(i<postTries){
      retn=0;
      #ifdef printMode
        Serial.println("MQTT Connected!");
      #endif
    }else{
      #ifdef printMode
        Serial.println("MQTT Connection failed!");  
      #endif  
    } 
  #endif
  return retn;
}

//*********************************
#ifdef adaMQTT
int8_t MQTT_publish_checkSuccess(Adafruit_MQTT_Publish &feed, const char *feedContent) {
  int8_t retn = -1;  //returns 0 if connection success
    #ifdef printMode
      Serial.println(F("*MQTT_publish_checkSuccess*"));
      Serial.println(F("Sending data..."));
    #endif  
    uint8_t i=0;
    while (i<postTries &&(! feed.publish(feedContent))) {
      i++;
      delay(5000);
    }  
    if(i<postTries){
      retn=0;
      #ifdef printMode
        Serial.print(F("Publish succeeded in tries: "));Serial.println(i+1);
      #endif
    }else{
      #ifdef printMode
        Serial.println(F("Publish failed!"));  
      #endif        
    } 
 
  return retn;
}
#endif 

//************************************
void napTime(){
  if (espSleepSeconds >0 && espAwakeSeconds >0 && espAwakeTimeIsUp(espAwakeSeconds*1000) ==1 ){
    setupWakeupConditions();  //interrupt if door or PIR state changes
    #ifdef printMode
      Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
    #endif
    ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);
  }
}
      
//************************************
void postAlert(){                
  
  //Post alert to cloud if alertMode is enabled
  #ifdef printMode 
     Serial.println(F("*postAlert*"));
  #endif
  #ifdef alertMode
    if(postSensorData()==0){
      #ifdef printMode
        Serial.println(F("postSensorData Success"));
      #endif
    }else{  
      //if failed to post alert,restore adaptive thresholds to enable future alert  
      for (int i=0; i<numberOfPlatforms;i++){
        tempHighThresh[i] = tempHighThreshReset[i];
        tempLowThresh[i] = tempLowThreshReset[i];
        #ifdef printMode
          Serial.println(F("postSensorFail"));
          Serial.print(F("reseting temp threshholds to "));
          Serial.print(tempLowThresh[i]);
          Serial.print(F(" and ")) ;
          Serial.println(tempHighThresh[i]);
        #endif
        luxReportedHigh[i] = luxReportedHighReset[i];
        luxReportedLow[i] = luxReportedLowReset[i];
        h2oReportedHigh[i] = h2oReportedHighReset[i];
        h2oReportedLow[i] = h2oReportedLowReset[i];
        doorReportedHigh[i] = doorReportedHighReset[i];
        doorReportedLow[i] = doorReportedLowReset[i];
        pirReportedHigh[i] = pirReportedHighReset[i];
        pirReportedLow[i] = pirReportedLowReset[i];

        platform[i].id = i;
      }
    }  
  #endif
}

//*********************************
void postAlerts(){
  #ifdef alertMode
    uint8_t alerts = 0;             // determine whether we have one or more alerts:    
    for (uint8_t i=0;i<numberOfPlatforms;i++){  //determine if an alarm threshhold was crossed
      alerts=alerts+alert[i];
    }
    if (alerts>0){              // if we have alerts, activate cellular network and report via ifttt if alertMode set
      sendEspNow_1to1();        //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                                                               
      #ifdef cellularMode
        activateCellular(); 
        readSignalStrength(0);
        postAlert();
        displayStatus(0);             // update OLED display 
        simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode 
      #endif
    }
  #endif
}
      
//*********************************
int8_t postSensorData(){       //upload sensor data to  adaFruit, dweet.io or IFTTT

  #ifdef printMode 
    Serial.print(F("*postSensorData*"));
  #endif
  int8_t retn = -1;  //returns 0 if connection success
  formatData();      // transfer sensor data to buf
  uint8_t i;         //keeps track of attempted postings   
  uint8_t err; 
  delay(1000);
  #ifdef adaMQTT
    retn = MQTT_connect();
    
      float temp = platform[0].temperature;
      dtostrf(temp, 1, 2, buf2);
      MQTT_publish_checkSuccess(feed_temp, buf2);
      float hum = platform[0].humidity;
      dtostrf(hum, 1, 2, buf2);
      retn = MQTT_publish_checkSuccess(feed_hum, buf2);
      
      retn = MQTT_publish_checkSuccess(feed_csv, buf);
//
    #ifdef adaMQTT_Subscribe  //NEEDS WORK!

      delay(1000);
      // Wait for incoming subscription packets
      #ifdef printMode
        Serial.println(F("readSubscription begin.............")); 
      #endif
      Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqtt.readSubscription(5000))) {
        if (subscription == &feed_command) {
          #ifdef printMode
            Serial.print(F("*** Got: "));
            Serial.println((char *)feed_command.lastread);
          #endif
        }
    
        #ifdef printMode
          Serial.println(F("readSubscription end.............")); 
        #endif  
 
        //Control an LED based on what we receive from the command feed subscription!
        if (strcmp((char *)feed_command.lastread, "ON") == 0) {
          #ifdef printMode
            Serial.println(F("*** Commanded to turn on LED!"));
          #endif
          turnOnBoardLED();
          MQTT_publish_checkSuccess(feed_cmd, "ON");
          delay(5000);
        }
        else if (strcmp((char *)feed_command.lastread, "OFF") == 0) {
          #ifdef printMode
            Serial.println(F("*** Commanded to turn off LED!"));
          #endif
          turnOffBoardLED();
          MQTT_publish_checkSuccess(feed_cmd, "OFF");
          delay(5000);
        }
      }  

    #endif //adaMQTT_Subscribe 
    formatData();      // transfer sensor data to buf
  #endif  //adaMQTT

  //exit if successfully posted unless we are in setup mode and testCellular is defined
  #ifdef testCellular 
    if(bootCount<1){
      retn=-1;        
    }   
  #endif
  if (retn==0){
    for(uint8_t j=0;j<numberOfPlatforms;j++){
     // platform[j].door =  0;
      platform[j].doorCount = 0;
      priorDoor = 0;
      platform[j].pirCount = 0;
    }
    strcpy(prelude,"");
    //if(bootCount==3){goto ifttt_;}     // SORRY goto! append a daily ifttt as backup
    //if(bootCount==5){goto dweet_;}     // SORRY goto! append a daily dweet as backup
    //if(bootCount==17){goto dweet_;}    // SORRY goto! append another daily dweet as backup
    return 0;
  } 

ifttt_:  //Ifttt processing
  #ifdef iFTTTMode
    snprintf(body, sizeof(body),"{\"value1\":\"%s\"}", buf);
    #ifdef printMode
      Serial.print(F("body: ")); Serial.println(body);
    #endif 
    i=0;
    while (i < postTries && !fona.postData("POST",SECRET_IFTTT_HB_URL,body)) {
      #ifdef printMode 
        Serial.println(F("."));
      #endif
      delay(5000);
      i++;
    }
    if(i<postTries){
      retn=0;
    }
  #endif   //IFTTTMode

  //exit if successfully posted unless we are in setup mode and estCellular is defined
  #ifdef testCellular 
    if(bootCount<1){
      retn=-1;
    }   
  #endif
  if (retn==0){
    for(uint8_t j=0;j<numberOfPlatforms;j++){
     // platform[j].door =  0;
      platform[j].doorCount = 0;
      priorDoor = 0;
      platform[j].pirCount = 0;
    }
    strcpy(prelude,"");  
    return 0;
  } 

dweet_:  //dweet processing
  #ifdef dweetMode
    // GET request use the IMEI as device ID
    snprintf(URL, sizeof(URL),"http://dweet.io/dweet/for/%s?%s",imei,buf);

    #ifdef printMode
      Serial.println(F("fona.HTTP_GET_start"));
      Serial.print(F("URL: ")); Serial.println(URL);
    #endif
    uint16_t statusCode;
    int16_t length;
    i = 0;               // Count the number of attempts 
    while (i < postTries && !fona.HTTP_GET_start(URL,&statusCode, (uint16_t *)&length)) {
      #ifdef printMode
        Serial.print(F("."));
      #endif
      Serial.print(F("Dweet failure #")); Serial.println(i);
      delay(5000);
      i++; 
    }
    if (i<postTries){
      retn=0;
    }
    #ifdef printMode
      Serial.print("statusCode, length: ");Serial.print (statusCode);Serial.print(", ");Serial.println(length);
    #endif
    fona.HTTP_GET_end();  //causes verbose response AT+HTTPREAD (and? AT+HTTPTERM to terminate HTTP?)
    
    if(i<postTries){ 
    for(uint8_t j=0;j<numberOfPlatforms;j++){
     // platform[j].door =  0;
      platform[j].doorCount = 0;
      priorDoor = 0;
      platform[j].pirCount = 0;
    }  
    strcpy(prelude,"");    
    return 0;
    }
    #endif //dweetMode  
  return retn;
}

//*********************************
void postSensorHeartbeatData(){
  if (heartbeatTimeIsUp(heartbeatMinutes*60000L) ==1 ){
    publishMQTT(sensorID);
    if (espSleepSeconds ==0){
      bootCount++;
    }
    #ifdef cellularMode
      sendEspNow_1to1();          //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled;                                                         
      activateCellular();
      readSignalStrength(0);
      postSensorData();          
      displayStatus(sensorID);      // update OLED display here to update signal strength
      simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode
      #ifdef alertMode
        for (uint8_t i=0; i<numberOfPlatforms; i++){    //reset for next heartbeat cycle 
          sensorStatus[i]=0; 
        }
      #endif
    #endif
  }  
}

//*********************************
void powerUpSimModule() {                // Power on the module
  #ifdef printMode
    Serial.println(F("*powerUpSimModule*"));
  #endif

  #ifdef BOTLETICS
    digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
    delay(100);                          // 100 msec pulse for SIM7000
    digitalWrite(pinFONA_PWRKEY, HIGH);
    delay(5000);                         // give it time to take hold - long delay
  #endif
  #ifdef LILLYGO
    //matches LILLYGO_GPSTest.ino
    pinMode(pinFONA_PWRKEY,OUTPUT);
    digitalWrite(pinFONA_PWRKEY, LOW);
    delay(1000);                           // datasheet ton = 1 sec
    digitalWrite(pinFONA_PWRKEY, HIGH);
    delay(5000);
  #endif    
}     
                 
//*********************************
void printSensorData(){
  if(chillTimeIsUp(chillSeconds*1000)==1){   //slow down loop for serial monitor readability
    #ifdef printMode
      Serial.println(F("*printSensorData*"));
      Serial.println(F("id      temp    hum    pres    lux    aH2o    dH2o    door   doorCount  sonic   pir  pir#     Fails"));
      
      for (int i=0;i<numberOfPlatforms;i++){ 
        //Serial.print(platform[i].id);Serial.print("\t");
        Serial.print(i);Serial.print("\t");
        Serial.print(platform[i].temperature);Serial.print("\t");
        Serial.print(platform[i].humidity);Serial.print("\t");
        Serial.print(platform[i].pressure);Serial.print("\t");  
        Serial.print(platform[i].lux);Serial.print("\t");     
        Serial.print(platform[i].aH2o);Serial.print("\t"); 
        Serial.print(platform[i].dH2o);Serial.print("\t"); 
        Serial.print(platform[i].door);Serial.print("\t"); 
        Serial.print(platform[i].doorCount);Serial.print("\t"); 
        Serial.print(platform[i].sonic);Serial.print("\t"); 
        Serial.print(platform[i].pir);Serial.print("\t");
        Serial.print(platform[i].pirCount);Serial.print("\t");
        Serial.println(platform[i].sendFailures);
      }
    #endif
  }
}

//********************************
void printWakeupID(uint8_t wakeupID){
  #ifdef printMode 
    Serial.print(F("*printWakeupID* ---> ")); 
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

//*********************************
int processAlerts(uint8_t i) {         // set flag to post alert to iFTTT if temp outside limit and adjust temp threshholds;
                                       // also set flag if other sensors need to have changes reported.
                                       
  int flag=0;                          //indicate no alerts at start

  #ifdef alertMode  
    #ifdef printMode
      Serial.print(F("*processAlerts*")); Serial.print (i); Serial.print(F(": "));
        Serial.print(" Thresh: " + String(tempLowThresh[i]));
        Serial.print(" - " + String(tempHighThresh[i]));
        Serial.print(" lux: = " + String(luxReportedLow[i]));
        Serial.print(" - " + String(luxReportedHigh[i]));
        Serial.print(" h2o: " + String(h2oReportedLow[i]));
        Serial.print(" - " + String(h2oReportedHigh[i]));
        Serial.print(" door: " + String(doorReportedLow[i]));
        Serial.print(" - " + String(doorReportedHigh[i]));
        Serial.print(" pir: " + String(pirReportedLow[i]));
        Serial.println(" - " + String(pirReportedHigh[i]));
    #endif

    tempHighThreshReset[i] = tempHighThresh[i];
    tempLowThreshReset[i] = tempLowThresh[i];
    luxReportedLowReset[i] = luxReportedLow[i];
    luxReportedHighReset[i] = luxReportedHigh[i];
    h2oReportedLowReset[i] = h2oReportedLow[i];
    h2oReportedHighReset[i] = h2oReportedHigh[i];
    doorReportedHighReset[i] = doorReportedHigh[i];
    doorReportedLowReset[i] = doorReportedLow[i]; 
    pirReportedHighReset[i] = pirReportedHigh[i];
    pirReportedLowReset[i] = pirReportedLow[i];
        
    if (iftTemps[i]==1){
      if (platform[i].temperature - tempHighThresh[i] > 0) {
        #ifdef printMode
          Serial.print(F(" ;1-Temp exceeds ")); Serial.print (tempHighThresh[i]);
        #endif    
        tempHighThresh[i] = tempHighThresh[i] + tempIncrement;
        flag=1; //postToIfTTT();
        strcpy(prelude,"HI TEMP"); 
        goto lux;    
      }  
      if (platform[i].temperature - (tempLowThresh[i] )<0) {      
        #ifdef printMode
          Serial.print(F(" ;2-Temp is less than ")); Serial.print(tempLowThresh[i]);
        #endif
        tempLowThresh[i] = tempLowThresh[i] - tempIncrement;  
        flag=1; //postToIfTTT(); 
        strcpy(prelude,"LOW TEMP");
        goto lux;    
      }
      if (platform[i].temperature - (tempHighThresh[i] -1.5*tempIncrement)<0) {  //not sure about this
        if(tempHighThresh[i]-tempHighLimit>0){
          #ifdef printMode
            Serial.print(F(" ;3-Temp is less than ")); Serial.print(tempHighThresh[i]);  
          #endif  
          tempHighThresh[i] = tempHighThresh[i] - tempIncrement;  
          flag=1;  //post alert
          strcpy(prelude,"LOW TEMP");  
          goto lux;    
        }
      }
      if (platform[i].temperature - (tempLowThresh[i] + 1.5*tempIncrement)>0) {
        if(tempLowThresh[i]-tempLowLimit<0){
          #ifdef printMode
            Serial.print(F(" ;4-Temp exceeds ")); Serial.print (tempLowThresh[i]);  
          #endif  
          tempLowThresh[i] = tempLowThresh[i] + tempIncrement;  
          flag=1;  //post alert 
          strcpy(prelude,"HI TEMP");
          goto lux;    
        }
      }
    }  //iftTemps

lux:
    if (iftLuxs[i]==1){
      //if (testMode==1) {Serial.println(lux[i]);  }
      if(platform[i].lux - luxThreshhold > 0){
        #ifdef printMode 
          Serial.println(" lights are ON");
        #endif
        if (luxReportedHigh[i]==0) {
          luxReportedHigh[i]=1 ;
          luxReportedLow[i] = 0;
          flag=1; //post alert 
          #ifdef printMode 
            Serial.print(F("; Lights ON alert")); 
          #endif 
          strcpy(prelude,"LIGHT ON");  
        }
      }else{
        #ifdef printMode 
          Serial.println("Lights are OFF");
        #endif  
        if (luxReportedLow[i]==0) {
          luxReportedLow[i]=1 ;
          luxReportedHigh[i] = 0;
          flag=1; //post alert 
          #ifdef printMode 
            Serial.print(F("; Lights OFF alert")); 
          #endif  
          strcpy(prelude,"LIGHT OFF"); 
        }
      }
    }
    //doors:
    //door = 0 when closed, 1 when open due to pullup resister
    if(iftDoors[i]==1){
      if(platform[i].door >= 1){

         #ifdef printMode 
            Serial.print(F("; door open")); 
         #endif 
        if (doorReportedHigh[i]==0) {
          doorReportedHigh[i]=1 ;
          doorReportedLow[i] = 0;
          flag=1; //post alert 
          
            strcpy(prelude,"GAR OPEN");      
             
          #ifdef printMode 
            Serial.print(F("; door open alert")); 
          #endif 
        }
      }else{
        
         #ifdef printMode 
            Serial.print(F("; door closed")); 
         #endif 
         if (doorReportedLow[i]==0) {
            doorReportedLow[i]=1 ;
            doorReportedHigh[i] = 0;
            flag=1; //post alert 
            
              strcpy(prelude,"GAR SHUT");
            
            #ifdef printMode 
              Serial.print(F("; door closed alert")); 
            #endif 
         }
      }
    }
    
    //PIRS:
    if(iftPirs[i]==1){
      if(platform[i].pir >= 3){
         #ifdef printMode 
            Serial.print(F("; movement detected")); 
         #endif 
        if (pirReportedHigh[i]==0) {
          pirReportedHigh[i]=1 ;
          pirReportedLow[i] = 0;
          flag=1; //post alert 
          #ifdef printMode 
            Serial.print(F("; movement detected alert")); 
          #endif 
          strcpy(prelude,"PIR"); 
        }
      }else{
         #ifdef printMode 
            Serial.print(F("; movement NOT detected")); 
         #endif 
         if (pirReportedLow[i]==0) {
          pirReportedLow[i]=1 ;
          pirReportedHigh[i] = 0;
          flag=1; //post alert 
         #ifdef printMode 
            Serial.print(F("; movement stopped alert")); 
         #endif 
         strcpy(prelude,"NO PIR"); 
         }
      }
    }
    //h2o:
    if(iftH2os[i]==1){
      if(platform[i].aH2o - h2oThreshhold > 0){
        if (h2oReportedHigh[i]==0) {
          h2oReportedHigh[i]=1 ;
          h2oReportedLow[i] = 0;
          flag=1; //post alert 
          #ifdef printMode 
            Serial.print(F("; flood alert")); 
          #endif
          strcpy(prelude,"WATER");  
        }
      }else{
        if (h2oReportedLow[i]==0) {
          h2oReportedLow[i]=1 ;
          h2oReportedHigh[i] = 0;
          flag=1; //post alert 
          #ifdef printMode 
            Serial.print(F("; NO flood alert")); 
          #endif
          strcpy(prelude,"NO WATER");  
         }
      }
    }
    
    #ifdef printMode
      Serial.println(F(" "));
     #endif  
  #endif  //alertMode
  
  return(flag);
}

//***********************************
void processGarage(){
  // 1. Disable garage door if car is present AND hatch is up,
  // 2. Disable hatch if car is present AND garage door is up.
  // 3. Close garage door if the car is absent AND the garage door is open AND there has been no motion for an hour.
  #ifdef GARAGE
    Serial.print(F("*processGarage*")); Serial.print(F("priorMillis, (millis() -  priorMillis): "));Serial.print(priorMillis);Serial.print(F(", "));Serial.println(millis()-priorMillis);

    if(platform[2].sonic <= sonicThreshhold){  Serial.print(F("Car is PRESENT.  "));
      if(platform[1].door>=OPEN){     Serial.println(F("Garage door is OPEN; DISABLE hatch."));
        //disable Hatch, solid red (to be developed)

        //after motion delay, send telegram
        
      }else if (platform[3].door >=OPEN){ Serial.println(F("Hatch is OPEN; DISABLE garage door"));  //disable gar door flash red
        //disable gar door  //done by HA for now
      }else{                          Serial.println(F("Hatch and garage door are CLOSED. CHILL."));  
        priorMillis=0;  
      }

    }else{                            Serial.print(F("Car is ABSENT.  "));  
      if(platform[1].door >=1){    Serial.print(F("Garage is OPEN"));
          if(platform[1].pir>=MOTION){Serial.println(F("There is MOTION in the garage."));
            priorMillis=millis();     //Restart the timer.
          }else{                      Serial.println(F("There is NO MOTION in the garage."));
            if((priorMillis==0)||(priorMillis>millis())){       //if timer has not started, or timer has rolled around, restart the timer.
              priorMillis=millis();   //Serial.print(F("priorMillis set to "));Serial.println(priorMillis);
            }
            if(millis()-priorMillis>=60000) { // <--test 60000= 60 sec){  //Close door after an hour of no activity.
             // digitalWrite(pinGarage,1); Serial.println(F("Timeout: Close garage door."));
              delay (1500);
             // digitalWrite(pinGarage,0);
              priorMillis=0;          //reset timer.
            }   
          }
      }else{                          Serial.println(F("Garage is CLOSED. CHILL."));
        priorMillis=0;
      }
    }
  #endif  
}
//*************************************
void  processSensorPlatforms(){
  //Read local and remote sensors if available and display on OLED display and serial monitor
  for (uint8_t i=0; i< numberOfPlatforms; i++){  // for each sensor:
    alert[i]=0;                   //clear alert
    #ifndef HTTPGET
      platformStatus[i]=0;
    #endif  
//Serial.print("processSensorPlatforms platformStatus(");Serial.print(i); Serial.print(") = ");Serial.println(platformStatus[i]);
    if (platformStatus[i]==0){    // if sensor has not been sampled yet:    
      #ifdef HTTPGET              // if we are using http GET via wifi connection with sensor platforms:
                                  // NOTE if using espNOW the data will be pushed to HUB; no need for further setup
        killWifi();               // prepare to connect via wifi with server
        if(setupWifiGET(i)==0){   // connect to local ESP32(i) wifi server if not local
      #endif
          platformStatus[i]= readSensorData(i); // need if no sleep time      
          if(platformStatus[i]==1){ //determine if sensor data has been updated
            sensorStatus[i]=1;    // indicate new data - display "%" after humidity in displayStatus()
            //#ifdef WIFI
              printSensorData();
              publishMQTT(i);                
            //#endif  
            alert[i]= processAlerts(i);    // compare sensors to threshholds and set alert status        
          }  
          #ifdef HTTPGET  
            killWifi();
        }
          #endif
    }  
      displayStatus(i);                   // update OLED display if there is one
      platform[i].id=numberOfPlatforms+1; // indicate data has been processed; new data will update id
      #ifdef alertMode
        platformStatus[i]=0;              // indicates readiness to read new data 
      #endif      
  }   
}    
//*************************
void readAh2o(){
  //Read analog value and convert to % as whole number 0-99
  #ifdef printMode
    //Serial.println(F("*readAh2o*"));
  #endif  
  if(h2os[sensorID]==1){
    //sensorData.aH2o = 2.44*(4095-analogRead(pinAh2o))/100;   //whole number 1-99 %
    aH2oMeasured =  touchRead(pinAh2o);
    if(aH2oMeasured > 5){
      aH2oMeasured = 5;  
    }
    sensorData.aH2o = 5 - aH2oMeasured;
  }  
}

//***********************************
void readBatteryVoltage (uint8_t i){  
  if (bat[i]==1){
    #ifdef printMode
      Serial.println("*ReadBatteryVoltage*");
    #endif
    uint16_t vBATT; 
    if (! fona.getBattVoltage(&vBATT)) {
      #ifdef printMode
        Serial.println(F("Failed to read Batt"));
        Serial.print(F("VBat = ")); Serial.print(vBATT); Serial.println(F(" mV"));
      #endif
    } else {
      #ifdef printMode
         Serial.print(F("VBat = ")); Serial.print(vBATT); Serial.println(F(" mV"));
      #endif
    }
      vbat[i]=vBATT/100;
  }
}

//***********************************
void readDh2o(){
  #ifdef printMode
    //Serial.println(F("*readDh2o*"));
  #endif  
  if(h2os[sensorID]==1){
    //sensor reads 1 when dry, 0 when wet
    //we want 0=dry (normal status)        
    //platform[sensorID].dH2o = !digitalRead(pinDh2o);
  }  
}

//***********************************
void readDoor(){  
  #ifdef printMode
    //Serial.print(F("*readDoor*"));Serial.print(F(" pinDoor: "));Serial.println(digitalRead(pinDoor));  
  #endif
  if (doors[sensorID]==1){  //interrupt updates doors count

     //door = 1 when closed; we want 0 (normal state) when closed
     if (platform[sensorID].door > 0){
       if (priorDoor == !digitalRead(pinDoor)){
       }else{
         platform[sensorID].door=!digitalRead(pinDoor);
         platform[sensorID].doorCount ++;
         priorDoor=platform[sensorID].door;
         
       }
     }else{
       platform[sensorID].door = !digitalRead(pinDoor);  //report door state 0 if closed, 1 if open
       platform[sensorID].doorCount = platform[sensorID].door;
       priorDoor=platform[sensorID].door;                 //save dooor status for later comparison
     }
  }
}   
//**********************************        
bool readNetStatus() {
  #ifdef testMode
    Serial.println(F("readNetStatus*"));
  #endif     

  int i = fona.getNetworkStatus();
  #ifdef printMode
    const char* networkStatus[]={"Not registered","Registered (home)","Not Registered (searching)","Denied","Unknown","Registered roaming"};    
    Serial.print(F("fona.getNetworkStatus")); Serial.print(i); Serial.print(F(": "));Serial.println(F(networkStatus[i]));
    Serial.println(F("fona.getNetworkInfo()"));
    fona.getNetworkInfo();  
  #endif    
  
  if (!(i == 1 || i == 5)) return false;
  else return true;
}

//***********************************
void readPhotoCell(){
  //Read analog value and convert to % as whole number 0-99
  #ifdef printMode
    //Serial.println(F("*readPhotoCell*"));
  #endif  
  if (luxs[sensorID]==1){
     platform[sensorID].lux = 2.44*((4095-analogRead(pinPhotoCell))/100);  //whole number 1-99 %
  }
}

//***********************************
void readPir(){
  #ifdef printMode
    //Serial.print(F("*readPir* "));
  #endif
  if (pirs[sensorID]==1){
    //pir = 0 when no motion (normal state)
    platform[sensorID].pir = digitalRead(pinPir); //+ platform[sensorID].pir ;
    platform[sensorID].pirCount = platform[sensorID].pirCount + platform[sensorID].pir;    
  }  
  #ifdef printMode
    //Serial.println(sensorData.pir);
  #endif
}

//*********************************
uint8_t readSensorData(uint8_t i){             // temp & voltage
  #ifdef printMode
//  if (i=0){
 //   Serial.print(F("*readSensorData*"));Serial.print(i);Serial.println(F("**************************************"));
//  }  
  #endif      

  uint8_t returnFlag = 1;  //return success causes ':' display on OLED
  if (sensors[i]=="local"){
  
    platform[i].id=i;
  
   if(sensorTimeIsUp(sensorSeconds*1000)==1){   //slow down loop for sensor R&R   
//Serial.print(F("*readSensorData*"));Serial.print(i);Serial.println(F("*************************************"));  
      //readSignalStrength(i); causes crash
      //readBatteryVoltage(i); causes crash
      readSonic();
      #ifdef MCP9808_              
        tempsensor.wake();                // Wake up the MCP9808 if it was sleeping
        float c = tempsensor.readTempC(); //read temperature in Celsius units
        float x = c * 9.0 / 5.0 + 32;     //calculate Fahrenheit
        if (isnan(x)){
          returnFlag=0;         //causes ? on OLED
          x=platform[i].temperature;
        }
        platform[i].temperature = x;
        delay(500);
      #endif
  
      #ifdef AHT10_
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
        //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
        //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
        float x=temp.temperature;
//Serial.println(x);   
        x=1.8*x+32.0;
//Serial.println(x);        
        if (isnan(x)){
          returnFlag=0;         //causes ? on OLED
          x=platform[i].temperature;
        }
        platform[i].temperature = x;
  
        x=humidity.relative_humidity;
        if (isnan(x)){
          x=platform[i].humidity;
        }
        platform[i].humidity=x;      
      #endif 
      
      #ifdef BME_
        float x = 1.8*bme.readTemperature()+32.0;
        if (isnan(x)){
          returnFlag=0;         //causes ? on OLED
          x=platform[i].temperature;
        }
        platform[i].temperature = x;
    
        x = (bme.readHumidity());  
        if (isnan(x)){
          x=platform[i].humidity;
        }
        //humidity[i] = x;
        platform[i].humidity=x;
       
         x = bme.readPressure()/100;
         if (isnan(x)){
           x=platform[i].pressure;
         }
         platform[i].pressure = x;
       #endif
       
       #ifdef DHT_
         float x = dht.readTemperature(true);  //*F
         if (isnan(x)){
           returnFlag=0;      //causes ? on OLED
           x=platform[i].temperature;
         }
         platform[i].temperature = x;
         
         x = (dht.readHumidity());
         if (isnan(x)){
           x=platform[i].humidity;
         }
         platform[i].humidity = x;
       #endif  
  
        #ifdef SHT20_
          float x = 1.8*sht20.readTemperature()+32.0;
          if (isnan(x)){
            returnFlag=0;         //causes ? on OLED
            x=platform[i].temperature;
          }
          platform[i].temperature = x;
      
          x = (sht20.readHumidity());  
          if (isnan(x)){
            x=platform[i].humidity;
          }
          platform[i].humidity=x;
        #endif
    }else{
      returnFlag = 0;
    }
      
  }else if (sensors[i]=="wifi"){ // read remote sensors
    #ifdef HTTPGET
      // Check WiFi connection status
      #ifdef printMode
          Serial.print(F("WL_CONNECTED = "));Serial.println(WL_CONNECTED);
          Serial.print(F("WiFi.status() = "));Serial.println(WiFi.status());
      #endif
  
      if(WiFi.status()!=WL_CONNECTED ){ 
        setupWifii(i);
      }
      if(WiFi.status()== WL_CONNECTED ){ 
        String payload = "--"; 
        if(temps[i]==1){
          payload = httpGETRequest(serverNameTemperature);

          if (payload!="--"){    //retain prior reading if no data so we dont trigger ifttt
            platform[i].temperature = payload.toFloat();
 
          }
        }
        if (hums[i]==1){ 
          payload = httpGETRequest(serverNameHumidity);
          if (payload!="--"){   //retain prior reading if no data so we dont trigger ifttt
            platform[i].humidity = payload.toFloat();
          }
        } 
        if(pres[i]==1){ 
          //payload = "";
          payload = httpGETRequest(serverNamePressure);
          if (payload!="--"){   //retain prior reading if no data so we dont trigger ifttt
            platform[i].pressure = payload.toFloat();
          }
        }
        if(luxs[i]==1){     
          payload = httpGETRequest(serverNameLux);
          if (payload!="--"){   //retain prior reading if no data so we dont trigger ifttt
            platform[i].lux = payload.toFloat();
          }        
        }
        if(h2os[i]==1){
          payload = httpGETRequest(serverNameAh2o);
          platform[i].aH2o = payload.toFloat();
          payload = httpGETRequest(serverNameDh2o);
          platform[i].dH2o = payload.toFloat();
        }
        if(doors[i]==1){
          payload = httpGETRequest(serverNameDoor);
          platform[i].door = payload.toFloat();
        }  

         platform[i].id=i;  //indicate successful data read
        
       }else{ 
         returnFlag=0;      //causes fail ? displayed on oled
       }
    #endif
  
    #ifdef ESPNOW
      #ifdef testMode
        //Serial.print("platform[sensorData.id].id :");Serial.println(boardsStruct[sensorData.id-1].id);
      #endif
      if (platform[i].id == i){  //indicate espnow data was received
        returnFlag = 1;
      }else{
        returnFlag = 0;  
      }
    #endif
  }
  #ifdef testMode
    Serial.print("returnFlag = ");Serial.println (returnFlag);  
  #endif
  return(returnFlag);
}

//*******************************
void readSignalStrength(int i){
  #ifdef testMode
    Serial.println(F("*readSignalStrength*"));
  #endif      
  dbm[i] = 0;
  #ifdef cellularMode
    uint8_t n = fona.getRSSI();
    if (n == 0) dbm[i] = -115;
    if (n == 1) dbm[i] = -111;
    if (n == 31) dbm[i] = -52;
    if ((n >= 2) && (n <= 30)) {
      dbm[i] = map(n, 2, 30, -110, -54);
    }
    #ifdef printMode
      Serial.print(F("fona.getRSSI()  "));
      Serial.print(dbm[i]); Serial.println(F(" dBm"));
    #endif
  #endif 
}
//*************************************
void readSonic(){
  if(sonics[sensorID]==1){
    #ifdef testMode
      Serial.println(F("*reasdSonic*"));Serial.print(F("entry reportFlag: "));Serial.println(reportFlag);
    #endif     

    //Send trigger pulse
    digitalWrite(pinSonicTrigger, HIGH);
    unsigned long time_now = micros();
    while (micros() < time_now + 10);
    digitalWrite(pinSonicTrigger, LOW);
  
    //measure how long for the pulse to return and calculate the distance in cm
    float duration = pulseIn(pinSonicEcho, HIGH);
    float sensorDistance = (duration / 2) * 0.0344;
    uint16_t distance = round(sensorDistance+1); //rounding up
    Serial.printf("\n\nDistance = %d cm\n", distance);
    
    if (abs(distance-sensorData.sonic)>=15 ) {
      sensorData.sonic=distance;       
      reportFlag = true;
    }    
    
    #ifdef printMode
      Serial.print(F("exit reportFlag: "));Serial.println(reportFlag);
    #endif
  }
}

//************************************
void readTelegram(){
  #ifdef TELEGRAM
    if (millis() > lastTimeBotRan + botRequestDelay)  {
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      while(numNewMessages) {
        Serial.println("got response");
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      lastTimeBotRan = millis();
   }
  #endif
}
//*************************************
uint8_t readWakeupID(){
  //read wakeup reason and return code
  #ifdef printMode 
    Serial.println(F("*readWakupID*"));
  #endif   
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
       sensorData.door ++ ;//= !digitalRead(pinDoor);
       priorDoor=!digitalRead(pinDoor);
       #ifdef printMode 
         Serial.print(F("priorDoor: "));Serial.println(priorDoor);
       #endif
       //printSensorData();
       return 0;
    case ESP_SLEEP_WAKEUP_EXT1 :      
       //sensorData.pir ++; //= sensorData.pir + digitalRead(pinPir);
       sensorData.pir = digitalRead(pinPir);
       sensorData.pirCount ++;       
       //printSensorData();
       return 1;
    case ESP_SLEEP_WAKEUP_TIMER : return 2;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return 3;
    case ESP_SLEEP_WAKEUP_ULP : return 4;
    default : return 5;
  }    
}

//***********************************
  void restartIfTime(){
  #ifdef printMode 
    Serial.print(F("*restartIfTime* bootCount = "));Serial.println(bootCount);
  #endif     
  if (bootCount>=bootResetCount){
    if (wakeupID==2){
      //if(sensorData.pir<=3){
        #ifdef printMode
          Serial.println("");delay (5000);
          Serial.println(F("Rebooting.............*************.............")); 
        #endif
        ESP.restart();  //Reboot the esp32 to prevent memory issues
      //} 
    }
  }      
  bootCount++;
}  
//*************************************
void runSelfTest(){                 //pwr SIM on/off, test Cellular network if testCellular enabled 
  #ifdef cellularMode              // If cellular comms is enabled
    if (bootCount==1){
      powerUpSimModule();          // Power up the SIM7000 module by goosing the PWR pin 
      delay (5000);
      simModuleOffIfSimSleepMode();// Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
      #ifdef testCellular          //if we want to test cellular data on first startup
        activateCellular(); 
        readSignalStrength(0);
        postSensorData();       // upload test data if enabled
        displayStatus(0);          // update OLED display to show initial data
        simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
      #endif      
    }        
  #endif
}

//*************************************
int sendATCmd(char ATCode[], char parm1[], char parm2[]){  //220622
  /*send AT  to modem, check for response and return 0 if expected response true else return -1
  *ATCode = <20 character AT comd example: AT+HTTPPARA
  *parm1, parm 2 = parameters as defined in SimComm AT Command manual
   */
  #ifdef printMode
    Serial.println(F("*sendATCmd*___________________"));
  #endif
  
  //1. construct command from ATCode,parms
  char ATCode_[255]; 

  strcpy(ATCode_,ATCode);strcat(ATCode_,parm1);  //append parm1 even if blank
  //append comma and parm2 if not blank
  switch (strlen(parm2)){
    case 0:
      break;
    default:
      //int len = snprintf(ATCode_,sizeof(ATCode_),"%s","%s",",%s",ATCode,parm1,parm2);  //doesn't work
      strcat(ATCode_,",");strcat(ATCode_,parm2);  //do this instead for now
  }    

  #ifdef printMode
    //Serial.print("len returned by snprintf = ");Serial.print(len);
    Serial.print("; len ATCode + parameters = ");Serial.println(strlen(ATCode_));
  #endif    

  //2. submit ATCmd to modem
  int delays = 0; 
  while (1){
    #ifdef printMode 
      Serial.print(F("-->"));Serial.println(ATCode_);
    #endif
    fona.println(ATCode_);               
    delay (500); // 500 ms delay is a safe tradeoff

    //3. check if modem response is ready
checkFona_:                          //I know, I know, bad form.. fix it later
    if (fona.available()){
      
      //3a. process response
      char resp[50];
      fona.readString().toCharArray(resp,sizeof(resp));
      int len; 
      #ifdef printMode
        Serial.print(F("<--"));
        for(len=2; len<=sizeof(resp);len++){
          Serial.print(F("."));
          Serial.print(resp[len]);
          if (resp[len]=='\0'){
            Serial.print(F("  Len: "));
            Serial.println(len);
            break;
          }  
        }
      #endif
      
      //3b. Return -1 if 'OK' is not in response, else return 0 if HTTPACTION and ,200, found
      char * pch;
      pch = strstr (resp,"OK");
      if (pch) {
        #ifdef printMode
          Serial.print(F("OK in position: "));Serial.println((int)(pch-resp));
        #endif 
        pch = strstr(resp,"HTTPACTION");
        if (pch){
          #ifdef printMode
            Serial.print(F("HTTPACTION in position: "));Serial.println((int)(pch-resp));
          #endif  
          pch = strstr(resp,",200,");
          if(pch){
            #ifdef printMode
              Serial.print(F(",200, in position: "));Serial.println((int)(pch-resp));
            #endif  
            return 0;
          }else{
            #ifdef printMode
              Serial.println(F(",200, NOT FOUND"));
            #endif  
            return -1;
          }
        }else{
          return 0;
        }
        return 0;
      }else{
        #ifdef printMode
          Serial.println (F("OK NOT FOUND")); 
        #endif  
        return -1; 
      }
    }else{   //if fona not available
      //3c. delay and try again if modem busy
      #ifdef printMode              
        Serial.print(F("."));
      #endif  
      delay(500); // Retry delay
      delays++;
      if (delays==10){  //give it 5 seconds
        return -1;
      }
      goto checkFona_;
    }
  }
}

//***********************************
void sendEspNow_1to1(){
  //format data and send to single ESPNOW peer if #define espnow_1to1 is enabled
  #ifdef ESPNOW_1to1
    #ifdef printMode
      Serial.print(F("*sendEspNow1to1* "));
    #endif    // 
    sensorData.id = sensorID;
    //printSensorData();

    //format ESPNow data to send
    dataOut.sonic = platform[0].sonic;
    dataOut.temperature = platform[0].temperature;
    dataOut.humidity = platform[0].humidity;
    dataOut.pressure = platform[0].pressure;
    dataOut.door = platform[1].door;
    dataOut.doorCount = platform[1].doorCount;
    dataOut.pir = platform[1].pir+platform[0].pir;
    
    esp_err_t espResult = esp_now_send(SECRET_broadcastAddress, (uint8_t *) &dataOut, sizeof(dataOut)); // Send message via ESP-NOW
    #ifdef printMode
      if (espResult == ESP_OK) { 
        Serial.println(F(" Sent with success"));
      }else{
        Serial.println(F(" Error sending the data"));  
      }
    #endif
    delay(espNowDelay);
  #endif
}

//*************************************
static int sensorTimeIsUp(long msec){    //Read data at timed intervals       
  static unsigned long previousMillis = 0;   
  unsigned long currentMillis = millis();
//Serial.println("currentMillis  previousMillis");
//Serial.print(currentMillis);  Serial.print("    ");Serial.println(previousMillis);
  if (currentMillis - previousMillis >= msec) {
    previousMillis = currentMillis; 
    return(1);
  }else{
    if (previousMillis == 0){  //indicate time has passed the first time thru
      previousMillis = currentMillis;  
      return(1);
    }else{  
      return(0);
    }
  }
} 

//*************************************
void serialPrintVersion(){         
   //Show startup info to serial monitor if printMode enabled
   #ifdef printMode
      Serial.begin(115200);         // Initialize Serial Monitor 
      turnOnBoardLED();           //turn on board LED during setup
      Serial.print(F("***** "));Serial.print(F(APP)); Serial.println(F(VERSION));
      Serial.print(F("bootCount= "));Serial.println(bootCount);
      printWakeupID(wakeupID);
      printSensorData();
   #endif
}
//***********************************
static int serialTimeIsUp(long msec){    //Read data at timed intervals  
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
void setupEspNow(){
  #ifdef ESPNOW
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
  
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(espNowOnDataRecv);

    #ifdef ESPNOW_1to1

      #ifdef gateway
        //Use active wifi channel in case destination connected to it.
        int32_t channel = getWiFiChannel(SECRET_WIFI_SSID);
        //WiFi.printDiag(Serial); // Verify channel number before
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
        //WiFi.printDiag(Serial); // verify channel change
      #endif

      // register for Send CB to get the status of Trasnmitted packet
      esp_now_register_send_cb(ESPNOW_1to1_OnDataSent);
      
      // Register peer
      memcpy(peerInfo.peer_addr, SECRET_broadcastAddress, 6);
      peerInfo.channel = 0;  
      peerInfo.encrypt = false;
      
      // Add peer        
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        #ifdef printMode
          Serial.println(F("Failed to add peer"));
        #endif
        return;
      }
      #endif
  #endif
}

//********************************* 
void setupMQTT_Subscribe(){              //setup if adaMQTT_Subscribe enabled
   #ifdef adaMQTT_Subscribe   
     activateCellular(); 
     MQTT_connect();
     #ifdef printMode
       Serial.println(F("Subscribe begin............."));
     #endif
     mqtt.subscribe(&feed_command); // Only if you're using MQTT
     #ifdef printMode
       Serial.println(F("Subscribe complete............."));
     #endif
   #endif 
}   

//*********************************  
void setupOledDisplay(){
  #ifdef testMode
        Serial.println(F("*setupOledDisplay*"));
  #endif
  #ifdef OLED_
    Wire.begin();
    oled.begin(&Adafruit128x64,I2C_ADDRESS); 
  #endif  
}

//*********************************
void setupPinModes(){                
  //Set Pin Modes INPUT / OUTPUT
  #ifdef testMode
     Serial.println(F("*setupPinModes*"));
  #endif
  pinMode(pinBoardLED,OUTPUT);
  pinMode(pinFONA_PWRKEY, OUTPUT);
  #ifdef BOTLETICS  
    pinMode(pinFONA_RST, OUTPUT); 
    digitalWrite(pinFONA_RST, HIGH); // Default state
    digitalWrite(pinFONA_PWRKEY, OUTPUT); //210220 ??
  #endif
  if (doors[sensorID]==1){
    pinMode (pinDoor, INPUT);
  }  
  if (pirs[sensorID]==1){
    pinMode (pinPir, INPUT);
  }
  if (luxs[sensorID]==1){
    pinMode (pinPhotoCell, INPUT);
  }  
  //if (h2os[sensorID]==1){  
  //    pinMode (pinAh2o, INPUT);
  //    pinMode (pinDh2o, INPUT_PULLUP);
  //}

  if (sonics[sensorID]==1){
    pinMode (pinSonicTrigger, OUTPUT);
    pinMode (pinSonicEcho, INPUT);
  }
  pinMode(pinGarage, OUTPUT);  

}

//*********************************
void setupSensors(){             // Wake up the MCP9808 if it was sleeping
  #ifdef printMode
     Serial.println(F("*WakeUpSensors*"));
  #endif
  #ifdef AHT10_
    aht.begin();
    delay(100);
  #endif 
  #ifdef BME_ 
    bool bme280=bme.begin(0x76);
    delay(100);
    if (!bme280){
       #ifdef printMode
          Serial.println(F("Failed to initiate bme280"));
       #endif
    }
  #endif
  #ifdef DHT_
    dht.begin();
    delay(100);
  #endif
  #ifdef MCP9808_      
    if (!tempsensor.begin()) {
      #ifdef printMode
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
int setupSimModule() {  
  #ifdef printMode
    Serial.println(F("*setupSimModule*"));
    Serial.println(F("fonaSS.begin(115200, SERIAL_8N1, pinFONA_TX, pinFONA_RX)"));
    Serial.println(F("fonaSS.println('AT+IPR=9600')"));    
  #endif
  
  #ifdef BOTLETICS
    fonaSS.begin(115200, SERIAL_8N1, pinFONA_TX, pinFONA_RX); // baud rate, protocol, ESP32 RX pin, ESP32 TX pin
    //if (testMode==1) {Serial.println(F("Configuring to 9600 baud"));}
    fonaSS.println("AT+IPR=9600"); // Set baud rate
    delay(100); // let the command complete
  #endif
  #ifdef printMode
    Serial.println(F("fonaSS.begin(9600, SERIAL_8N1, pinFONA_TX, pinFONA_RX)"));
    Serial.println(F("fona.begin(fonaSS)"));
  #endif  
  
  fonaSS.begin(UART_BAUD, SERIAL_8N1, pinFONA_TX, pinFONA_RX); // Switch to 9600
  if (! fona.begin(fonaSS)) {
    #ifdef printMode
      Serial.println(F("Couldn't find FONA"));
    #endif
    return(-1);
  }else{
    #ifdef printMode
        Serial.println(F("found FONA"));
    #endif
  }

  type = fona.type(); // read sim type
  #ifdef testMode
    Serial.print(F("fona.type: "));Serial.println(type);
  #endif

  #ifdef testMode
    Serial.print(F("fona.getIMEI(imei)"));
  #endif
  uint8_t imei1 = fona.getIMEI(imei);
  if (imei1 > 0) {
    #ifdef printMode
      Serial.print(F(" Module IMEI: ")); Serial.println(imei);
    #endif
  }
  return(0);
}
//*********************************
void setupTelegram(){
  #ifdef TELEGRAM
    // Connect to Wi-Fi
    //WiFi.mode(WIFI_STA);
   // WiFi.begin(ssid, password);
    #ifdef ESP32
      client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
    #endif
    delay(1000);
    //while (WiFi.status() != WL_CONNECTED) {
    //  Serial.println("Connecting to WiFi..");
    //}
    // Print ESP32 Local IP Address
    //Serial.println(WiFi.localIP());
  #endif
}
//*********************************
void setupWakeupConditions(){
  //set wakeup conditions based on door & PIR sensor
  #ifdef printMode
    Serial.print(F("*setupWakeupConditions*"));
    Serial.print(F(" door = "));Serial.print(sensorData.door);
    Serial.print(F("; pir = "));Serial.println(sensorData.pir);
  #endif 

  //Set up door causes wakeup when opened or closeed
  if(doors[sensorID]==1){
    if (digitalRead(pinDoor)==1){
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0); //0 = High OPEN 1 = Low CLOSED
    }else{
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,1);  
    } 
  }  

  //set up pir causes wakeup when movement detected
  if(pirs[sensorID]==1){
    #define wakeupBitmask 0x4000000000  //pir gpio 39
    esp_sleep_enable_ext1_wakeup(wakeupBitmask,ESP_EXT1_WAKEUP_ANY_HIGH);
  }  
}      

//***********************************
int setupWifiGET(uint8_t i){
  #ifdef HTTPGET  
    if (sensors[i]=="local"){return 0;}
    #ifdef printMode 
      Serial.print(F("*setupWiFiGET("));Serial.print(i);Serial.println(F(") *")); 
      Serial.print("wifiSSID[i]: ");Serial.print(wifiSSID[i]);
      Serial.print(F("  wifiPassword[i]: "));Serial.println(wifiPassword[i]);
    #endif
    turnOnBoardLED();
    WiFi.begin(wifiSSID[i], wifiPassword[i]);
    int attempt = 0;
    while(attempt < 5 && WiFi.status() != WL_CONNECTED) { 
      delay(500);
      #ifdef printMode
        Serial.print(F("."));
      #endif
      attempt++;
    }
    if (attempt >=5){
      #ifdef printMode
        Serial.print(F(" failed to connect to ssid "));Serial.println(wifiSSID[i]);
      #endif
      turnOffBoardLED();
      return (1);
    }else{
      #ifdef printMode
        Serial.print(F(" Connected to WiFi network "));Serial.print(i); Serial.print(F(" with IP Address: "));
        Serial.println(WiFi.localIP());
      #endif  
      return (0);
    }
  #endif
}

//*********************************
void simModuleOffIfSimSleepMode(){    // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
  #ifdef printMode
    Serial.println(F("*simModuleOffIfSimSleepMode*"));  
  #endif    
  #ifdef simSleepMode 
    #ifdef testMode
      Serial.print(F(" Powering down SIM module"));
    #endif
    //fona.powerDown(); //crashes
    
    #ifdef BOTLETICS
    //
      digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
      delay(2000);                          
      digitalWrite(pinFONA_PWRKEY, HIGH);
    #endif  
    
    #ifdef LILLYGO
     //fona.powerDown();
    
     digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
     delay(1500);                          
     digitalWrite(pinFONA_PWRKEY, HIGH);
   #endif  
 #endif
 delay(5000);                         // give it time to take hold - long delay
 #ifdef testMode
    Serial.print(F(" "));
 #endif        
}

//**************************************
void turnOnBoardLED(){              // Turn LED on 
  #ifdef  LILLYGO 
    digitalWrite(pinBoardLED, LOW);
  #else 
    digitalWrite(pinBoardLED, HIGH);
  #endif
} 

//*********************************    
void turnOffBoardLED(){             // Turn LED off
  #ifdef  LILLYGO 
    digitalWrite(pinBoardLED, HIGH);
  #else 
    digitalWrite(pinBoardLED, LOW);
  #endif
}    

//************************************************************WiFi Stuff
void setupWifi(){
  #ifdef HA
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      #ifdef printMode
        Serial.print(F("."));
      #endif
    }
    #ifdef printMode
      Serial.print(F("WiFi connected..  IP Address: "));
      Serial.println(WiFi.localIP());
    #endif
  #endif  
}
void connectToWifi() {
  #ifdef HA
    #ifdef printMode
      Serial.println(F("Connecting to Wi-Fi..."));
    #endif
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  #endif
}

 #ifdef HA
void WiFiEvent(WiFiEvent_t event) {
   
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      #ifdef printMode
        Serial.print(F("WiFi connected..  IP Address: "));
        Serial.println(WiFi.localIP());
      #endif
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      #ifdef printMode
        Serial.println(F("WiFi lost connection"));
      #endif
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}
  #endif


//**************************************************************************MQTT Stuff  
void setupMQTT(){
    #ifdef HA
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
    #endif
}
void connectToMqtt() {
  #ifdef HA
      #ifdef printMode
        Serial.println(F("\nConnecting to MQTT..."));
      #endif
    mqttClient.connect();
  #endif
}

void onMqttConnect(bool sessionPresent) {
  #ifdef HA
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

void publishMQTT(uint8_t i){
  #ifdef HA
    #ifdef printMode
      Serial.print(F("*publishMQTT* (HA) "));Serial.println(i);
    #endif
    #ifdef CASA_1
      switch (i){
        case 0:      
          //mqttClient.publish(strcat(location[1],TEMPERATURE_TOPIC), 0, false, (String(platform[1].temperature)).c_str());
          mqttClient.publish(HUB_TEMPERATURE_TOPIC, 0, false, (String(platform[0].temperature)).c_str());
          mqttClient.publish(HUB_HUMIDITY_TOPIC, 0, false, (String(platform[0].humidity)).c_str());
          mqttClient.publish(HUB_PRESSURE_TOPIC, 0, false, (String(platform[0].pressure)).c_str()); 
          mqttClient.publish(HUB_SEND_FAILURES_TOPIC, 0, false, (String(platform[0].sendFailures)).c_str()); 
        break;
        case 1:
          mqttClient.publish(GARAGE_DOOR_TOPIC, 0, false, (String(platform[1].door)).c_str());
          mqttClient.publish(GARAGE_DOOR_COUNT_TOPIC, 0, false, (String(platform[1].doorCount)).c_str());
          //Serial.print(GARAGE_PIR_TOPIC);Serial.print("   -   "); Serial.println(String(platform[1].pir).c_str());
          mqttClient.publish(GARAGE_PIR_TOPIC, 0, false, (String(platform[1].pir)).c_str()); 
          mqttClient.publish(GARAGE_LUX_TOPIC, 0, false, (String(platform[1].lux)).c_str());  
          mqttClient.publish(GARAGE_AH2O_TOPIC, 0, false, (String(platform[1].aH2o)).c_str());  
          mqttClient.publish(GARAGE_DH2O_TOPIC, 0, false, (String(platform[1].dH2o)).c_str());          
          mqttClient.publish(GARAGE_SEND_FAILURES_TOPIC, 0, false, (String(platform[1].sendFailures)).c_str()); 
        break;
        case 2:
          mqttClient.publish(SONIC_TOPIC, 0, false, (String(platform[2].sonic)).c_str());   
          mqttClient.publish(SONIC_SEND_FAILURES_TOPIC, 0, false, (String(platform[2].sendFailures)).c_str()); 
        break;
        case 3:           
          mqttClient.publish(HATCH_TOPIC, 0, false, (String(platform[3].door)).c_str());
          mqttClient.publish(HATCH_SEND_FAILURES_TOPIC, 0, false, (String(platform[3].sendFailures)).c_str()); 
       break;
        case 4:
          //kitchen topis
          mqttClient.publish(KITCHEN_TEMPERATURE_TOPIC, 0, false, (String(platform[4].temperature)).c_str());
          mqttClient.publish( KITCHEN_HUMIDITY_TOPIC, 0, false, (String(platform[4].humidity)).c_str());
          mqttClient.publish(KITCHEN_PRESSURE_TOPIC, 0, false, (String(platform[4].pressure)).c_str());
          mqttClient.publish(KITCHEN_PIR_TOPIC, 0, false, (String(platform[4].pir)).c_str());
          mqttClient.publish(KITCHEN_SEND_FAILURES_TOPIC, 0, false, (String(platform[4].sendFailures)).c_str()); 
        break;
        case 5:
          //bathroom topics
          mqttClient.publish(BATHROOM_TEMPERATURE_TOPIC, 0, false, (String(platform[5].temperature)).c_str());
          mqttClient.publish(BATHROOM_HUMIDITY_TOPIC, 0, false, (String(platform[5].humidity)).c_str());
          mqttClient.publish(BATHROOM_PRESSURE_TOPIC, 0, false, (String(platform[5].pressure)).c_str());
          mqttClient.publish(BATHROOM_PIR_TOPIC, 0, false, (String(platform[5].pir)).c_str());
          mqttClient.publish(BATHROOM_SEND_FAILURES_TOPIC, 0, false, (String(platform[5].sendFailures)).c_str()); 
       break;
        case 6:
          //bathroom2 topics
          mqttClient.publish(BATHROOM2_TEMPERATURE_TOPIC, 0, false, (String(platform[6].temperature)).c_str());
          mqttClient.publish(BATHROOM2_HUMIDITY_TOPIC, 0, false, (String(platform[6].humidity)).c_str());
          mqttClient.publish(BATHROOM2_PRESSURE_TOPIC, 0, false, (String(platform[6].pressure)).c_str());
          mqttClient.publish(BATHROOM2_PIR_TOPIC, 0, false, (String(platform[6].pir)).c_str());
          mqttClient.publish(BATHROOM2_SEND_FAILURES_TOPIC, 0, false, (String(platform[6].sendFailures)).c_str()); 
        break;
      }
      //platform[1].doorCount=0;
      //platform[1].pir=0;
      //platform[4].pir=0;
      //platform[5].pir=0;
      //platform[6].pir=0;
      //platform[1].sendFailures=0;
      //mqttClient.publish(KITCHEN_PIR_TOPIC, 0, false, (String(platform[1].pir)).c_str());
      //mqttClient.publish(KITCHEN_PIR_TOPIC, 0, false, (String(platform[4].pir)).c_str());
      //mqttClient.publish(BATHROOM_PIR_TOPIC, 0, false, (String(platform[5].pir)).c_str());
      //mqttClient.publish(BATHROOM2_PIR_TOPIC, 0, false, (String(platform[6].pir)).c_str());
    #endif  //CASA_1
    
    #ifdef CASA_2
      switch(i){
        case 0:
          //HUB:
          Serial.print("(platform[0].temperature: ");Serial.println(platform[0].temperature);
          mqttClient.publish(HUB_TEMPERATURE_TOPIC, 0, false, (String(platform[0].temperature)).c_str());
          mqttClient.publish(HUB_HUMIDITY_TOPIC, 0, false, (String(platform[0].humidity)).c_str());
          mqttClient.publish(HUB_SEND_FAILURES_TOPIC, 0, false, (String(platform[0].sendFailures)).c_str()); 
          mqttClient.publish(HUB_PRESSURE_TOPIC, 0, false, (String(platform[0].pressure)).c_str());
        break;
        case 1:
          //Basement:
          //mqttClient.publish(strcat(location[1],TEMPERATURE_TOPIC), 0, false, (String(platform[1].temperature)).c_str());
          mqttClient.publish(BASEMENT_TEMPERATURE_TOPIC, 0, false, (String(platform[1].temperature)).c_str());
          mqttClient.publish(BASEMENT_HUMIDITY_TOPIC, 0, false, (String(platform[1].humidity)).c_str());
          //mqttClient.publish(PRESSURE_TOPIC, 0, false, (String(platform[1].pressure)).c_str()); 
          //mqttClient.publish(DOOR_TOPIC, 0, false, (String(platform[1].door)).c_str());
          //mqttClient.publish(DOOR_COUNT_TOPIC, 0, false, (String(platform[1].doorCount)).c_str());
          mqttClient.publish(BASEMENT_PIR_TOPIC, 0, false, (String(platform[1].pir)).c_str()); 
          mqttClient.publish(BASEMENT_PIR_COUNT_TOPIC, 0, false, (String(platform[1].pirCount)).c_str()); 
          mqttClient.publish(BASEMENT_LUX_TOPIC, 0, false, (String(platform[1].lux)).c_str());  
          mqttClient.publish(BASEMENT_AH2O_TOPIC, 0, false, (String(platform[1].aH2o)).c_str());  
          mqttClient.publish(BASEMENT_DH2O_TOPIC, 0, false, (String(platform[1].dH2o)).c_str());  
          mqttClient.publish(BASEMENT_SEND_FAILURES_TOPIC, 0, false, (String(platform[1].sendFailures)).c_str()); 
        break;
        case 2:

          //bedroom topics
          mqttClient.publish(BEDROOM_TEMPERATURE_TOPIC, 0, false, (String(platform[2].temperature)).c_str());
          mqttClient.publish(BEDROOM_HUMIDITY_TOPIC, 0, false, (String(platform[2].humidity)).c_str());
          mqttClient.publish(BEDROOM_DOOR_TOPIC, 0, false, (String(platform[2].door)).c_str());
          mqttClient.publish(BEDROOM_DOOR_COUNT_TOPIC, 0, false, (String(platform[2].doorCount)).c_str());
          mqttClient.publish(BEDROOM_SEND_FAILURES_TOPIC, 0, false, (String(platform[2].sendFailures)).c_str()); 
        break;
        case 3:
          
          //kitchen topics
          mqttClient.publish(KITCHEN_TEMPERATURE_TOPIC, 0, false, (String(platform[3].temperature)).c_str());
          mqttClient.publish(KITCHEN_HUMIDITY_TOPIC, 0, false, (String(platform[3].humidity)).c_str());
          mqttClient.publish(KITCHEN_DOOR_TOPIC, 0, false, (String(platform[3].door)).c_str());
          mqttClient.publish(KITCHEN_DOOR_COUNT_TOPIC, 0, false, (String(platform[3].doorCount)).c_str());
          mqttClient.publish(KITCHEN_PIR_TOPIC, 0, false, (String(platform[3].pir)).c_str());
          mqttClient.publish(KITCHEN_PIR_COUNT_TOPIC, 0, false, (String(platform[3].pirCount)).c_str());
          mqttClient.publish(KITCHEN_LUX_TOPIC, 0, false, (String(platform[3].lux)).c_str());
          mqttClient.publish(KITCHEN_SEND_FAILURES_TOPIC, 0, false, (String(platform[3].sendFailures)).c_str()); 
        break;

        case 4:

        //bathroom topics
        mqttClient.publish(BATHROOM_TEMPERATURE_TOPIC, 0, false, (String(platform[4].temperature)).c_str());
        mqttClient.publish(BATHROOM_HUMIDITY_TOPIC, 0, false, (String(platform[4].humidity)).c_str());
        //mqttClient.publish(BATHROOM_LUX_TOPIC, 0, false, (String(platform[4].lux)).c_str());
        mqttClient.publish(BATHROOM_SEND_FAILURES_TOPIC, 0, false, (String(platform[4].sendFailures)).c_str()); 
      break;
      }

      //platform[3].doorCount=0;
     // platform[1].pir=0;
      //platform[3].pir=0;
      //mqttClient.publish(BASEMENT_PIR_TOPIC, 0, false, (String(platform[1].pir)).c_str());
      //mqttClient.publish(KITCHEN_PIR_TOPIC, 0, false, (String(platform[3].pir)).c_str());
    #endif //CASA_2

    #ifdef CASA_2a
    switch(i){
      case (0):
        //HUB:
        mqttClient.publish(HUBa_TEMPERATURE_TOPIC, 0, false, (String(platform[0].temperature)).c_str());
        mqttClient.publish(HUBa_HUMIDITY_TOPIC, 0, false, (String(platform[0].humidity)).c_str());
        mqttClient.publish(HUBa_SEND_FAILURES_TOPIC, 0, false, (String(platform[0].sendFailures)).c_str()); 
        //mqttClient.publish(HUBa_PRESSURE_TOPIC, 0, false, (String(platform[0].pressure)).c_str());
      break;
      case 1:
        //Basement:
        //mqttClient.publish(strcat(location[1],TEMPERATURE_TOPIC), 0, false, (String(platform[1].temperature)).c_str());
        mqttClient.publish(BASEMENTa_TEMPERATURE_TOPIC, 0, false, (String(platform[1].temperature)).c_str());
        mqttClient.publish(BASEMENTa_HUMIDITY_TOPIC, 0, false, (String(platform[1].humidity)).c_str());
        //mqttClient.publish(PRESSURE_TOPIC, 0, false, (String(platform[1].pressure)).c_str()); 
        //mqttClient.publish(DOOR_TOPIC, 0, false, (String(platform[1].door)).c_str());
        //mqttClient.publish(DOOR_COUNT_TOPIC, 0, false, (String(platform[1].doorCount)).c_str());
        mqttClient.publish(BASEMENTa_PIR_TOPIC, 0, false, (String(platform[1].pir)).c_str()); 
        mqttClient.publish(BASEMENTa_PIR_COUNT_TOPIC, 0, false, (String(platform[1].pirCount)).c_str()); 
        mqttClient.publish(BASEMENTa_LUX_TOPIC, 0, false, (String(platform[1].lux)).c_str());  
        mqttClient.publish(BASEMENTa_AH2O_TOPIC, 0, false, (String(platform[1].aH2o)).c_str());  
        mqttClient.publish(BASEMENTa_DH2O_TOPIC, 0, false, (String(platform[1].dH2o)).c_str());  
        mqttClient.publish(BASEMENTa_SEND_FAILURES_TOPIC, 0, false, (String(platform[1].sendFailures)).c_str()); 
      break;
      case 2:
        //bathroom topics
        mqttClient.publish(BATHROOMa_TEMPERATURE_TOPIC, 0, false, (String(platform[2].temperature)).c_str());
        mqttClient.publish(BATHROOMa_HUMIDITY_TOPIC, 0, false, (String(platform[2].humidity)).c_str());
        mqttClient.publish(BATHROOMa_LUX_TOPIC, 0, false, (String(platform[2].lux)).c_str());
        mqttClient.publish(BATHROOMa_SEND_FAILURES_TOPIC, 0, false, (String(platform[2].sendFailures)).c_str()); 
      break;
    }       
    #endif  //casa_2a 
  #endif
}

  #ifdef HA
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef printMode
    Serial.println(F("Disconnected from MQTT."));
  #endif
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  #ifdef printMode
    Serial.println(F("Subscribe acknowledged."));
  #endif
}

void onMqttUnsubscribe(uint16_t packetId) {
  #ifdef printMode
    Serial.println(F("Unsubscribe acknowledged."));
  #endif  
}

void onMqttPublish(uint16_t packetId) {
  #ifdef printMode
    Serial.print("onMqttPublish packet ID = ");Serial.println(packetId);
  #endif  
}

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
 // displayWeatherReport(reptTempF, reptHumidity, reptPressure, reptLight);
}


#endif

#ifdef TELEGRAM
//Handle what happens when you receive new messages
void handleNewMessages(int numNewMessages) {
  #ifdef printMode
    Serial.print(F("handleNewMessages  --> "));
    Serial.println(String(numNewMessages));
  #endif  

  for (int i=0; i<numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != SECRET_CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    #ifdef printMode
      Serial.println(text);
    #endif
    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "ON - LED on \n";
      welcome += "OFF - LED off \n";
      //welcome += "STATE - sensor status \n";
      welcome += "G - garage door\n";
      bot.sendMessage(chat_id, welcome, "");
    }

   if (text == "ON") {
      bot.sendMessage(chat_id, "LED state set to ON", "");
      turnOnBoardLED();
    }
    
    if (text == "OFF") {
      bot.sendMessage(chat_id, "LED state set to OFF", "");
      turnOffBoardLED();
    }

    if (text == "G") {
      digitalWrite(pinGarage,1);
      delay (2500);
      digitalWrite(pinGarage,0);
    }
/*    if (text == "/state") {
      if (digitalRead(ledPin)){
        bot.sendMessage(chat_id, "LED is ON", "");
      }
      else{
        bot.sendMessage(chat_id, "LED is OFF", "");
      }
    } 
      
    if (text == "/readings") {
      String readings = getReadings();
      bot.sendMessage(chat_id, readings, "");
    }  */
  }
}
#endif

///////////////////////
// *** Setup() ***
//////////////////////
void setup() {
  delay(200);                      // ESP32 Bug workaround -- don't reference rtc ram too soon!!
  setupPinModes();                 // Set Pin Modes INPUT / OUTPUT (necessary to turn board LED on, etc.) 
  initializeArrays();              // Initialize sensor threshholds etc.
  wakeupID = readWakeupID();       // process wakeup sources & return wakeup identifier
  restartIfTime();                 // restart if bootcount>bootResetCount on timer wakeup else increment bootCount 
  setupOledDisplay(); 
  displayVersion();                // display software version and blink onboard led 5 sec if bootCount=1
  turnOnBoardLED();
  displayPlatforms();              // Display each platform latest sensor readings on OLED display 
  serialPrintVersion();            // If printMode enabled, begin serial monitor, Show startup info, turnOnBoardLED
  //setupOTA();
  //attachInterrupt(digitalPinToInterrupt(pinPir), pirInterrupt, RISING);  //in case you want to try it!
  //attachInterrupt(digitalPinToInterrupt(pinDoor), doorChangeInterrupt, CHANGE); //in case you want to try it!
  setupSensors();                  // Wake up the HUB temp sensors 
  runSelfTest();                   // SIM on/off, test network on first boot if testCellular enabled 
  setupMQTT();                     //if HA is set
  setupEspNow();                   // initialize communication with with external sensor platforms if enabled; 
                                   // incoming data pushed by platforms via espNow cause in interrupt which is
                                   //  processed at the espNowOnDataRecv() function.
//  setupMQTT_Subscribe();           // setup if adaMQTT_Subscribe enabled
  setupTelegram();
  turnOffBoardLED();
}

/////////////////
// ***  Loop()  ***
////////////////// 
void loop() {

// 1. Poll local sensors if they exist  
  readDoor();
  readAh2o();
  readDh2o();
  readPhotoCell(); 
  readPir();
  readSonic();
  processGarage();
  readTelegram();

// 2. Read add'l local and remote sensors if available and display on OLED and serial monitor
  processSensorPlatforms(); 

// 3. Upload existing alerts if alertMode enabled   
  postAlerts();                             
  
// 4. Heartbeat: upload latest periodic data if its time to HAS via MQTT to and adaFruit/IFTTT/dweet via cellular
  postSensorHeartbeatData();

// 5. sendEspNow_1to1();      //GROWTH) format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                              //successful data transfer will cause the system to sleep if sleepSeconds > 0   
                                                 
// 6. publishMQTT();          // NOTE: publishMQTT() is called from within the ESPNOW Callback function espNowOnDataRecv()
                              // and so it updates HA only when new data is received.  
                              // Enable it here if data is received via WiFi or non ESPNOW protocol.    

// 7. Go to sleep if snooze mode time has elapsed
  napTime();

// 8. Loop back to step 1   
  #ifdef printMode
    delay(1000);    //delay before looping so that staus display is readable if printMode is enabled
  #endif  
}
