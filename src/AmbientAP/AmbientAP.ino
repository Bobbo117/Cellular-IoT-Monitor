
const char* APP = "AmbientAP ";
const char* VERSION = "2023 v01.31";

/////////////////////////////////////////////////////////////////////////////////////
//
// AmbientAP is a flexible, multi-featured sensor platform.  It can:
//  1. expose sensor values via http GET command requests from a client.
//  2. expose sensor values via lighter footprint ESPNOW protocol which pushes data to a peer/client periodically without receiving a request.
//  3. expose sensor values via MQTT to Home Assistant or other MQTT broker (under development).
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
//  5. report to 1 ESPNOW peer or up to 4 peers.
//  6. sleep and wake up via a timer, an interrupt (such as a window being opened) or motion, or it can stay awake.
//  7. sleep immediately after sending a successfully received ESPNOW message when configured as a 1 to 1 peer.
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
         
         4. Onboard LED states for debug/verification purposes when printMode is on:
            -->ON at setup Start, off at setup end.
            -->ON at GET temperature
            -->OFF at GET humidity
            -->OFF at  sleep     
*/
//////////////////////////////////////////////////////////////  
//*******         Secrets file option          ***********//
//////////////////////////////////////////////////////////////
    #define gateway        //internet gateway (router) in use by destination device
    #ifdef gateway
      #include <secrets.h>  //need to know router ssid in order to use same channel
    #endif  

//////////////////////////////////////////////////////////////  
  #define oledFormat2         // displays all sensors on OLED display
  //#define h2oThreshhold  10 // wet - dry threshhold % for OLED display
  #define h2oThreshhold  1    // wet - dry threshhold % for OLED display
//*******         Compile-time Options           ***********//
//////////////////////////////////////////////////////////////
  //*****************
  // 1. Operatiing Modes
  //*****************
  #define printMode           //option: comment out if no printout to Serial monitor 
  #define OLED_               //option: comment out if no oled display 128x64
  //#define oledFormat1       // displays only temp humidity pressure on OLED display
  bool reportFlag = true;     //issue report to HUB after startup, also when new data exists
  
  //*****************                                                                                    
  // 2. Timing Parameters
  //    NOTE: Wemos D1 Mini 8266 max sleep time is 71 minutes = 4260 seconds
  //*****************

  RTC_DATA_ATTR int bootCount = 0;    //increment by 1 each time the processor wakes up OR experiences chillCount
  #define bootResetCount 6*24         //10 min/wakeup period x6 = 60 min x 24 = 24 hrs; i.e., reset once per day

  //*****************
  //  3. Each sensor board needs unique display title and ssid; 
  //  uncomment one triplet and comment // the others:
  //*****************

//#define ID1  //gar door
//  #define ID2    // car sonic
  #define ID3  //hatch

  #ifdef ID1
    //FL garage or ME basement for me 
    uint8_t sensorID = 1;
    #define displayTitle " ~GARAGE 1~"  
    const char* ssid = "AMBIENT_1";
  
    const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
    const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
    const long chillSeconds = 5;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
  
  
    //Select one platform temperature sensor:
    #define AHT10_        // Adafruit AHT10  //garage id 1
    //#define BME_          // BME280
    //#define DHT_            // DHT11,21,22, etc.
    //#define SHT20_        // DFRobot SHT20
  #endif

  #ifdef ID2
    // ME Bedroom or FL Lanai for me 
    uint8_t sensorID = 2;    
    #define displayTitle " ~SONIC 2~"
    const char* ssid = "AMBIENT_2";
  
    const int sleepSeconds = 0; //8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
    const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
    const long chillSeconds = 10;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
  
    //Select one platform temperature sensor:
    //#define AHT10_        // Adafruit AHT10  //garage id 1
    //#define BME_          // BME280
    //#define DHT_            // DHT11,21,22, etc.
    //#define SHT20_        // DFRobot SHT20
  #endif

  #ifdef ID3
    //FL or ME Kitchen for me 
    uint8_t sensorID = 3;
    #define displayTitle " ~HATCH 3~"
    const char* ssid = "HATCH"
  
    const int sleepSeconds = 0; //600*60;       //8*60 default sleep time in seconds; 0 if no sleep   
    const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
    const long chillSeconds = 5;        //interval between readings if sleepSeconds = 0 slows serial monitor updates
  
    //Select one platform temperature sensor:
    //#define AHT10_        // Adafruit AHT10  //garage id 1
    //#define BME_          // BME280
    //#define DHT_            // DHT11,21,22, etc.
    //#define SHT20_        // DFRobot SHT20
  #endif

  //*****************
  //  4. sensor inventory for each platform
  //*****************
  // arrays to indicate types of sensors aboard each sensor platform (1=presnt, 0 = absent)
  // HUB  id=0; boards are 1,2,3.. example: temps[]={1,0,1,0} indicates hub and sensor #2 have temp sensors, sensor #1 and #3 do not.
  uint8_t temps[] = {1,1,0,1}, hums[]={1,1,0,1}, dbms[]={1,0,0,0}, press[]={0,1,0,1}, bat[]={1,0,0,0};
  uint8_t luxs[] = {0,0,0,0}, h2os[] = {0,0,0,0},doors[]={0,1,0,1},pirs[]={0,1,0,0},sonics[]={0,0,1,0};
  
  //*****************
  // 5. Select one hardware device below and comment out the other: 
  // Board selection impacts pin assignments, setup of pin modes, 
  // pwr on & off pulse duration, and onboard LED on/off states
  //*****************
  #define ESP32           //Recommended choice is esp32
  //#define ESP8266       //use for wemos D1 Mini.  
                          //Multiple D1 Minis seem to interfere with one another and have limited range
  
  //*****************
  // 5. Select one of the 3 following protocols for communication between HUB and sensor platforms; 
  // ESPNOW_1to1 is preferred; NOTE: WIFI is a memory hog:
  //*****************
  //#define WIFI          //set up as wifi server w/o router
  #define ESPNOW_1to1     //send data to one peer: enter MAC address of receiver in next section    
  //#define ESPNOW_1toN   //send to multiple peers: enter MAC addresses of receivers in next section 

  //*****************
  // if selecting ESPNOW_1to1 enter 1 MAC Address for hub; ESPNOW_1toN, enter 4 MAC addresses; 
  //******************
  #ifdef ESPNOW_1to1
    //uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x2B, 0x6D, 0x00};  //example for esp32 #7 MAC Address AC:67:B2:2B:6D:00
    //uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x85, 0x6A, 0x68};  //esp32 #12 Mac Address 8C:AA:B5:85:6A:68
     uint8_t broadcastAddress[] = {0xA8, 0x03, 0x2A, 0x74, 0xBE, 0x8C};  //LILLYGO MAC Address A8:03:2A:74:BE:8C
    //uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0x30, 0x62, 0x94};   // lillygo2: 40:91:51:30:62:94
  #endif
  #ifdef ESPNOW_1toN
    //comment these out and use your own AmbientHUB or other peer MAC addresses:
    uint8_t broadcastAddress1[] = {0xA8, 0x03, 0x2A, 0x74, 0xBE, 0x8C};  //LILLYGO MAC Address A8:03:2A:74:BE:8C
    uint8_t broadcastAddress2[] = {0x78, 0x21, 0x84, 0x7F, 0x82, 0x14};  //esp32 #11 Mac Address 78:21:84:7F:82:14
    uint8_t broadcastAddress3[] = {0x7C, 0x9E, 0xBD, 0xE3, 0xC5, 0xC8};  //esp32 #9 Mac Address 7C:9E:BD:E3:C5:C8
    //uint8_t broadcastAddress4[] = {0x8C, 0xAA, 0xB5, 0x85, 0x6A, 0x68};  //esp32 #12 Mac Address 8C:AA:B5:85:6A:68
    uint8_t broadcastAddress4[] = {0xAC, 0x67, 0xB2, 0x2B, 0x6D, 0x00};  //esp32 #7 MAC Address AC:67:B2:2B:6D:00
  #endif
  
  //*****************                                                                                    //*****************
  // 6. ESPNOW Parameters (if ESPNOW is selected)
  //*****************
  int espNowAttempts=0;            //number of ESPNOW transmission attempts so far (do not adjust)
  #define espNowAttemptsAllowed 30  //number of unsuccessful attempts allowed before sleeping if sleepSeconds > 0
  int espNowDelay = 50;         //delay in msec between espnow attempts

  //*****************
  // 7. ESP32 PIN DEFINITIONS
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
  #ifdef ESP32
    #define pinBoardLED 2               //onboard LED
    #define pinPir 39                   //HC-SR501 PIR sensor                
    #define pinDoor 35                  //door wired to magnetic reed switch, NO & C connected 
                                        //  to pinDoor and ground; 100K pullup R to pinDoor & 3.3v
                                        //  pinDoor = 1 = closed, 0 = open.
                                        //NOTE if this pin is changed, 
                                        //  change setupWakeupConditions as well.
    //#define pinDh2o 18                 //digital moisture digital indicater *                         
    //#define pinAh2o 36                 //analog moisture analog measure *
                                        //   * Hiletgo LM393 FC37 moisture monitor
                                        
    #define pinAh2o 32                  // touchpin                                
    #define pinPhotoCell 34             // analog input 
                                        // esp32 analog input; photocell connected to GPIO pin & gnd, 
                                        // 10K pullup resister, i.e., connected to GPIO pin & 3.3v
    #define pinSDA 21                   // ESP 12C Bus SDA for BME temp sensor, OLED, etc
    #define pinSCL 22                   // ESP 12C Bus SCL for BME temp sensor, OLED, etc

    #define pinSonicTrigger 23          
    #define pinSonicEcho 33
    
  #endif
 
  //*****************
  // 8. ESP8266 D1 Mini PIN DEFINITIONS
  //***************** 
  #ifdef ESP8266
    #define pinBoardLED 2               //onboard LED
    #define pinDoor 14                  //door switch D5
    #define pinDh2o 13                  //digital flood sensor input D7                             
    //#define pinAh2o                     //analog flood sensor not connected
    #define pinPhotoCell A0             //analog input 
  #endif

  //*****************
  // 9. Temp Sensor Libraries
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

  uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
  uint8_t wakeupID;                   //reason sensor wode up; see readWakeupID 

  // Platform sensor structure
  // Structure to send data must match the receiver structure:
  //*****************
   typedef struct platforms {  // create an definition of platformm sensors
      uint8_t id;              // id must be unique for each sender; HUB  id=0; boards are 1,2,3..
      float temperature;       // F
      float humidity;          // %
      float pressure;          // mb
      uint8_t lux;             // 0-99 % of full illumination
      uint8_t aH2o;            // 0-99 % of full sensor level detection
      uint8_t dH2o;            // 0 or 1 digital readout 0=dry  (normal state)
      uint8_t doorCount;       // # times door opened and closed after previous report.
      uint8_t door;            // door = 0 when closed, 1 when open
      uint8_t pir;             // # times pir detected motion after previous report
      uint16_t sonic;           // 0=absent 1=present
      uint8_t sendFailures;    // # failed attempts to send via espNow
  } platforms;
  
  #ifdef ESP32  //store the reaadings persistently if esp32
    RTC_DATA_ATTR platforms sensorData ={sensorID,0,0,0,0,0,0,0,0,0,0,0};     
  #endif
  #ifdef ESP8266
    platforms sensorData = {sensorID,0,0,0,0,0,0,0,0,0,0,0};   to initialize  
  #endif  
  uint8_t aH2oMeasured = 0; // sensor measurement prior to preocessing
  RTC_DATA_ATTR uint8_t priorDoor = 0; //last door status = 0 (closed) or 1 (open)
  String hum="--",tem="--",pres="--"; //adjusted values sent to hub if wifi used: "--" if nan or current reading if valid
  //String payload ="11,22,3333,4,55,66,77,----"; future plan

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
  #ifdef WIFI
    #ifdef ESP32
      #include <WiFi.h>                  // wifi libr for esp32
    #endif
    #ifdef ESP8266
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
  #ifdef ESPNOW_1to1 
    #include <esp_now.h>
    #ifdef ESP32
      #include <WiFi.h>                  // wifi libr for esp32
      #include <esp_wifi.h>
    #endif
    #ifdef ESP8266
      #include <ESP8266WiFi.h>                // wifi libr for esp8266
      #include <ESPAsyncTCP.h>           //added for esp8266
    #endif  
    esp_now_peer_info_t peerInfo;   // Create peer interface if sending
  #endif

  //*****************
  // ESP NOW One to Many libraries
  //*****************
  #ifdef ESPNOW_1toN
    #ifdef ESP32
      #include <WiFi.h>                  // wifi libr for esp32
    #endif
    #ifdef ESP8266
      #include <ESP8266WiFi.h>           // wifi libr for esp8266
      #include <ESPAsyncTCP.h>           //added for esp8266
    #endif                  
    #include <esp_now.h>  
    esp_now_peer_info_t peerInfo;
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
/////////////////////////////////////
//*****       Setup()      *********
////////////////////////////////////
void setup(){
  //Read sensors, 1) setup wifi server OR 2) transmit data using ESPNOW and go to sleep in less than 2 sec!
  delay(200);                   //ESP32 Bug workaround -- don't reference rtc ram too soon!!
  serialPrintVersion();         //Show startup info to serial monitor if printMode enabled
  setupPinModes();
  wakeupID = readWakeupID();    //process wakeup sources & return wakeup indicator
  restartIfTime();              //restart ea bootcount>bootResetCount if timer wakeup else increment bootCount 
  setupOledDisplay();           //prepare the oled display if #define includeOled is enabled
  displayVersion();             //display version and blink onboard led 5 sec on bootCount=1
  turnOnBoardLED();             //illuminate the on-board LED
  //attachInterrupt(digitalPinToInterrupt(pinPir), pirInterrupt, RISING);  //in case you want to try it!
  //attachInterrupt(digitalPinToInterrupt(pinDoor), doorChangeInterrupt, CHANGE); //in case you want to try it!
  setupSensors();               //initialize sensors 
  readDoor();                   //read door status. NOTE door is wakeup trigger, also polled during loop
  //readPir();                  //read pir status. NOTE pir is wakeup trigger; updated during readWakeupID
  readAh2o();                   //read analog flood level value
  readDh2o();                   //read digital flood indicater
  readPhotoCell();              //read photocell value
  readSonic();
  readTemperature();            //valid data updates temperature and temp; invalid updates only temp with  "--"
  readHumidity();               //valid data updates humidity and hum; invalid updates only hum with  "--"
  readPressure();               //valid data updates pressure and pres; invalid updates only pres with  "--"              
  printSensorData();
  displayStatus();              //display latest valid sensor readings on oled display if #define incledeOled is enabled
  
  setupESPNOW_1to1();           //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                                //successful data transfer will cause the system to sleep if sleepSeconds > 0
 
  setupESPNOW_1toN();           //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
                                //will NOT cause system to go to sleep if sleepSeconds>0 and data was received successfully

  setupWifiServer();            //initialize wifi server if #define WIFI is enabled

  turnOffBoardLED();            //turn off the board LED when setup complete
}

//////////////////////////////////
//*****      Loop()      *********** 
/////////////////////////////////
void loop(){                  //Execute repeatedly if system did not go to sleep during setup:
  readDoor();
  //readPir();                //read pir status via wakeup rather than polling
  readAh2o();
  readDh2o();
  readPhotoCell();  

  if(chillTimeIsUp(chillSeconds*1000)==1){   //slow down loop for serial monitor readability + sensor R&R   

    if (sleepSeconds == 0){   //if unit does not sleep, read all sensors if chillSeconds has elapsed since last reading
      //but first, check for reboot conditions 
      bootCount++;            //increment counter
      if (bootCount>=bootResetCount){
        ESP.restart();        //Reboot the esp32 to prevent memory issues
      }                              
      readSonic();            //sonic gives false readings if triggered too quickly     
      readTemperature();      //valid data updates temperature and temp; invalid updates only temp with  "--"
      readHumidity();         //valid data updates humidity and hum; invalid updates only hum with  "--"
      readPressure();         //valid data updates pressure and pres; invalid updates only pres with  "--"                 
      displayStatus();        //display latest vald sensor readings on oled display if #define incledeOled is enabled
    }else{ 
                       
      //Set wakeup conditions and go to sleep if it is nap time
      if (sensorNapTime(awakeSeconds*1000) ==1 ){        //determine if awakeSeconds have elapsed
        setupWakeupConditions();  //interrupt if door or PIR state changes
        #ifdef printMode        //issue sleep message if #define printMode is enabled
          Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif 
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);   //go to sleep for sleepSeconds 
      }
    }
    reportFlag=true;          //resend sensor data every chillSeconds even if no change; 
                              //send in main loop in case multiple tries are necessary.
    printSensorData();
  }
  if (reportFlag){            //Send data each time it changes as indicated by reportFlag
    sendEspNow_1to1();        //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                              //successful data transfer will cause the system to sleep if sleepSeconds > 0                                

    sendEspNow_1toN();        //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
  }                                
}                             //repeat at top of the loop
//*********************************
void blinkBoardLED(int sec){      // Blink board LED for sec seconds at .5 second intervals
  #ifdef printMode 
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
void displayStatus(){  
  //display sensor readings on oled display if #define OLED_ is enabled
  #ifdef printMode
    Serial.println(F("*displayStatus*"));
  #endif  
  
  #ifdef OLED_  
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
      #ifdef printMode
        Serial.print(" tem, Temperature: ");Serial.print(tem);Serial.print(", ");Serial.print(iTemperature);Serial.println(" *F");
      #endif 
      if(tem=="--"){
        oled.print (" F?");
      }else{ 
        oled.print(" F ");
      }

      #ifdef printMode
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
      #ifdef printMode
        Serial.print(" hum, Humidity: ");Serial.print(hum);Serial.print(F(", "));Serial.print(iHumidity);Serial.println(F("%"));
      #endif  

      if(luxs[sensorID]==1){
        oled.print ("LIGHT   ");
        #ifdef printMode
           Serial.print(" lux: ");Serial.println(sensorData.lux);
        #endif  
        oled.print(sensorData.lux);
        oled.println("%      ");
      }  
    
      if(h2os[sensorID]==1){
        if(sensorData.aH2o - h2oThreshhold > 0){  //display leak status in real time
          oled.print("WET");
          oled.print(aH2oMeasured);
          #ifdef printMode
            Serial.println(" FLOOR aH2o WET");
          #endif  
        }else{
          oled.print("DRY");
          oled.print(aH2oMeasured);
          #ifdef printMode
            Serial.print(" FLOOR aH2o DRY; aH2oMeasured = ");Serial.println(aH2oMeasured);
          #endif  
        }
      }
      
      if(pirs[sensorID]==1){
          oled.print(" m");
          oled.print(sensorData.pir);
          #ifdef printMode
            Serial.print(" MOTION = "); Serial.println(sensorData.pir);
          #endif  
      }
      
      if(doors[sensorID]==1){
        if(digitalRead(pinDoor)==1) {
          oled.print(" SH");
          #ifdef printMode
            Serial.println(" DOOR SHUT");
          #endif  
        }else{
          oled.print(" OP");
          #ifdef printMode
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
      //printSensorData();
      //data sent success; reset count parameters
      sensorData.doorCount = 0;
      sensorData.pir = 0;
      sensorData.sendFailures=0;
      
      if(sleepSeconds==0){
        reportFlag=false;        
      }else{
        setupWakeupConditions();
        #ifdef printMode
          Serial.print(F(" sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif 
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);
      }  
    }else{
      sensorData.sendFailures ++ ;
      Serial.print("sensorData.sendFailures = ");Serial.println(sensorData.sendFailures);
      if (sensorData.sendFailures >= espNowAttemptsAllowed){ 
        sensorData.sendFailures = 0;  //must reset here otherwise only one send attemp next times 
        if(sleepSeconds==0){
          //reportFlag=false;        
        }else{
          setupWakeupConditions();  //interrupt if door state changes
          #ifdef printMode
            Serial.print(F(" sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
          #endif  
          ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);   
        }       
      }
    }
  }
#endif

//***ESP NOW One to Many Callback function***************
#ifdef ESPNOW_1toN
  void ESPNOW_1toN_OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // callback verifies successful receipt after message was sent (see loop below for sendEspNow_1toN function)
  
    #ifdef printMode 
      //print hte MAC address and whether received successfully or not   
      Serial.println(F("*OnDataSent*"));
      char macStr[18];
      Serial.print(F("Packet to: "));
      //Copy the sender mac address to a string
      snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      Serial.print(macStr);
      Serial.print(F(" send status:\t"));
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif
  }
#endif

//*********************************
String getAh2o(){
  #ifdef printMode
    Serial.println(F("*getAh2o*"));
  #endif  
  return String(sensorData.aH2o);
}

//***********************************
String getDh2o(){
  #ifdef printMode
    Serial.println(F("getDh2o*"));
  #endif  
  return String(sensorData.dH2o);
}

//***********************************
String getTemperature() {  
  //http GET retrieves latest valid reading
  #ifdef printMode
    Serial.println(F("*getTemperature*"));
    Serial.println(tem);
  #endif  
  turnOnBoardLED();  
  return tem;
}

//***********************************
String getDoor(){
  #ifdef printMode
    Serial.println(F("getDoor*"));
    Serial.print(" door = ");Serial.println(sensorData.door);
  #endif  
  return String(sensorData.door);
}

//*********************************
String getPhotoCell(){ 
  #ifdef printMode
    Serial.print(F("getPhotoCell*")); 
    Serial.print(" lux: ");Serial.println(sensorData.lux);
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
  #ifdef printMode
    Serial.println(F("*getHumidity*"));
    Serial.println(hum);
  #endif  
  turnOffBoardLED();
  return hum;
}

//***********************************
String getPressure() {   
  //http GET retrieves last reading
  #ifdef printMode
    Serial.println(F("*getPressure*"));
    Serial.println(pres);
  #endif  
  return pres;
}

//**************************************
#ifdef gateway
//Determine the active wifi channel being used by the wifi gateway.
//Credits: https://github.com/m1cr0lab-esp32/esp-now-network-and-wifi-gateway
//
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
//*******************************
void printSensorData(){
  #ifdef printMode  
    Serial.println(F("*printSensorData*"));
    Serial.println(F("id      temp    hum    pres    lux    aH2o    dH2o    door   doorCount  sonic   pir  Fails"));    
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
    Serial.print(sensorData.sendFailures);Serial.println(F("\t"));
  #endif  
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

//***********************************
void readAh2o(){
  //Read analog value and convert to % as whole number 0-99
  if(h2os[sensorID]==1){
    #ifdef printMode
      Serial.print(F("*readAh2o* "));
    #endif  

    //sensorData.aH2o = 2.44*(4095-analogRead(pinAh2o))/100;   //whole number 1-99 %
    aH2oMeasured =  touchRead(pinAh2o);
    if(aH2oMeasured > 5){
      aH2oMeasured = 5;  
    }
    sensorData.aH2o = 5 - aH2oMeasured;
    #ifdef printMode
      Serial.println(sensorData.aH2o);//Serial.println(F("% "));
    #endif
  }
}

//***********************************
void readDh2o(){
  if(h2os[sensorID]==1){
    #ifdef printMode
      Serial.print(F("*readDh2o* "));
    #endif  

    //sensor reads 1 when dry, 0 when wet
    //we want 0=dry (normal status)
    //sensorData.dH2o = !digitalRead(pinDh2o);

    #ifdef printMode
      Serial.println(sensorData.dH2o);
    #endif
  }  
}

//***********************************
void readDoor(){
  if (doors[sensorID]==1){

/*    #ifdef printMode
      Serial.print(F("*readDoor*"));
      Serial.print(F(" pinDoor : "));Serial.print(digitalRead(pinDoor));
      //Serial.print(F(" priorDoor : "));Serial.print(sensorData.door);
      Serial.print(F(" doorCount: "));Serial.println(sensorData.doorCount);
    #endif
    */
    //door = 1 when closed, 0 when open due to pullup resister
    //we invert to report 0 = closed so report csv shows all 0 for normal state
    if(sensorData.door == digitalRead(pinDoor)){  //if door state changed
        sensorData.door = !digitalRead(pinDoor); //report door state 0 if closed, 1 if open
        sensorData.doorCount++;                  //increment doorCount
        reportFlag = true;                       //report right away 
    }
    
    
    
    
    
    /*
    if (sensorData.doorCount > 0) {
      if (priorDoor == digitalRead(pinDoor)){
        //do nothing if door state is unchanged
      }else{
        //
        sensorData.door = !digitalRead(pinDoor);  //report door state 0 if closed, 1 if open
        sensorData.doorCount++;                  //increment doorCount
        priorDoor=sensorData.door;                //save dooor status for later comparison
        reportFlag = true;
      }
    }else{
      //doorCount = 0 (no prior unreported open doors) processing:
      sensorData.door = !digitalRead(pinDoor);  //report door state 0 if closed, 1 if open
      sensorData.doorCount=sensorData.door;     //set doorCount 0 if closed, 1 if open
      priorDoor=sensorData.door;                //save door status for later comparison
      //reportFlag=true;  //not sure we need it
    }*/
  }  
  #ifdef printMode
    //Serial.print("Exit door: ");Serial.println(sensorData.door);
  #endif
}

//*********************************
void readHumidity() {  
  //valid data updates sensorData.humidity and hum; invalid updates only hum with  "--"
  #ifdef printMode
    Serial.print(F("*readHumidity* "));
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
   }else{sensorData.humidity = x;
  }
  #ifdef printMode
    Serial.print(F("humidity, hum: "));Serial.print(sensorData.humidity);Serial.print(F(", "));Serial.println(hum);
  #endif
}

//***********************************
void readPhotoCell(){
  //Read analog value and convert to % as whole number 0-99
  if (luxs[sensorID]==1){
    #ifdef printMode
      Serial.print(F("*readPhotoCell* "));
    #endif  

    sensorData.lux = 2.44*((4095-analogRead(pinPhotoCell))/100);  //whole number 1-99 %
  
    #ifdef printMode
      Serial.print(F("lux%, A/D: "));Serial.print(sensorData.lux); Serial.print(F("% "));
      Serial.println(analogRead(pinPhotoCell)); 
    #endif  
  }  
}

//***********************************
void readPir(){
  if (pirs[sensorID]==1){
    #ifdef printMode
      Serial.print(F("*readPir* "));
    #endif

    //pir = 0 when no motion (normal state)
    sensorData.pir = digitalRead(pinPir) + sensorData.pir ;
    //pirCount=pirCount+sensorData.pir;    
  }  
  #ifdef printMode
    Serial.println(sensorData.pir);
  #endif
}
//*************************************
void readSonic(){
  if(sonics[sensorID]==1){
    #ifdef printMode
      Serial.println(F("*reasdSonic*"));Serial.print(F("entry reportFlag: "));Serial.println(reportFlag);
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
    Serial.printf("\n\nDistance = %d cm\n", distance);  Serial.printf("\n\nPrior Distance = %d cm\n", sensorData.sonic);

    if (abs(distance-sensorData.sonic)>=6 ) {
      sensorData.sonic=distance;       
      reportFlag = true;
    }    
    
    #ifdef printMode
      Serial.print(F("exit reportFlag: "));Serial.println(reportFlag);
    #endif
  }
}

//***********************************
void readPressure(){   
  //valid data updates sensorData.pressure and pres; invalid updates only pres with  "--"
  #ifdef printMode
    Serial.print(F("*readPressure* "));
  #endif 
  float x = 0; 
  pres="--"; //indicate no reading
  #ifdef BME_ 
    x = bme.readPressure()/100.0;    //millibars
    pres = String(x);
    if (pres == "nan") {pres = "--";
      }else{sensorData.pressure = x;
    }
  #endif                                                                    
  #ifdef printMode
    Serial.print(F("pressure, pres: "));Serial.print(sensorData.pressure);Serial.print(F(", "));Serial.println(pres);
  #endif  
}

//***********************************
void readTemperature() {  
  //valid data updates sensorData.temperature and temp; invalid updates only temp with  "--"
  #ifdef printMode
    Serial.print(F("*readTemperature* "));
  #endif 
  float x = 0; 
  #ifdef AHT10_
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    x=temp.temperature;
    x=1.8*x+32.0;
  #endif 
  #ifdef DHT_
    x=dht.readTemperature(true);  //true = *F
  #endif
  #ifdef BME_
    x=1.8*bme.readTemperature()+32.0;
  #endif 
  #ifdef SHT20_
    x = 1.8*sht20.readTemperature()+32.0;
  #endif
  tem=String(x);
  if (tem=="nan"){tem = "--";
  }else{sensorData.temperature = x;
  }

  #ifdef printMode 
    Serial.print(F("temperature, tem: "));Serial.print(sensorData.temperature);Serial.print(F(", "));Serial.println(tem);
  #endif    
}

//*************************************
uint8_t readWakeupID(){
  //read wakeup reason and return code
  #ifdef printMode 
    Serial.println(F("*readWakupID*"));
  #endif   
  reportFlag=true;
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
       sensorData.door = !digitalRead(pinDoor);
       sensorData.doorCount ++ ;// tracks # changes between reports
       //priorDoor=!digitalRead(pinDoor);
       #ifdef printMode 
         Serial.print(F("Door: "));Serial.print(sensorData.door);Serial.print(F(" doorCount: "));Serial.println(sensorData.doorCount);
       #endif
       return 0;
    case ESP_SLEEP_WAKEUP_EXT1 :      
       sensorData.pir ++; //= sensorData.pir + digitalRead(pinPir);
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
    Serial.print(F("*restartIfTime* bootCount = "));Serial.print(bootCount);Serial.print(F(" \t\pir = "));
    Serial.println(sensorData.pir);  
  #endif     
  if (bootCount>=bootResetCount){  //if bootCount meets threshhold AND:
    if (wakeupID==2){               //timer caused wakeup
      if(sensorData.pir<=3){        //no unreported pir data(allowing for a couple false hits)
        if(sensorData.door==0){     //no unreported door data
          #ifdef printMode
            Serial.println("");delay (5000);
            Serial.println(F("Rebooting.............*************.............")); 
          #endif                    
          ESP.restart();            //Reboot the esp32 to prevent memory issues
        }  
      } 
    }
  }      
  bootCount++;
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

    esp_err_t espResult = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData)); // Send message via ESP-NOW
    #ifdef printMode
      if (espResult == ESP_OK) { 
        Serial.println(F(" Sent with success"));
        delay(espNowDelay);  //delay to prevent resending prior to response
      }else{
        Serial.println(F(" Error sending the data"));  
      }
    #endif
    delay(espNowDelay);
  #endif
}

//***********************************
void sendEspNow_1toN(){
  //format data and send to multiple ESPNOW peers if #define espnow_1toN is enabled
  #ifdef ESPNOW_1toN
    #ifdef printMode
      Serial.print(F("*sendEspNow1toN*"));
    #endif
   esp_err_t espResult = esp_now_send(0, (uint8_t *) &sensorData, sizeof(sensorData)); // Send message via ESP-NOW
    #ifdef printMode 
      if (espResult == ESP_OK) {
        Serial.println(F( "Sent with success"));
      }
      else {
        Serial.println(F(" Error sending the data"));
      }
    #endif
    delay(espNowDelay);
  #endif                  
}  

//***********************************
static int sensorNapTime(long msec){    
  //Read data at timed intervals  
  //#ifdef printMode
  //  Serial.println(F("*sensorNapTime*"));
  //#endif  
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
void serialPrintVersion(){      //Show startup info to serial monitor if printMode enabled
  #ifdef printMode              //print the following to status monitor if printMode defined:
    Serial.begin(115200);
    turnOnBoardLED();           //turn on board LED during setup
    Serial.println(F(""));
    Serial.print(F("***** "));Serial.print(F(APP)); Serial.print(F(VERSION)); 
    Serial.println(displayTitle);
    Serial.println(F("----------------------------")); 
    printWakeupID(wakeupID);
    printSensorData();        //print test data on first boot, previous data thereafter
  #endif
}
  
//*********************************
void setupESPNOW_1to1(){
  //initializes espnow, registers the peer's MAC address, and registers the peer
  #ifdef printMode
    Serial.println(F("*setupESPNOW_1to1*"));
  #endif
  #ifdef ESPNOW_1to1
    sensorData.id = sensorID;
    WiFi.mode(WIFI_STA);

    #ifdef gateway
      //Use the active wifi channel being used by the wifi gateway.
      //Credits: https://github.com/m1cr0lab-esp32/esp-now-network-and-wifi-gateway
      //
      int32_t channel = getWiFiChannel(SECRET_WIFI_SSID);
      //WiFi.printDiag(Serial); // Verify channel number before
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      //WiFi.printDiag(Serial); // verify channel change
    #endif
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      #ifdef printMode
        Serial.println(F("Error initializing ESP-NOW"));
      #endif  
      return;
    }
  
    // register for Send CB to get the status of Trasnmitted packet
    esp_now_register_send_cb(ESPNOW_1to1_OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
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
}

//*********************************
void setupESPNOW_1toN(){
  #ifdef printMode
    Serial.println(F("*setupESPNOW_1toN*"));
  #endif

  //initializes espnow, registers the peer MAC addresses, and registers four peers
  #ifdef ESPNOW_1toN

    sensorData.id = sensorID;
  
    WiFi.mode(WIFI_STA);
   
    if (esp_now_init() != ESP_OK) {
      #ifdef printMode
        Serial.println(F("Error initializing ESP-NOW"));
      #endif  
      return;
    }
    
    esp_now_register_send_cb(ESPNOW_1toN_OnDataSent);
     
    // register peer
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // register first peer  
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      #ifdef printMode
        Serial.println(F("Failed to add peer1"));
      #endif  
      return;
    }
    // register second peer  
    memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      #ifdef printMode
        Serial.println(F("Failed to add peer2"));
      #endif      
      return;
    }
    // register third peer
    memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      #ifdef printMode
        Serial.println(F("Failed to add peer3"));
      #endif  
      return;
    }
    // register fourth peer
    memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      #ifdef printMode
        Serial.println(F("Failed to add peer4"));
      #endif  
      return;
    }
 
  #endif
}

//*********************************
void setupOledDisplay(){
  //setup OLED display on I2C bus
  #ifdef printMode
    Serial.println(F("*setupOledDisplay*"));
  #endif
  #ifdef OLED_
    //if(bootCount==0){  //only initialize on restart so OLED doesnt CLEAAR
      Wire.begin();  
      #if OLED_RESET  >= 0
        oled.begin(&Adafruit128x64, I2C_ADDRESS, OLED_RESET);
      #else 
        oled.begin(&Adafruit128x64, I2C_ADDRESS);
      #endif
   // } 
  #endif 
}

//*********************************
void setupPinModes(){                
  //Set Pin Modes INPUT / OUTPUT / PULLUP
  #ifdef printMode
    Serial.println(F("*setupPinModes*"));
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
    #ifdef ESP32
      pinMode (pinAh2o, INPUT);
      pinMode (pinDh2o, INPUT_PULLUP);
    #endif
    #ifdef ESP8266
      pinMode (pinDh2o, INPUT);
    #endif  
  }
*/    
}

//*********************************
void setupSensors(){
  //initializes sensors
  #ifdef printMode
    Serial.println(F("*setupSensors*"));
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
       #ifdef printMode
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
void setupWakeupConditions(){
  //set wakeup conditions based on door & PIR sensor
  #ifdef printMode
    Serial.print(F("*setupWakeupConditions*"));
    Serial.print(F(" door --> "));Serial.print(!digitalRead(pinDoor));
    Serial.println(F("; pir --> 1"));//Serial.println(sensorData.pir);
  #endif 

  //Set up door causes wakeup when opened or closeed
  if(doors[sensorID]==1){          //if this sensor platform has a door sensor:
    if (digitalRead(pinDoor)==1){  //determine wakeup condition based on door closed or open
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0); //0 = High OPEN 1 = Low CLOSED
     }else{
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,1);  
    } 
  }  

  //set up pir causes wakeup when movement detected
  if(pirs[sensorID]==1){                //if this sensor platform has a pir sensor:
    #define wakeupBitmask 0x4000000000  //wakeup on pir gpio 39
    esp_sleep_enable_ext1_wakeup(wakeupBitmask,ESP_EXT1_WAKEUP_ANY_HIGH);
  }  
}      

//*************************************
void setupWifiServer(){
  //Initializes WIFI server for access to clients that issue GET commands to retrieve sensor data
  #ifdef WIFI
    #ifdef printMode
      Serial.println(F("*setupWifiServer*"));
      Serial.print(F("sensorID: "));Serial.println(sensorID);
      Serial.print(F("SSID: "));Serial.println(ssid);
    #endif
    //WiFi.softAP(ssid, password);   //use this if you want a passsword
    WiFi.softAP(ssid);   // OMIT password for open AP 
    delay(100);
    //IPAddress Ip(192,168,4,sensorID);
    IPAddress Ip(192,168,4,1);
    IPAddress NMask(255,255,255,0);
    WiFi.softAPConfig(Ip,Ip,NMask);
    
    IPAddress IP = WiFi.softAPIP();
    #ifdef printMode
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
    Serial.println(F("*turnOnBoardLED*"));
  #endif    
  #ifdef ESP8266
    digitalWrite(pinBoardLED, LOW);
  #endif  
  #ifdef ESP32
    digitalWrite(pinBoardLED, HIGH);
  #endif  
} 

//*********************************    
void turnOffBoardLED(){             
  // Turn board LED off
  #ifdef testMode
    Serial.println(F("*turnOffBoardLED*"));
  #endif  
  #ifdef ESP8266
    digitalWrite(pinBoardLED, HIGH);
  #endif  
  #ifdef ESP32
    digitalWrite(pinBoardLED, LOW);
  #endif 
}    
