
const char* VERSION = "AmbientAP 2022 v7.19";

/*////////////////////////////////////////////////////////////////////////////////////

I HOPE THIS SOFTWARE IS USEFUL TO YOU. 

Copyright 2022 Robert Jessup  MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE
*/////////////////////////////////////////////////////////////////////////////////////

/*//////////////////////////////////////////////////////////////////////////////////// 
The code within the '#ifdef ESPNOW_1to1' and '#ifdef ESPNOW_1toN' to #endif 
compiler directives is adapted from the following:
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
*////////////////////////////////////////////////////////////////////////////////////

/************************************************************************************
AmbientAP is a flexible, multi-featured sensor platform.  It can:
  1. expose sensor values via http GET command requests from a client.
  2. expose sensor values via lighter footprint ESPNOW protocol which pushes data to a peer/client periodically without receiving a request.
  3. operate on ESP32 or ESP8266 (including d1 mini) controller.
  4. accommodate BME280 (via I2C) or dht sensors (via pin 5).
  5. report to 1 ESPNOW peer or up to 4 peers.
  6. sleep and wake up via a timer, an interrupt (such as a window being opened), or it can stay awake.
  7. sleep immediately after sending a successfully received ESPNOW message when configured as a 1to1 peer.
  8. operate with or without an OLED display.
  9. operate as one of three such devices using this software by assigning unique SensorID 1, 2, or 3 to each sensor platform.

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
//*******         Compile-time Options           ***********//
//////////////////////////////////////////////////////////////

  //*****************
  // 1. Operatiing Modes
  //*****************
  #define printMode           //option: comment out if no printout to Serial monitor
  
  #define OLED_               //option: comment out if no oled display 128x64
  //#define oledFormat1       // displays only temp humidity pressure on OLED display
  #define oledFormat2         // displays all sensors on OLED display
  #define h2oThreshhold  10   // wet - dry threshhold % for OLED display

  //*****************                                                                                    //*****************
  // 2. Timing Parameters
  //    NOTE: Wemos D1 Mini 8266 max sleep time is 71 minutes = 4260 seconds
  //*****************
  const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
  const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
  const long chillSeconds = 30;        //interval between readings if sleepSeconds = 0 slows serial monitor updates

  RTC_DATA_ATTR int bootCount = 0;    //increment by 1 each time the processor wakes up
  #define bootResetCount 6*24         //10 min/wakeup period x6 = 60 min x 24 = 24 hrs; i.e., reset once per day
  
  //*****************
  // 3. Select one hardware device below and comment out the other: 
  // Board selection impacts pin assignments, setup of pin modes, 
  // pwr on & off pulse duration, and onboard LED on/off states
  //*****************
  #define ESP32           //Recommended choice is esp32
  //#define ESP8266       //use for wemos D1 Mini.  
                          //Multiple D1 Minis seem to interfere with one another and have limited range

  //*****************
  // 4. Select one platform temperature sensor:
  //****************
  //#define BME_          // BME280
  #define DHT_            // DHT11,21,22, etc.
  
  //*****************
  // 5. Select one of the 3 following protocols for communication between HUB and sensor platforms; 
  // ESPNOW_1to1 is preferred; NOTE: WIFI is a memory hog:
  //*****************
  //#define WIFI          //set up as wifi server
  #define ESPNOW_1to1     //send data to one peer: enter MAC address of receiver in next section    
  //#define ESPNOW_1toN   //send to multiple peers: enter MAC addresses of receivers in next section 
  
  //*****************
  // if selecting ESPNOW_1to1 enter 1 MAC Address for hub; ESPNOW_1toN, enter 4 MAC addresses; 
  //******************
  #ifdef ESPNOW_1to1
    uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x2B, 0x6D, 0x00};  //example for esp32 #7 MAC Address AC:67:B2:2B:6D:00
                                                                         // lillygo 2: 40:91:51:30:62:94
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
  #define espNowAttemptsAllowed 3  //number of unsuccessful attempts allowed before sleeping if sleepSeconds > 0
  int espNowDelay = 10000;         //delay in msec between espnow attempts

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
    #define pinDoor 35                  //door wired to magnetic reed switch, NO & C connected 
                                        //  to pinDoor and ground; 100K pullup R to pinDoor & 3.3v
                                        //  pinDoor = 1 = closed, 0 = open.
                                        //NOTE if this pin is changed, 
                                        //  change setupInteruptConditions as well.
    #define pinDh2o 18                 //digital moisture digital indicater *                         
    #define pinAh2o 36                 //analog moisture analog measure *
                                        //   * Hiletgo LM393 FC37 moisture monitor
    #define pinPhotoCell 34             // analog input 
                                        // esp32 analog input; photocell connected to GPIO pin & gnd, 
                                        // 10K pullup resister, i.e., connected to GPIO pin & 3.3v
    #define pinSDA 21                   // ESP 12C Bus SDA for BME temp sensor, OLED, etc
    #define pinSCL 22                   // ESP 12C Bus SCL for BME temp sensor, OLED, etc
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
  // 9. Sensor Libraries
  //*****************
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

  //*****************
  //  10. Each sensor board needs unique display title and ssid; 
  //  uncomment one triplet and comment // the others:
  //*****************
  //uint8_t sensorID = 1;
  //#define displayTitle " ~AMBIENT 1~"  
  //const char* ssid = "AMBIENT_1";
  
  uint8_t sensorID = 2;    
  #define displayTitle " ~AMBIENT 2~"
  const char* ssid = "AMBIENT_2";
  
  //uint8_t sensorID = 3;
  //#define displayTitle " ~AMBIENT 3~"
  //const char* ssid = "AMBIENT_3";

////////////////////////////////////////////////////////////////////////////
//*********            End of Compile-time Options           ***************
////////////////////////////////////////////////////////////////////////////

  uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds

  //*****************
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
      uint8_t door;            // 0 or 1 door open or closed. door = 0 when closed (normal state)
  } platforms;
  
  #ifdef ESP32  //store the reaadings persistently if esp32
    RTC_DATA_ATTR platforms sensorData ={sensorID,0,0,0,0,0,0,0};     
  #endif
  #ifdef ESP8266
    platforms sensorData = {sensorID,0,0,0,0,0,0,0};   to initialize  
  #endif  

  String hum="--",temp="--",pres="--"; //adjusted values sent to hub if wifi used: "--" if nan or current reading if valid
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
    #ifdef ESP32
      #include <WiFi.h>                  // wifi libr for esp32
    #endif
    #ifdef ESP8266
      #include <ESP8266WiFi.h>           // wifi libr for esp8266
      #include <ESPAsyncTCP.h>           //added for esp8266
    #endif  
    #include <esp_now.h>
    esp_now_peer_info_t peerInfo;   // Create peer interface              
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

/////////////////////////////////////
//*****       Setup()      *********
////////////////////////////////////
void setup(){
  //Execute one time setup, read and display sensor values, and setup wifi server OR transmit data using ESPNOW.
  setupPinModes();
  #ifdef printMode              //print the following to status monitor if printMode defined:
    Serial.begin(115200);
    turnOnBoardLED();           //turn on board LED during setup
    Serial.println(F(""));delay (5000);  //allow time for user to switch to status monitor after compilation
    Serial.print(F(VERSION));Serial.print(F(" - ")); Serial.println(F(displayTitle));
    Serial.println(F("----------------------------")); 
    printWakeupReason();
    printSensorData();
  #endif
  restartIfTime();              //restart ea 24 hrs  
  readDoor();                   //read door status
  readAh2o();                   //read analog flood level value
  readDh2o();                   //read digital flood indicater
  readPhotoCell();              //read photocell value
  setupSensors();               //initialize sensors (can delay 2-4 seconds for sensor ready)
  readTemperature();            //valid data updates temperature and temp; invalid updates only temp with  "--"
  readHumidity();               //valid data updates humidity and hum; invalid updates only hum with  "--"
  readPressure();               //valid data updates pressure and pres; invalid updates only pres with  "--"              
  setupOledDisplay();           //prepare the oled display if #define includeOled is enabled
  displayStatus();              //display latest valid sensor readings on oled display if #define incledeOled is enabled
  setupWifiServer();            //initialize wifi server if #define WIFI is enabled
  
  setupESPNOW_1to1();           //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                                //successful data transfer will cause the system to sleep if sleepSeconds > 0
 
  setupESPNOW_1toN();           //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
                                //will NOT cause system to go to sleep if sleepSeconds>0 and data was received successfully
  printSensorData();
  
  turnOffBoardLED();            //turn off the board LED when setup complete
}

//////////////////////////////////
//*****      Loop()      *********** 
/////////////////////////////////
void loop(){ 
  //Execute repeatedly if system did not go to sleep during setup
  if (sleepSeconds == 0){      //if unit does not sleep, read all sensors if chillSeconds has elapsed since last reading                              
    if(chillTimeIsUp(chillSeconds*1000)==1){   //slow down loop for serial monitor readability + sensor R&R   
      readDoor();
      readAh2o();
      readDh2o();
      readPhotoCell();       
      readTemperature();      //valid data updates temperature and temp; invalid updates only temp with  "--"
      readHumidity();         //valid data updates humidity and hum; invalid updates only hum with  "--"
      readPressure();         //valid data updates pressure and pres; invalid updates only pres with  "--"                 
      displayStatus();        //display latest vald sensor readings on oled display if #define incledeOled is enabled
    }   
  }else{                  
    //Set interrupt conditions and go to sleep if it is nap time
    if (sensorNapTime(awakeSeconds*1000) ==1 ){        //determine if awakeSeconds have elapsed
      setupInterruptConditions();  //interrupt if door state changes
      #ifdef printMode        //issue sleep message if #define printMode is enabled
        Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
      #endif 
      ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);   //go to sleep for sleepSeconds 
    }
  }
  sendEspNow_1to1();          //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                              //successful data transfer will cause the system to sleep if sleepSeconds > 0                                

  sendEspNow_1toN();          //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
}                             //repeat at top of the loop

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
    oled.clear(); 
    oled.setFont(fixed_bold10x15);
    oled.println(displayTitle);       //top line is display title
    #ifdef oledFormat1
      oled.print("  ");oled.print(sensorData.temperature);oled.println(" F");
      oled.print("  ");oled.print(sensorData.humidity);oled.println(" %");
      oled.print("  ");oled.print(sensorData.pressure);oled.println("mb"); 
    #endif
    #ifdef oledFormat2 
      int iTemperature=sensorData.temperature + .5;
      if(temp=="--"){
        oled.print ("?");
      }
      oled.print(iTemperature);
      #ifdef printMode
        Serial.print("temp, Temperature: ");Serial.print(temp);Serial.print(", ");Serial.print(iTemperature);Serial.println(" *F");
      #endif  
      oled.print(" F   ");
      
      int iHumidity=sensorData.humidity +.5;
      if(hum=="--"){
        oled.print ("?");
      }
      oled.print(iHumidity);oled.println("%");
      #ifdef printMode
        Serial.print("hum, Humidity: ");Serial.print(hum);Serial.print(F(", "));Serial.print(iHumidity);Serial.println(F("%"));
      #endif  
    
      oled.print ("LIGHT  ");
      #ifdef printMode
         Serial.print("lux: ");Serial.println(sensorData.lux);
      #endif  

      float x=sensorData.lux;
      if (x>99) {x = 99;}    //limit to 99%
      dtostrf(x,1,0,buf1);
      if ( x<10){ 
        strcpy(buf0,"0");strcat(buf0,buf1);strcpy(buf1,buf0);
      }
      oled.print(buf1);
      oled.println("%");
    
       // oled.print("FLOOR ");
      if(sensorData.aH2o - h2oThreshhold > 0){  //display leak status in real time
        oled.print("WET    ");
        #ifdef printMode
          Serial.println(" FLOOR aH2o WET");
        #endif  
      }else{
        oled.print("DRY    ");
        #ifdef printMode
          Serial.println(" FLOOR aH2o DRY");
        #endif  
      }

      //oled.print("DOOR ");
      if(sensorData.door==0){
        oled.println("SHUT");
        #ifdef printMode
          Serial.println(" DOOR SHUT");
        #endif  
      }else{
        oled.println("OPEN");
        #ifdef printMode
          Serial.println(" DOOR OPEN");
        #endif  
      }
    #endif  
  #endif    //OLED_
}

//*********************************
//***ESP NOW One to One Callback function***************  
#ifdef ESPNOW_1to1  
  void ESPNOW_1to1_OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //callback verifies successful receipt after message was sent (see loop below for sendEspNow_1to1 function)
    #ifdef printMode
      Serial.println(F("*OnDataSent*"));
      Serial.print(F("\r\nLast Packet Send Status:\t"));
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif
    if(status==0){
      if(sleepSeconds==0){
      }else{
        setupInterruptConditions();  //interrupt if door state changes
        #ifdef printMode
          Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif  
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);
      }  
    }else{
      espNowAttempts++;
      if (espNowAttempts >= espNowAttemptsAllowed){ 
        setupInterruptConditions();  //interrupt if door state changes
        #ifdef printMode
          Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif  
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);        
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
    Serial.println(temp);
  #endif  
  turnOnBoardLED();  
  return temp;
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

//*******************************
void printSensorData(){
  #ifdef printMode  
    Serial.println(F("*printSensorData*"));
    Serial.print(sensorData.id);Serial.print(F(" id "));
    Serial.print(sensorData.temperature);Serial.print(F(" F "));
    Serial.print(sensorData.humidity);Serial.print(F(" % "));
    Serial.print(sensorData.pressure);Serial.print(F(" mb "));
    Serial.print(sensorData.lux);Serial.print(F(" lux "));
    Serial.print(sensorData.aH2o);Serial.print(F(" aH2o "));
    Serial.print(sensorData.dH2o);Serial.print(F(" dh2o "));
    Serial.print(sensorData.door);Serial.println(F(" door "));
  #endif  
}

//********************************
void printWakeupReason(){
  #ifdef printMode  //print_wakeup_reason()
    Serial.println(F("*printWakeupReason*"));
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason){
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println(F("Wakeup caused by external signal using RTC_IO")); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println(F("Wakeup caused by external signal using RTC_CNTL")); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println(F("Wakeup caused by timer")); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println(F("Wakeup caused by touchpad")); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println(F("Wakeup caused by ULP program")); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
  #endif 
}

//***********************************
void readAh2o(){
  //Read analog value and convert to % as whole number 0-99
  #ifdef printMode
    Serial.println(F("*readAh2o*"));
  #endif  
//  turnOnBoardLED(); 
  sensorData.aH2o = 2.44*(4095-analogRead(pinAh2o))/100;   //whole number 1-99 %
  #ifdef printMode
    Serial.print("aH2o: ");Serial.print(sensorData.aH2o);Serial.println(F("% "));
  #endif
//  turnOffBoardLED();
}

//***********************************
void readDh2o(){
  #ifdef printMode
    Serial.println(F("*readDh2o*"));
  #endif  
  //sensor reads 1 when dry, 0 when wet
  //we want 0=dry (normal status)
  sensorData.dH2o = digitalRead(pinDh2o);
  if (sensorData.dH2o >0){
    sensorData.dH2o = 0;
  }else{ 
    sensorData.dH2o = 1; 
  }
  #ifdef printMode
    Serial.print("dH2o: ");Serial.println(sensorData.dH2o);
  #endif  
}

//***********************************
void readDoor(){
  #ifdef printMode
    Serial.println(F("*readDoor*"));
  #endif
  //door = 1 when closed, 0 when open due to pullup resister
  //we want door = 0 when closed (normal state):
  sensorData.door = digitalRead(pinDoor);
  if (sensorData.door >0){
    sensorData.door = 0;
  }else{ 
    sensorData.door = 1; 
  }
  
  #ifdef printMode
    Serial.print("door: ");Serial.println(sensorData.door);
  #endif
}

//*********************************
void readHumidity() {  
  //valid data updates sensorData.humidity and hum; invalid updates only hum with  "--"
  #ifdef printMode
    Serial.println(F("*readHumidity*"));
  #endif  
  float x=0;
  #ifdef DHT_
    x = (dht.readHumidity());
  #endif
  #ifdef BME_
    x = (bme.readHumidity());
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
  #ifdef printMode
    Serial.println(F("*readPhotoCell*"));
  #endif  
  sensorData.lux = 2.44*((4095-analogRead(pinPhotoCell))/100);  //whole number 1-99 %
  #ifdef printMode
    Serial.print(F("lux: "));Serial.print(sensorData.lux); Serial.print(F("% "));
    Serial.println(analogRead(pinPhotoCell)); 
  #endif  
}

//***********************************
void readPressure(){   
  //valid data updates sensorData.pressure and pres; invalid updates only pres with  "--"
  #ifdef printMode
    Serial.println(F("*readPressure*"));
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
    Serial.println(F("*readTemperature*"));
  #endif 
  float x = 0; 
  #ifdef DHT_
    x=dht.readTemperature(true);  //true = *F
  #endif
  #ifdef BME_
    x=1.8*bme.readTemperature()+32.0;
  #endif 
  temp=String(x);
  if (temp=="nan"){temp = "--";
     }else{sensorData.temperature = x;
  }
  #ifdef printMode 
     Serial.print(F("temperature, temp: "));Serial.print(sensorData.temperature);Serial.print(F(", "));Serial.println(temp);
  #endif    
}

//***********************************
  void restartIfTime(){
  if (bootCount>=bootResetCount){
    #ifdef printMode
      Serial.println("");delay (5000);
      Serial.println(F("Rebooting.............*************.............")); 
    #endif
    ESP.restart();  //Reboot the esp32 to prevent memory issues
  }   
  bootCount++;
}  
//***********************************
void sendEspNow_1to1(){
  //format data and send to single ESPNOW peer if #define espnow_1to1 is enabled
  #ifdef ESPNOW_1to1
    #ifdef printMode
      Serial.println(F("*sendEspNow1to1*"));
    #endif    // 
    sensorData.id = sensorID;

    esp_err_t espResult = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData)); // Send message via ESP-NOW
    #ifdef printMode
      if (espResult == ESP_OK) { 
        Serial.println(F("Sent with success"));
      //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
      }else{
        Serial.println(F("Error sending the data"));  
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
      Serial.println(F("*sendEspNow1toN*"));
    #endif
   esp_err_t espResult = esp_now_send(0, (uint8_t *) &sensorData, sizeof(sensorData)); // Send message via ESP-NOW
    #ifdef printMode 
      if (espResult == ESP_OK) {
        Serial.println(F("Sent with success"));
      }
      else {
        Serial.println(F("Error sending the data"));
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
void setupESPNOW_1to1(){
  //initializes espnow, registers the peer's MAC address, and registers the peer
  #ifdef printMode
    Serial.println(F("*setupESPNOW_1to1*"));
  #endif
  #ifdef ESPNOW_1to1

    sensorData.id = sensorID;
    
    WiFi.mode(WIFI_STA);
  
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      #ifdef printMode
        Serial.println(F("Error initializing ESP-NOW"));
      #endif  
      return;
    }
  
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
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
void setupInterruptConditions(){
  //set door interupt based on current state open or closed
  #ifdef printMode
    Serial.println(F("*setupInterruptConditions*"));
    Serial.print(F("door = "));Serial.println(sensorData.door);
  #endif 
  if(sensorData.door==0){  //processed door = 1 when raw data dooor = 0
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0); //0 = High OPEN 1 = Low CLOSED
    //esp_sleep_enable_ext0_wakeup(pinDoor,1); 
  }else{
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,1);  

  } 
}      

//*********************************
void setupOledDisplay(){
  //setup OLED display on I2C bus
  #ifdef printMode
    Serial.println(F("*setupOledDisplay*"));
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
void setupPinModes(){                
  //Set Pin Modes INPUT / OUTPUT / PULLUP
  #ifdef printMode
    Serial.println(F("*setupPinModes*"));
  #endif   
  pinMode(pinBoardLED,OUTPUT);
  pinMode (pinDoor, INPUT);
  pinMode (pinPhotoCell, INPUT);
  #ifdef ESP32
    pinMode (pinAh2o, INPUT);
    pinMode (pinDh2o, INPUT_PULLUP);
  #endif
  #ifdef ESP8266
    pinMode (pinDh2o, INPUT);
  #endif  
}

//*********************************
void setupSensors(){
  //initializes sensors
  #ifdef printMode
    Serial.println(F("*setupSensors*"));
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
  #ifdef printMode
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
  #ifdef printMode
    Serial.println(F("*turnOffBoardLED*"));
  #endif  
  #ifdef ESP8266
    digitalWrite(pinBoardLED, HIGH);
  #endif  
  #ifdef ESP32
    digitalWrite(pinBoardLED, LOW);
  #endif 
}    
