
const char* VERSION = "AmbientAP 2022 v6.20";

/************************************************************************************
AmbientAP is a flexible, multi-featured sensor platform.  It can:
  1. expose sensor values via http GET command requests from a client.
  2. expose sensor values via lighter footprint ESPNOW protocol which pushes data to a peer/client periodically without receiving a request.
  3. operate on ESP32 or ESP8266 (including d1 mini) controller.
  4. accommodate BME280 (via I2C) or dht sensors (via pin 5).
  5. report to 1 ESPNOW peer or up to 4 peers.
  6. sleep and wake up via a timer, or it can stay awake.
  7. sleep immediately after sending a successfully received ESPNOW message when configured as a 1to1 peer.
  8. operate with or without an OLED display.
  9. operate as one of three such devices using this software by assigning unique SensorID 1, 2, or 3 to each sensor platform.

NOTES -  1. IF using MELife for ESP32 ESP-32S Development Board, use Arduino IDE "esp32 Dev Module" or "MH ET LIVE ESP32 Devkit"
         2. IF using d1 Mini 8266, use Arduino IDE "LOLIN(WEMOS) d1 R2 & Mini"

            For d1 Mini 8266, connect pins D0 and RST pins w/jumper 
            to wake up from sleep; disconnect jumper to load program

         3. Be sure to include all libraries shown in #include statements below for the selected processor in step 1 or 2
         
         4. Onboard LED states for debug/verification purposes when printMode is on:
            -->ON at Setup Start, off at setup end.
            -->ON at GET temperature
            -->OFF at GET humidity
            -->OFF at  sleep     

3/21 add ESPNOW option vs WIFI option stuff
6/9  add ME AmbientHUB MAC address
     add DHT sensors
     test 1toN - OK
     test 1to1 - OK
     send espNOW only until successful, then sleep when in 1to1 mode
6/12 cleanup, rearrange code for readability, enhance comments

*/

/********************************************************************************************
  * The code within the '#ifdef ESPNOW_1to1' and '#ifdef ESPNOW_1toN' blocks is adapted from the following:
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
********************************************************************************************/

/****************Compile-time Options*********************************************************/
#define printMode      //option: comment out if no printout to computer monitor
#define includeOled    //option: comment out if no oled display
                                                                                         
//Select a Processor:
#define ESP32           //Recommended choice
//#define ESP8266       //use for wemos D1 Mini.  NOTE - Use if only one AmbientAP sensor exists.
                        //Multiple D1 Minis seem to interfere with one another and have limited range

/***************Select only one of the 3 following transmission protocol #defines: *************/                    
//#define WIFI

#define ESPNOW_1to1   //send data to one peer: enter MAC address of receiver
      uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x2B, 0x6D, 0x00};  //esp32 #7 MAC Address AC:67:B2:2B:6D:00
      
//#define ESPNOW_1toN     //send to multiple peers: enter MAC addresses of receivers
      //comment these out and use your own AmbientHUB or other peer MAC addresses:
      uint8_t broadcastAddress1[] = {0xA8, 0x03, 0x2A, 0x74, 0xBE, 0x8C};  //LILLYGO MAC Address A8:03:2A:74:BE:8C
      uint8_t broadcastAddress2[] = {0x78, 0x21, 0x84, 0x7F, 0x82, 0x14};  //esp32 #11 Mac Address 78:21:84:7F:82:14
      uint8_t broadcastAddress3[] = {0x7C, 0x9E, 0xBD, 0xE3, 0xC5, 0xC8};  //esp32 #9 Mac Address 7C:9E:BD:E3:C5:C8
      //uint8_t broadcastAddress4[] = {0x8C, 0xAA, 0xB5, 0x85, 0x6A, 0x68};  //esp32 #12 Mac Address 8C:AA:B5:85:6A:68
      uint8_t broadcastAddress4[] = {0xAC, 0x67, 0xB2, 0x2B, 0x6D, 0x00};  //esp32 #7 MAC Address AC:67:B2:2B:6D:00

/********************ESPNOW misc***********************************************************/
int espNowAttempts=0;            //number of ESPNOW transmission attempts so far (do not adjust)
#define espNowAttemptsAllowed 3  //number of unsuccessful attempts allowed before sleeping if sleepSeconds > 0
int espNowDelay = 10000;         //delay in msec between espnow attempts

/****************Select a temp humidity (pressure) sensor: *********************************/
#define DHTTYPE DHT11
//#define DHTTYPE DHT21  //AM2301
//#define DHTTYPE DHT22  //AM2302
//#define BME280         //temp hum pressure

//**************PIN DEFINITIONS*****************
//note: esp32 set pin 15 low to prevent startup log on Status display - good to know for battery operation
#define pinBoardLED 2                 //onboard LED
#ifdef ESP32
  #ifdef DHTTYPE
    #define pinDHT 5                  //5, or use 27 not 4 for DHT temp hum sensors
  #endif
  #define pinSDA 21                   // ESP 12C Bus SDA for BME temp sensor, OLED, etc
  #define pinSCL 22                   // ESP 12C Bus SCL for BME temp sensor, OLED, etc
#endif
#ifdef ESP8266
  #ifdef DHTTYPE
    #define pinDHT 14                 //for DHT11,dht22 temp hum sensors
  #endif
  #define pinSDA 4                    // ESP 12C Bus SDA for BME temp sensor, OLED, etc
  #define pinSCL 5                    // ESP 12C Bus SCL for BME temp sensor, OLED, etc
#endif 
//****************Sleep Timer: NOTE: Wemos D1 Mini 8266 max sleep time is 71 minutes = 4260 seconds
const int sleepSeconds = 8*60;       //8*60 default sleep time in seconds; 0 if no sleep   
const long awakeSeconds = 2*60;      //2*60 default interval in seconds to stay awake as server; ignored if sleepSeconds = 0 
const long chillSeconds = 30;        //interval between readings if sleepSeconds = 0 slows serial monitor updates

//***************Each sensor board needs unique display title and ssid; uncomment one triplet and comment // the others:
uint8_t sensorID = 1;
#define displayTitle " ~AMBIENT 1~"  
const char* ssid = "AMBIENT_1";

//uint8_t sensorID = 2;    
//#define displayTitle " ~AMBIENT 2~"
//const char* ssid = "AMBIENT_2";

//uint8_t sensorID = 3;
//#define displayTitle " ~AMBIENT 3~"
//const char* ssid = "AMBIENT_3";

//************************End of Compile-time Options******************************************
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds

//*****************Sensor libraries***********************
#include <Wire.h>               //12C bus library for BME280 sensor or OLED display
#include <Adafruit_Sensor.h>    //sensor library

#ifdef DHTTYPE
  #include "DHT.h"
  DHT dht(pinDHT,DHTTYPE);
#endif

#ifdef BME280
  #include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
  Adafruit_BME280 bme;            // I2C
#endif

//*************SENSOR READINGS*******************
#ifdef ESP32
  RTC_DATA_ATTR float temperature1=81.1,humidity1=81.1,pressure = 1000.1; //initial values, then latest valid reading (if not nan or --)
#endif
#ifdef ESP8266  //note 8266 does not have RTC_DATA_ATR to preserve measurements when sleeping
  float temperature1=81.1,humidity1=81.1,pressure = 1000.0;               //initial values, then temperature from sensor if valid
#endif
String hum="--",temp="--",pres="--"; //adjusted values sent to hub: "--" if nan or current reading if valid

//*************OLED DISPLAY*******************
#ifdef includeOled
  #include <Wire.h>
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
  #define I2C_ADDRESS 0x3C            // 0X3C+SA0 - 0x3C or 0x3D
  #define OLED_RESET     -1           // Reset pin # (or -1 if sharing Arduino reset pin)
  SSD1306AsciiWire oled;
#endif

//***************WIFI SERVER STUFF******************
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

//****************ESP NOW One to One Setup*****************
#ifdef ESPNOW_1to1 
  #include <esp_now.h>
  #ifdef ESP32
    #include <WiFi.h>                  // wifi libr for esp32
  #endif
  #ifdef ESP8266
    #include <ESP8266WiFi.h>           // wifi libr for esp8266
    #include <ESPAsyncTCP.h>           //added for esp8266
  #endif                
 
  // Structure to send data must match the receiver structure:
  typedef struct struct_message {
      uint8_t id; // must be unique for each sender board
      float t;    //temperature
      float h;    //humidity
      float p;    //pressure
  } struct_message;
  struct_message sensorData;      // Create a struct_message called sensorData
  
  esp_now_peer_info_t peerInfo;   // Create peer interface
  
  // callback verifies successful receipt after message was sent (see loop below for sendEspNow_1to1 function)
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    #ifdef printMode
      Serial.println(F("*OnDataSent*"));
      Serial.print(F("\r\nLast Packet Send Status:\t"));
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif
    if(status==0){
      if(sleepSeconds==0){
      }else{
        #ifdef printMode
          Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif  
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);
      }  
    }else{
      espNowAttempts++;
      if (espNowAttempts >= espNowAttemptsAllowed){ 
        #ifdef printMode
          Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
        #endif  
        ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);        
      }
    }
  }
#endif

//****************ESP NOW One to Many Setup***************
#ifdef ESPNOW_1toN
  #include <esp_now.h>
  #ifdef ESP32
    #include <WiFi.h>                  // wifi libr for esp32
  #endif
  #ifdef ESP8266
    #include <ESP8266WiFi.h>           // wifi libr for esp8266
    #include <ESPAsyncTCP.h>           //added for esp8266
  #endif                  

  // Structure to send data must match the receiver structure:
  typedef struct test_struct {
    uint8_t id;
    float t;    //temterature
    float h;    //humidity
    float p;    //pressure
  } test_struct;
  test_struct sensorData;
  
  esp_now_peer_info_t peerInfo;
  
  // callback verifies successful receipt after message was sent (see loop below for sendEspNow_1toN function)
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
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

//****************Setup**********************
void setup(){
  //Execute one time setup, read and display sensor values, and setop wifi server OR transmit data using ESPNOW.
  delay(500);                   //ESP32 Bug workaround -- don't reference rtc ram too soon.
  setupPinModes();
  #ifdef printMode              //print the following to status monitor if printMode defined:
    Serial.begin(115200);
    turnOnBoardLED();           //turn on board LED during setup
    Serial.println(F(""));delay (5000);  //allow time for user to switch to status monitor after compilation
    Serial.print(F(VERSION));Serial.print(F(" - ")); Serial.println(F(displayTitle));
    Serial.println(F("----------------------------")); 
  #endif
  setupSensors();               //initialize sensors
  delay (2000);                 //was 2000 for bme280 or dht11; need 4000? for dht 22
  readTemperature();            //valid data updates temperature1 and temp; invalid updates only temp with  "--"
  readHumidity();               //valid data updates humidity1 and hum; invalid updates only hum with  "--"
  readPressure();               //valid data updates pressure1 and pres; invalid updates only pres with  "--"              
  setupOledDisplay();           //prepare the oled display if #define includeOled is enabled
  displayStatus();              //display latest valid sensor readings on oled display if #define incledeOled is enabled
  setupWifiServer();            //initialize wifi server if #define WIFI is enabled
  
  setupESPNOW_1to1();           //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                                //see OnDataSent procedure in section ESPNOW_1to1 above for callback processing
                                //successful data transfer will cause the system to sleep if sleepSeconds > 0
 
  setupESPNOW_1toN();           //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
                                //see OnDataSent procedure in section ESPNOW_1toN above for callback processing, which
                                //will cause system to go to sleep if sleepSeconds>0 and data was received successfully
  
  turnOffBoardLED();            //turn off the board LED when setup complete
}
//***************Loop****************** 
void loop(){ 
  //Execute repeatedly if system did not go to sleep during setup
  if (sleepSeconds == 0){  
                                //if unit does not sleep, read all sensors if chillSeconds has elapsed since last reading
    if(chillTimeIsUp(chillSeconds*1000)==1){ //slow down loop for serial monitor readability + sensor rest   
        readTemperature();      //valid data updates temperature1 and temp; invalid updates only temp with  "--"
        readHumidity();         //valid data updates humidity1 and hum; invalid updates only hum with  "--"
        readPressure();         //valid data updates pressure1 and pres; invalid updates only pres with  "--"                 
        displayStatus();        //display latest vald sensor readings on oled display if #define incledeOled is enabled
    }   
  }else{                  
                                //else if nap time, go to sleep
    if (sensorNapTime(awakeSeconds*1000) ==1 ){       //determine if awakeSeconds have elapsed
      #ifdef printMode          //issue sleep message if #define printMode is enabled
        Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(sleepSeconds);Serial.println(F(" seconds..zzzz"));
      #endif  
      ESP.deepSleep(sleepSeconds * uS_TO_S_FACTOR);   //go to sleep for sleepSeconds 
    }
  }
  sendEspNow_1to1();            //format data and send espnow msg to a single peer if #define ESPNOW_1to1 is enabled; 
                                //see OnDataSent procedure in section ESPNOW_1to1 above for callback processing
                                //successful data transfer will cause the system to sleep if sleepSeconds > 0                                

  sendEspNow_1toN();            //format data and send espnow msg to multiple peers if #define ESPNOW_1toN is enabled; 
                                //see OnDataSent procedure in section ESPNOW_1toN above for callback processing                            
}                               //repeat the loop

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
  //display sensor readings on oled display if #define incledeOled is enabled
  #ifdef printMode
    Serial.println(F("*displayStatus*"));
  #endif  
  #ifdef includeOled
    oled.clear(); 
    oled.setFont(fixed_bold10x15);
    oled.println(displayTitle);       //top line is display title
    oled.print("  ");oled.print(temperature1);oled.println(" F");
    oled.print("  ");oled.print(humidity1);oled.println(" %");
    oled.print("  ");oled.print(pressure);oled.println("mb"); 
  #endif
}

//***********************************
String getTemperature() {  
  //http GET retrieves prior reading
  #ifdef printMode
    Serial.println(F("*getTemperature*"));
    Serial.println(temp);
  #endif  
  turnOnBoardLED();  
  return temp;
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
//*********************************
String readHumidity() {  
  //valid data updates humidity1 and hum; invalid updates only hum with  "--"
  #ifdef printMode
    Serial.println(F("*readHumidity*"));
  #endif  
  #ifdef DHTTYPE
    float x = (dht.readHumidity());
  #endif
  #ifdef BME280
    float x = (bme.readHumidity());
  #endif
  hum=String(x);
  if (hum == "nan") {hum = "--";
   }else{humidity1 = x;
  }
  #ifdef printMode
    Serial.print(F("humidity1, hum: "));Serial.print(humidity1);Serial.print(F(", "));Serial.println(hum);
  #endif
  return hum;
}
//***********************************
String readPressure(){   
  //valid data updates pressure1 and pres; invalid updates only pres with  "--"
  #ifdef printMode
    Serial.println(F("*readPressure*"));
  #endif  
  pres="--"; //indicate no reading
  #ifdef BME280 
    float x = bme.readPressure()/100.0;    //millibars
    pres = String(x);
    if (pres == "nan") {pres = "--";
      }else{pressure = x;
    }
  #endif                                                                    
  #ifdef printMode
    Serial.print(F("pressure, pres: "));Serial.print(pressure);Serial.print(F(", "));Serial.println(pres);
  #endif  
  return pres;
}
//***********************************
String readTemperature() {  
  //valid data updates temperature1 and temp; invalid updates only temp with  "--"
  #ifdef printMode
    Serial.println(F("*readTemperature*"));
  #endif  
  #ifdef DHTTYPE
    float x=dht.readTemperature(true);  //true = *F
  #endif
  #ifdef BME280
    float x=1.8*bme.readTemperature()+32.0;
  #endif 
  temp=String(x);
  if (temp=="nan"){temp = "--";
     }else{temperature1=x;
  }
  #ifdef printMode 
     Serial.print(F("temperature1, temp: "));Serial.print(temperature1);Serial.print(F(", "));Serial.println(temp);
  #endif    
  return temp;
}
//***********************************
void sendEspNow_1to1(){
  //format data and send to single ESPNOW peer if #define espnow_1to1 is enabled
  #ifdef ESPNOW_1to1
    // Set values to send
    sensorData.id = sensorID;
    sensorData.t = temperature1;
    sensorData.h = humidity1;
    sensorData.p = pressure;
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
    sensorData.id = sensorID;
    sensorData.t = temperature1;
    sensorData.h = humidity1;
    sensorData.p = pressure;
    esp_err_t espResult = esp_now_send(0, (uint8_t *) &sensorData, sizeof(test_struct)); // Send message via ESP-NOW
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
  #ifdef printMode
    Serial.println(F("*sensorNapTime*"));
  #endif  
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
  #ifdef ESPNOW_1to1  
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
    esp_now_register_send_cb(OnDataSent);
    
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
  //initializes espnow, registers the peer MAC addresses, and registers four peers
  #ifdef ESPNOW_1toN
    WiFi.mode(WIFI_STA);
   
    if (esp_now_init() != ESP_OK) {
      #ifdef printMode
        Serial.println(F("Error initializing ESP-NOW"));
      #endif  
      return;
    }
    
    esp_now_register_send_cb(OnDataSent);
     
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
  #ifdef includeOled
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
  //Set Pin Modes INPUT / OUTPUT
  #ifdef printMode
    Serial.println(F("*setupPinModes*"));
  #endif   
  pinMode(pinBoardLED,OUTPUT);
}
//*********************************
void setupSensors(){
  //initializes sensors
  #ifdef printMode
    Serial.println(F("*setupSensors*"));
  #endif  
  #ifdef DHTTYPE
    dht.begin();
  #endif
  #ifdef BME280
    bool bme280=bme.begin(0x76);
    if (!bme280){
       #ifdef printMode
         Serial.println(F("Failed to initiate bme280"));
       #endif
    }
  #endif
}
//*************************************
void setupWifiServer(){
  //Initializes WIFI server for access to clients that issue GET commands to retrieve sensor data
  #ifdef WIFI
    #ifdef printMode
      Serial.print(F("sensorID: "));Serial.println(sensorID);
      Serial.print(F("SSID: "));Serial.println(ssid);
    #endif
    //WiFi.softAP(ssid, password);   
    WiFi.softAP(ssid);   // OMIT password for open AP 
    delay(100);
    IPAddress Ip(192,168,4,sensorID);
    IPAddress NMask(255,255,255,0);
    WiFi.softAPConfig(Ip,Ip,NMask);
    
    IPAddress IP = WiFi.softAPIP();
    #ifdef printMode
      Serial.print(F("AP IP address: "));Serial.println(IP);
    #endif 
  
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getTemperature().c_str());
    });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getHumidity().c_str());
    });
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getPressure().c_str());
    });
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
