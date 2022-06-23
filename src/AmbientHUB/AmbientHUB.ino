
const char* VERSION = "AmbientHUB 2022 v6.20";

/*********************************************************************************************************
* AmbientHUB collects onboard and external sensor data and passes it via cellular connection to IFTTT.com and Dweet.io.    
*
* NOTES - 1. IF using MELife ESP32 ESP-32S Development Board, use Arduino IDE "esp32 Dev Module" or "MH ET LIVE ESP32 Devkit"
*               (tested with Botletics arduino SIM7000A shield and and-global SIM7000A boards - select BOTLETICS hardware device below)
* 
*         2. IF using LillyGo SIM7000G, select ESP32 Wrover Module, remove on board SD card for software upload
*         
*         3. Be sure to include all libraries shown in #include statements below. 
*            <xxx> are searched in library directories, "xxx" are searched in sketch local directory
*         
*         4. ESP LED states for debug/verification purposes:
*            -->Blinks 5 seconds at reboot or first time startup
*            -->ON at setup WiFi with a sensor
*            --OFF at kill WiFi with a ssensor
*    
*********************************************************************************************************
*Upgrades from v2.14:
*Add and test LILLYGO board option - passes
*Test and-Global board - passes
*shorten messages ** to serial monitor
*
*fixed testMode in ESPNOW mode
*Implemented ESP NOW option vs Wifi
*Working on fona.getBattVoltage and software powerDown
*remove STRING references outside of #ifdef WIFI
*Can display/ send a single sensor (modified formatData & displayStatus)
*formatData - removed labels F, %, dbm, and spaces because the are displayed as 20 in dweet; separate sensors with colon
*
*Upgrades from 5.15:
*5/22: commented out espNOW incoming data
*6/9: Maine config
*6/14 implement sendATCmd to prevent requesting verbose responses over cellular
*     connectToCellularNetwork via sendATCmd works
*6/15 postHeartBeat works via sendATCmd both dweet and ifttt
*6/16 formatData() add data tag before data
*6/16  "  redo to group temps together, hums together, etc
*6/17 sendATCmd convert from string to char
*6/18 connectToC..., Post... , sendAT has one small  string left
*6/20 misc cleanup old cmts etc
*6/21 all STRINGS were converted to char[] arrays or F macro handling!  sendATCmd() verifies OK, HTTPACTION ,200, etc.
*6/23 add secrets file
*/


//****************Setup IFTTT webhook API Key*********************************
/*OPTIONS: 1. Disable the #include <secrets.h> statement below, enable the #define statement below that, and replace the YOUR_IFTTT_KEY_GOES_HERE phrase.
//   or    2. Create a secrets.h file with the following line and replace the YOUR_IFTTT_KEY_GOES_HERE phrase:    
#define SECRET_IFTTT_HB_URL "http://maker.ifttt.com/trigger/HB/with/key/YOUR_IFTTT_KEY_GOES_HERE"  
**/

#include <secrets.h>                  
//#define SECRET_IFTTT_HB_URL "http://maker.ifttt.com/trigger/HB/with/key/YOUR_IFTTT_KEY_GOES_HERE"  //heartbeat event HB

//****************Compile-time Options*********************************
char dataTag[] = "ME";    //unique prefix to identify source of data packet     
/**select one hardware device below and comment out the others: **/
/*Board selection impacts pin assignments, setup of pin modes, pwr on & off pulse duration, and onboard LED on/off states*/
#define BOTLETICS  
//#define LILLYGO  

//Select one:
#define ESPNOW
//#define WIFI

//***************Set up modes = 1 to enable, 0 to disable where 1 or 0 is assigned******************
#define testCellular                // comment out if you dont want to enable test transmission to dweet, ifttt at reboot
#define printMode                   // comment out if you dont want brief status display of sensor readings, threshholds and cellular interactions to pc
//#define testMode                    // uncomment for verbose test messages to status monitor
#define ATMode                      // uncomment to use AT commands to reduce cellular return traffic instead of higher level FONA commands 
const int espSleepSeconds =  48*60; //or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
const long espAwakeSeconds = 12*60; // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
int heartbeatMinutes = 10;          // heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less thab espAwakeSeconds 

#define cellularMode 1              // 1 enables a cellular connection; 0 no connectiod (good for debugging code without tripping cellular comms)
#define alarmMode 0                 // 1 enables alarm ifttt when a measurement crosses threshhold; 0 no alarm
#define iFTTTMode 1                 // 1 enables use of ifttt; 0 no IFTTT
#define heartbeatMode 1             // 1 turns on periodic heartbeat reporting of all readings; 0 no heartbeat
RTC_DATA_ATTR int heartbeatToggle=1; //used by postHeartbeat to alternate heartbeat to ifttt.com (when value=1) and dweet.io (when value=0)

const long serialMonitorSeconds = 5*60;      // interval in seconds to refresh serial monitor sensor readings (enhance readability)
          
#define simSleepMode 1              // 1 = shut off sim shield between readings 
const long samplingRateSeconds = 10; //1*60; // interval in seconds between sensor readings in alert mode

/**************************IFTTT sensors and threshholds****************************///Condo and CP enents
#define tempHighLimit 90            // upper limit which will cause IFTTT alert
#define tempLowLimit 45             // lower limit which will cause IFTTT alert
#define tempIncrement 5             // Increment by which temp threshholds are moved to prevent multiple alarms caused by temp fluctuations around the threshhold

#define numberOfSensors 4           // number of sensor platforms available as Access Points + 1 for HUB
                                    //example: if hub has sensors, and there are 3 wireless sensors, numberOfSensors = 4
                                    //recommend leave at 4 independent of configuration   

int tempHighThreshReset[numberOfSensors] ,tempLowThreshReset [numberOfSensors];
RTC_DATA_ATTR int tempHighThresh[]={tempHighLimit,tempHighLimit,tempHighLimit,tempHighLimit};
RTC_DATA_ATTR int tempLowThresh []={tempLowLimit,tempLowLimit,tempLowLimit,tempLowLimit};

//arrays to indicate types of sensors aboard each sensor platform (1=presnt, 0 = absent)
int temps[] = {1,1,1,1}, dhtTemps[] = {1,1,1,1}, dhtHums[]={1,1,1,1}, dbms[]={1,0,0,0}, pres[]={1,1,1,1}, bat[]={1,0,0,0};

//Arrays to indicate which sensors can be used to generate IFTTT alert
int iftTemps[] = {0,0,0,0}, iftDhtTemps[] = {1,1,0,0}, iftDhtHums[]={0,0,0,0};

//arrays to hold ambient data initialized with test data
RTC_DATA_ATTR float temperature[] = {80.0,81.1,82.2,83.3};
RTC_DATA_ATTR float dhtTemperature[] =  {80.0,81.1,82.2,83.3};
RTC_DATA_ATTR float dhtHumidity[] = {0.0,10.0,20.0,30.0};    
RTC_DATA_ATTR float pressure[] = {0.0,0.0,0.0,0.0};        
RTC_DATA_ATTR int dbm[] = {10,0,0,0};
RTC_DATA_ATTR uint16_t vbat[] = {0,0,0,0};
char buf[50],buf2[6]; 
uint8_t sensorStatus[numberOfSensors];  //wifi success 1 or 0 to display : or ?? on oled header in displayStatus() or to stop at 1 reading if limited awake time
uint8_t alert[numberOfSensors];         // used to accumulate # sensors that have alert condition
const char* sensors[]={"local","wifi","wifi","wifi"};

//****************************************************************
#ifdef WIFI
  #include <WiFi.h>
  #include <HTTPClient.h>
  const char* wifiSSID[] = {"n/a","AMBIENT_1","AMBIENT_2","AMBIENT_3"};
  const char* wifiPassword[] = {"","","",""};
  
  //don't need all this, but it is convenient for debug purposes:
  const char* serverNameTemperature1 = "http://192.168.4.1/temperature";
  const char* serverNameHumidity1 = "http://192.168.4.1/humidity";
  const char* serverNamePressure1 = "http://192.168.4.1/pressure";
  //const char* serverNameTemperature2 = "http://192.168.4.2/temperature";
  //const char* serverNameHumidity2 ="http://192.168.4.2/humidity";
  //const char* serverNamePressure2 = "http://192.168.4.2/pressure";
  //const char* serverNameTemperature3 = "http://192.168.4.3/temperature";
  //const char* serverNameHumidity3 = "http://192.168.4.3/humidity";
  //const char* serverNamePressure3 = "http://192.168.4.3/pressure";
#endif  
//*****************************Miscellaneous*********************
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds  
RTC_DATA_ATTR int bootCount = 0;    //Incremented after first setup so delay only happen on initial startup gives time for user to open Status monitor 

uint8_t type;
char imei[16] = {0};             // 16 character buffer for IMEI
char URL[255];                   // buffer for request URL
char body[255];                  // buffer for POST 
//***************timer stuff*********************************************/
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h" 
}
/************************* Botletics SIM7000A Arduino Shield Connections to ESP32******/
#ifdef BOTLETICS
  #define UART_BAUD   9600
  #define pinBoardLED 2               // ESP32 on board led
  #define pinFONA_RST 5               // BOTLETICS shield D7
  #define pinButton 4                 // Spare
  #define pinFONA_TX 16               // ESP32 hardware serial RX2 to shield 10 TX
  #define pinFONA_RX 17               // ESP32 hardware serial TX2 to shield 11 RX
  #define pinFONA_PWRKEY 18           // BOTLETICS shield 6
  #define pinSDA 21                   // ESP 12C Bus SDA for BME temp sensor, OLED, etc
  #define pinSCL 22                   // ESP 12C Bus SCL for BME temp sensor, OLED, etc
#endif
/************************* Lilllygo SIM7000G Connections to ESP32******/
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
/******************************SIM7000 Misc*****************************************
/******* ORIGINAL ADAFRUIT FONA LIBRARY TEXT *******
  This is an example for our Adafruit FONA Cellular Module
  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542
  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_FONA.h"          //from https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
//#include <Adafruit_FONA.h>
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#include <HardwareSerial.h>
HardwareSerial fonaSS(1);
#define SIMCOM_7000                // SIM7000A/C/E/G

//**********************************Temperature sensors***********************
#include <Wire.h>               // 12C
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
Adafruit_BME280 bme;            // temp hum pressure sensor

#include "Adafruit_MCP9808.h"   // shield temp sensor (on 12C bus)
Adafruit_MCP9808 tempsensor = 
         Adafruit_MCP9808();    // Create the MCP9808 temperature sensor object
         
//********text only OLED***********/
#include "SSD1306Ascii.h"     // low overhead library text only
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C      // 0x3C+SA0 - 0x3C or 0x3D
#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
SSD1306AsciiWire oled;
  
/*****************WIFI, ESP_NOW************************/
#ifdef ESPNOW
  /*********
  * The code within this '#ifdef ESPNOW' block was adapted from the following:
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
  *********/

  #include <esp_now.h>
  #include <WiFi.h>

  typedef struct struct_message {
      uint8_t id; // must be unique for each sender board
      float t;
      float h;
      float p;
  } struct_message;
  
  // Create a struct_message called sensorData
  struct_message sensorData;

  // Create a structure to hold the readings from each board
  struct_message board1;
  struct_message board2;
  struct_message board3;

  // Create an array with all the structures
  struct_message boardsStruct[3] = {board1, board2, board3};

  // callback function that will be executed when data is received
  void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
    char macStr[18];
    #ifdef printMode
      Serial.print("Packet received from: ");
      snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      Serial.println(macStr);
    #endif  
    memcpy(&sensorData, incomingData, sizeof(sensorData));
    #ifdef printMode
      Serial.printf("Board ID %u: %u bytes\n", sensorData.id, len);
    #endif
    
    // Update the structures with the new incoming data
    boardsStruct[sensorData.id-1].id = sensorData.id;
    boardsStruct[sensorData.id-1].t = sensorData.t;
    boardsStruct[sensorData.id-1].h = sensorData.h;
    boardsStruct[sensorData.id-1].p = sensorData.p;
    temperature[sensorData.id] = sensorData.t;
    dhtTemperature[sensorData.id] = sensorData.t;
    dhtHumidity[sensorData.id]=sensorData.h;
    pressure[sensorData.id]=sensorData.p;
    #ifdef printMode    
      Serial.println("id         t      h      p");
      Serial.print(boardsStruct[sensorData.id-1].id);Serial.print("   ");
      Serial.print(dhtTemperature[sensorData.id]);Serial.print("   ");
      Serial.print(dhtHumidity[sensorData.id]);Serial.print("   ");
      Serial.println(pressure[sensorData.id]);  
    #endif    
  }
#endif

void setup() {/************************SETUP() ************************************/
   delay(500);                  //ESP32 Bug workaround -- don't reference rtc ram too soon.
   setupPinModes();             // Set Pin Modes INPUT / OUTPUT (necessary to turn board LED on, etc.) 
   if (bootCount ==0) {blinkBoardLED(5);}    // blink LED for 5 sec for feedback and to give user time to activate serial console
   initializeArrays();          // Initialize sensor threshholds
   setupOledDisplay(); 
   for (uint8_t i=0; i< numberOfSensors; i++){  // for each sensor:
      displayStatus(i);         // update OLED display       
   }
   #ifdef printMode
      Serial.begin(115200);          // Initialize Serial Monitor 
      Serial.print(F("***** ")); Serial.print(F(VERSION));Serial.println(F(" *****starts after 5 sec ***"));
      Serial.print(F("bootCount= "));Serial.println(bootCount);
   #endif
   wakeUpSensors();             // Wake up the HUB temp sensors 
   if (bootCount ==0 && cellularMode == 1){ //if new reboot:
     powerUpSimModule();          // Power up the SIM7000 module by goosing the PWR pin 
     delay (5000);
     simModuleOffIfSimSleepMode();// Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
     #ifdef testCellular
       if (cellularMode==1 && iFTTTMode==1){
          activateCellular(); 
          readSignalStrength(0);
          //postToIfTTT();            // upload test data to ifttt.com if enabled
          postHeartbeatData();       // upload test data to ifttt.com if enabled
          postHeartbeatData();       // upload test data to dweet.io if enabled
          displayStatus(0);          // update OLED display to show ss
          simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
       } 
     #endif  
   }
   delay (5000);
   bootCount++;

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
      esp_now_register_recv_cb(OnDataRecv);
  #endif
}
//*************************************Loop()*********************************************
void loop() {    
//0. Go to sleep if snooze mode nap time
  if (espSleepSeconds >0){  
    if (espAwakeSeconds >0 && espAwakeTimeIsUp(espAwakeSeconds*1000) ==1 ){
      #ifdef printMode
        Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
      #endif
      ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);
    }
  }  
//1. read local and remote sensors 
  if (sensorTimeIsUp(samplingRateSeconds*1000) ==1 ){
    for (uint8_t i=0; i< numberOfSensors; i++){  // for each sensor:
      if (sensorStatus[i]==0){    // if sensor has not been sampled yet:    
        #ifdef WIFI   
        //killWifi();               //prepare to connect via wifi with server
        //if(setupWifi(i)==0){      // connect to local ESP32(i) wifi server if not local
        #endif
          sensorStatus[i]= readSensorData(i); //1;     // wifi success to display ":" on oled header in displayStatus()
          if(sensorStatus[i]==1){
            printSensorData();
            alert[i]= processIfTTT(i);    // compare sensors to threshholds and set alert status
          }  
        #ifdef WIFI  
          //killWifi();
        //}
        #endif
      }  
      displayStatus(i);         // update OLED display   
    }
    delay(5000);                // allow latest display time before toggle timer takes over
    int alerts = 0;             // determine whether we have one or more alerts:    
    for (int i=0;i<numberOfSensors;i++){  //determine if an alarm threshhold was crossed
      alerts=alerts+alert[i];
    }
    if (alerts>0){              // if we have alerts, activate cellular network and report via ifttt if alarmMode set
      if (cellularMode==1 && iFTTTMode==1 && alarmMode==1){
        activateCellular(); 
        readSignalStrength(0);
        postToIfTTT();
        displayStatus(0);             // update OLED display to show signal strength
        simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
      }
    }
  }

  
//2. heartbeat: upload latest data if its time
  if (heartbeatTimeIsUp(heartbeatMinutes*60000L) ==1 && cellularMode==1 ){
    activateCellular();
    readSignalStrength(0);
    postHeartbeatData();          // upload sensor data to dweet.io or IFTTT if heartbeatMode enabled
    displayStatus(0);             // update OLED display here to update signal strength
    simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
  }
  
//3. display data on serial monitor JSON format for diagnostic purposes
  if (serialTimeIsUp(serialMonitorSeconds*1000) ==1 ){
    #ifdef printMode
      formatData();    
      sprintf(body, "{\"value1\":\"%s\"}", buf);
      Serial.println();
    #endif  
  }
}
//*********************************
int activateCellular(){
  #ifdef printMode
    Serial.println(F("*activateCellular*"));
  #endif
  if (cellularMode==1) {
    for (int i = 0;i<2;i++){
      powerUpSimModule();          // Power on the SIM7000 module by goosing the PWR pin 
      delay (1000); //9/19/2000 
      if(setupSimModule()==0){            // Establish serial communication and fetch IMEI
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
            return (0);
          }else { 
            //Serial.println(F("bad sig str - trying again to activate Cellular****"));
          }
        }
      }
    }
  }  
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
    #ifdef printMode
        Serial.print(F("."));
    #endif
    delay(2000); // Retry every 2s
    i ++;
    if (i>3){
      #ifdef printMode
          Serial.println(F("Failed to enable GPRS"));
      #endif
      return -1;  
    }
  }

  #ifdef printMode
      Serial.println(F("Enabled GPRS!"));
  #endif
  return 0;
}
//*********************************
void activateModem(){              // Set modem to full functionality
  #ifdef printMode
    Serial.println(F("*activateModem*"));
    Serial.println(F("fona.setFuntionality(1)")); 
    Serial.println(F("fona.setNetworkSettings('hologram')"));
    Serial.println(F("fona.setPreferredMode(38)"));
    Serial.println(F("fona.setPreferredLTEMode(1)"));
//    Serial.println(F("setOperatingBand('CAT-M', 12)"));
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
    1 CAT-M
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
int connectToCellularNetwork() {  //220618
  // Connect to cell network and verify connection every 2s until a connection is made
  #ifdef printMode
    Serial.println(F("*connectToCellularNetwork*"));
  #endif
  #ifndef ATMode
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
    
  #else
  
    int err=sendATCmd("AT+CGREG?","","");
    #ifdef printMode 
      if (err==0){              
        Serial.println(F("Connected!"));
      }else{
        Serial.println(F("Failed!"));
      }    
    #endif
    if (err==0){
      return 0;
    }
    return -1; 
  
  #endif
}
//*********************************
void displayStatus(uint8_t i){
  #ifdef testMode 
    Serial.print(F("*displayStatus "));Serial.print(i);Serial.println(F("*"));
  #endif  

  if (numberOfSensors>=2){
    oled.setFont(Arial_14);
    oled.setCursor(0,16*i);
    oled.print("T");
    oled.print(i);
    if (sensorStatus[i] ==0){
      oled.print("? ");  //indicates old or no measurement
    }else{  
      oled.print(": ");  //indicates current measurement
    }
  }else{
    oled.setFont(Arial_bold_14);
    oled.setCursor(0,0);
  }
  int value = dhtTemperature[i]+.5;
  oled.print(value);
  if (value < 100){
    oled.print("F");
  }
  oled.print(" ");  
  value = dhtHumidity[i]+.5;
  oled.print(value);
  oled.print("% ");

  if (dbm[i]==0){
    oled.println("");
  }else{
    oled.print(dbm[i]); 
    oled.println(" dbm");
  }  
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
//*********************************
void formatData(){                   //transfer comma separated sensor data to buf
  #ifdef testMode 
    Serial.println(F("*formatData*"));
  #endif

  //enter data TAG and separator
  strcpy (buf,dataTag);strcat(buf,"_");

  //process temps
  for(int i=0;i<numberOfSensors;i++){
    if(dhtTemps[i]==1){
      dtostrf(dhtTemperature[i], 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfSensors-1)){
        strcat(buf,"F_");
      }else{
          strcat(buf,":");   //comma separator causes error in http get in sendAT as written 
      }
    }
  }

  //process humidity
  for(int i=0;i<numberOfSensors;i++){
    if(dhtHums[i]==1){
      dtostrf(dhtHumidity[i], 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfSensors-1)){
        strcat(buf,"%_");
      }else{
        strcat(buf,":"); 
      }
    }
  }

  //process signal strength always last
  for(int i=0;i<numberOfSensors-1;i++){
    if(dbms[i]==1){
      dtostrf(dbm[i], 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfSensors-1)){
        strcat(buf,"db");
      }else{
        //since dbm is last, we don't need a separator
      }
    }
  }
  #ifdef printMode
    Serial.print(F("buf: "));Serial.println(buf);
  #endif
}
//***********************************
static int heartbeatTimeIsUp(long msec){    //Read data at timed intervals  
//  #ifdef testMode 
//    Serial.println(F("*heartbeatTimeIsUp*"));
//  #endif
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
void httpGETRequest(const char* serverName) {
  #ifdef testMode 
    Serial.print(F("*httpGETRequest*"));Serial.print(serverName);
  #endif  
#ifdef WIFI  
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--" ;

  
  if (httpResponseCode>0) {
    #ifdef testMode 
      Serial.print(F(" HTTP Response Code: "));
      Serial.print(httpResponseCode);
    #endif
    payload = http.getString();
    //x.toCharArray(payload,6);
    
  }
  else {
    #ifdef testMode 
      Serial.print(F(" Error code: "));
      Serial.print(httpResponseCode);
    #endif
  }
  #ifdef testMode 
    Serial.print(F(" Payload: "));Serial.println(payload);
  #endif  
  
  http.end();  //release resources
 // return payload;
#endif 
}

//************************************
void initializeArrays(){
  #ifdef testMode 
    Serial.println(F("*initializeArrays*"));
  #endif  
  for (int i=0; i<numberOfSensors; i++){ 
    tempHighThreshReset[i] = tempHighThresh[i];  //do it this way to preserve sleep vars
    tempLowThreshReset[i] = tempLowThresh[i];
    sensorStatus[i]=0;  //This causes "?" to display on OLED, indicating no real data
  }
}
//****************************************
void killWifi(){
    WiFi.disconnect();
    turnOffBoardLED();
    //if (testMode==1) {Serial.println("*WiFi killed*");}
}   

//*********************************
void postHeartbeatData(){       //upload sensor data to dweet.io or IFTTT

  #ifdef printMode 
    Serial.println(F("*postHeartbeatData*"));
  #endif
  if (heartbeatMode==1)  {   // GET request use the IMEI as device ID
    formatData();            // transfer sensor data to buf
    int i = 0;               // Count the number of failed attempts 
    if(heartbeatToggle==0){  //if 0, dweet:
      //sprintf(URL, "http://dweet.io/dweet/for/%s?T=%s",imei,buf);  //option provides T= prefix to data
      sprintf(URL, "http://dweet.io/dweet/for/%s?%s",imei,buf);

      #ifndef ATMode
        #ifdef printMode
          Serial.println(F("fona.HTTP_GET_start"));
          Serial.print(F("URL: ")); Serial.println(URL);
        #endif
        uint16_t statusCode;
        int16_t length;
        while (i < 3 && !fona.HTTP_GET_start(URL,&statusCode, (uint16_t *)&length)) {
          #ifdef printMode
            Serial.print(F("."));
          #endif
          Serial.print(F("Dweet failure #")); Serial.println(i);
          delay(5000);
          i++; 
        }
        #ifdef printMode
          Serial.print("statusCode, length: ");Serial.print (statusCode);Serial.print(", ");Serial.println(length);
        #endif
        fona.HTTP_GET_end();  //causes verbose response AT+HTTPREAD (and? AT+HTTPTERM to terminate HTTP?)
      #else
        int err = sendATCmd("AT+HTTPTERM","","");
        err = sendATCmd("AT+HTTPINIT","","");
        err = sendATCmd("AT+HTTPPARA","=CID","1");
        err = sendATCmd("AT+HTTPPARA","=URL",URL);
        delay(2000);
        err = sendATCmd("AT+HTTPACTION","=0","");
        err = sendATCmd("AT+HTTPTERM","",""); 
      #endif
      heartbeatToggle=1;     
    }else {               // if 1, ifttt:
      if (iFTTTMode==1){
        sprintf(body, "{\"value1\":\"%s\"}", buf);
        #ifdef printMode
          Serial.print(F("body: ")); Serial.println(body);
        #endif 
        #ifndef ATMode         
          while (i < 3 && !fona.postData("POST",SECRET_IFTTT_HB_URL,body)) {
            #ifdef printMode 
              Serial.println(F("."));
            #endif
            Serial.print("IFTTT failure #"); Serial.println(i);
            delay(5000);
            i++;
          }  
      #else
        int err = sendATCmd("AT+HTTPTERM","","");
        err = sendATCmd("AT+HTTPINIT","","");
        err = sendATCmd("AT+HTTPPARA","=CID","1");
        sprintf(URL, "%s?value1=%s",SECRET_IFTTT_HB_URL,buf);
        err = sendATCmd("AT+HTTPPARA","=URL",URL);
        delay(200);
        err = sendATCmd("AT+HTTPACTION","=0","");
        err = sendATCmd("AT+HTTPTERM","","");    
      #endif  
      }
      heartbeatToggle=0; 
    }   
    #ifdef printMode
      Serial.print(F("failed attempts = "));Serial.println(i);
    #endif  
  } 
}
//************************************
void postToIfTTT(){                //Post alert to IFTTT if iFTTTMode is enabled
  #ifdef printMode 
     Serial.println(F("*postToIfttt*"));
  #endif
  if (iFTTTMode ==1) {  
    formatData();                  //transfer ssensor data to buf
    sprintf(body, "{\"value1\":\"%s\"}", buf);

    #ifndef ATMode    
      #ifdef printMode
        Serial.println(F("body: ")); Serial.println(body);
      #endif  
      int attempts=0;                   //try 3 times to POST the URL and JSON body to IFTTT:
      #ifdef printMode
        Serial.println(F("fona.postdata('POST',SECRET_IFTTT_HB_URL,body)"));
      #endif
      while (attempts < 3 && !fona.postData("POST",SECRET_IFTTT_HB_URL,body)) {      Serial.print(F("."));  //original line
        attempts++;
        delay(5000);
      }
      #ifdef printMode
        Serial.print(F("failed attempts = "));Serial.println(attempts);
      #endif
      if (attempts > 2){  //reset threshholds if failed to post
        for (int i=0; i<numberOfSensors;i++){
          tempHighThresh[i] = tempHighThreshReset[i];
          tempLowThresh[i] = tempLowThreshReset[i];
          #ifdef printMode
            Serial.print(F("reseting temp threshholds to "));
            Serial.print(tempLowThresh[i]);
            Serial.print(F(" and ")) ;
            Serial.println(tempHighThresh[i]);
          #endif
        }  
      } 
    #else
      int err = sendATCmd("AT+HTTPTERM","","");
      err = sendATCmd("AT+HTTPINIT","","");
      err = sendATCmd("AT+HTTPPARA","=CID","1");
      sprintf(URL, "%s?value1=%s",SECRET_IFTTT_HB_URL,buf);
      err = sendATCmd("AT+HTTPPARA","=URL",URL);
      delay(200);
      err = sendATCmd("AT+HTTPACTION","=0","");
      err = sendATCmd("AT+HTTPTERM","","");         
    #endif  
    
  }           // endif IFtttMode = 1
}

//*********************************
void powerUpSimModule() {                // Power on the module
  #ifdef printMode
    Serial.println(F("*powerUpSimModule*"));
  #endif
//  digitalWrite(pinFONA_PWRKEY, LOW);   // turn it off??
//  delay(100);                          // 100 msec pulse for SIM7000
//  digitalWrite(pinFONA_PWRKEY, HIGH);

//  pinMode(pinFONA_PWRKEY, OUTPUT);
//  digitalWrite(pinFONA_PWRKEY, LOW);
//  delay(1000);                           // 1 sec
//  digitalWrite(pinFONA_PWRKEY, HIGH);
//  delay(10000);                         // give it time to take hold - long delay
//}

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
  for(int i=0;i<numberOfSensors;i++){
    //if (testMode==1) {Serial.print("printSensorData ");Serial.print(i); Serial.println("***");}
    #ifdef printMode
      Serial.print(F("Temp: ")); Serial.print(temperature[i]); 
      Serial.print(F(" dhtTemp: ")); Serial.print(dhtTemperature[i]);
      Serial.print(F(" Hum: ")); Serial.print(dhtHumidity[i]);
      Serial.print(F(" Pres: ")); Serial.print(pressure[i]);
      Serial.print(F(" dbm: ")); Serial.print(dbm[i]);
      Serial.print(F(" vBat: ")); Serial.println(vbat[i]);
    #endif
  }
}
//*********************************
int processIfTTT(uint8_t i) {              // set flag to post alert to iFTTT if temp outside limit
                                       // and adjust temp threshholds 
    #ifdef printMode
      Serial.print(F("*processIfTTT*")); Serial.print (i); Serial.print(F(": "));
      Serial.print(" Thresh: "); Serial.print(tempLowThresh[i]);
      Serial.print(" - "); Serial.println(tempHighThresh[i]);
    #endif

    tempHighThreshReset[i] = tempHighThresh[i];
    tempLowThreshReset[i] = tempLowThresh[i];
    int flag=0;

    if (iftDhtTemps[i]==1){
    if (dhtTemperature[i] - tempHighThresh[i] > 0) {
      #ifdef testMode
        Serial.print(F(" ;1-Temp exceeds ")); Serial.print (tempHighThresh[i]);
      #endif    
      tempHighThresh[i] = tempHighThresh[i] + tempIncrement;
      flag=1; //postToIfTTT();
      goto bye;    
    }  
    if (dhtTemperature[i] - (tempLowThresh[i] )<0) {      
      #ifdef testMode
        Serial.print(F(" ;2-Temp is less than ")); Serial.print(tempLowThresh[i]);
      #endif
      tempLowThresh[i] = tempLowThresh[i] - tempIncrement;  
      flag=1; //postToIfTTT(); 
      goto bye;    
    }
    if (dhtTemperature[i] - (tempHighThresh[i] -1.5*tempIncrement)<0) {  //not sure about this
      if(tempHighThresh[i]-tempHighLimit>0){
        #ifdef testMode
          Serial.print(F(" ;3-Temp is less than ")); Serial.print(tempHighThresh[i]);  
        #endif  
        tempHighThresh[i] = tempHighThresh[i] - tempIncrement;  
        flag=1;  //postToIfTTT(); 
        goto bye;    
      }
    }
    if (dhtTemperature[i] - (tempLowThresh[i] + 1.5*tempIncrement)>0) {
      if(tempLowThresh[i]-tempLowLimit<0){
        #ifdef testMode
          Serial.print(F(" ;4-Temp exceeds ")); Serial.print (tempLowThresh[i]);  
        #endif  
        tempLowThresh[i] = tempLowThresh[i] + tempIncrement;  
        flag=1;  //postToIfTTT(); 
        goto bye;    
      }
    }
    }
bye:   
  #ifdef testMode
    Serial.println(F(" "));
  #endif  

  return(flag);
}
//***********************************
void readBatteryVoltage (uint8_t i){
    Serial.println("*ReadBatteryVoltage*");
    uint16_t vBATT = 2500;
//if (! fona.getBattVoltage(&vBATT)) {
//  Serial.println(F("Failed to read Batt"));
//}
  //   Serial.println(vBATT);
    vbat[i]=vBATT;
  /*
          uint16_t vbatt;
        if (! fona.getBattVoltage(&vbatt)) {
          Serial.println(F("Failed to read Batt"));
          Serial.print(F("VBat = ")); Serial.print(vbatt); Serial.println(F(" mV"));
        } else {
          Serial.print(F("VBat = ")); Serial.print(vbatt); Serial.println(F(" mV"));
        }
    */    
}
//**********************************        
bool readNetStatus() {
  #ifdef testMode
    Serial.println(F("readNetStatus*"));*
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
//*********************************
uint8_t readSensorData(uint8_t i){             // temp & voltage
  #ifdef testMode
    Serial.print(F("*readSensorData "));Serial.print(i);Serial.println(F("*"));
  #endif      
  uint8_t returnFlag = 1;  //return success
  if (sensors[i]=="local"){
    readShieldTemp(i);
    readBatteryVoltage(i);

        float v = 1.8*bme.readTemperature()+32.0;
    
       if (isnan(v)){returnFlag=0; v=dhtTemperature[i];}
       dhtTemperature[i]=v;
 
       v = (bme.readHumidity());
   
       if (isnan(v)){v=dhtHumidity[i];}
       dhtHumidity[i] = v;
   
       v = bme.readPressure()/100;
       if (isnan(v)){v=pressure[i];}
       pressure[i] = v;
   
      // if (testMode==1) {Serial.print("local temp: ");Serial.print(dhtTemperature[i]);
                         //Serial.print(" local hum: ");Serial.print(dhtHumidity[i]);Serial.print(" ");}
                         //Serial.print(" local pres: ");Serial.println(pressure[i]);

     returnFlag=1;
  }else if (sensors[i]=="wifi"){ 
    #ifdef WIFI
      // Check WiFi connection status
      #ifdef testMode
          Serial.print(F("WL_CONNECTED = "));Serial.println(WL_CONNECTED);
          Serial.print(F("WiFi.status() = "));Serial.println(WiFi.status());
      #endif
  
      if(WiFi.status()!=WL_CONNECTED ){ setupWifi(i);}
      if(WiFi.status()== WL_CONNECTED ){ 
        String payload = "";
//      switch (i){
//        case 1:
          //payload = "";
          //httpGETRequest(serverNameTemperature1);
          if (payload!="--"){    //retain prior reading if no data so we dont trigger ifttt
            dhtTemperature[i] = payload.toFloat(); 
            temperature[i] = payload.toFloat();
          }
          //payload = "";
          //httpGETRequest(serverNameHumidity1);
          if (payload!="--"){   //retain prior reading if no data so we dont trigger ifttt
            dhtHumidity[i] = payload.toFloat();
          }
          //payload = "";
          //httpGETRequest(serverNamePressure1);
          if (payload!="--"){   //retain prior reading if no data so we dont trigger ifttt
            pressure[i] = payload.toFloat();
          }
     
    }else{returnFlag=0;
  }
  #endif
  #ifdef ESPNOW
    #ifdef testMode
      Serial.print("boardsStruct[sensorData.id-1].id :");Serial.println(boardsStruct[sensorData.id-1].id);
    #endif
    if (boardsStruct[sensorData.id-1].id == i){
      returnFlag = 1;
    }else{
      returnFlag = 0;  
    }
  #endif
  }
  //Serial.print("returnFlag: ");Serial.println(returnFlag);
  return(returnFlag);
}

//*********************************
void readSensorShieldData(int i){            
  #ifdef testMode
     Serial.println(F("*readSensorShieldData*"));
  #endif
  readShieldTemp(i);
  readSignalStrength(i);
}
//*********************************  
void readShieldTemp(int i){
  #ifdef testMode
     Serial.println(F("*readShieldTemp*"));
  #endif
     
  // Measure temperature
  tempsensor.wake();               // Wake up the MCP9808 if it was sleeping
  float tempC = tempsensor.readTempC();  //read temperature in Celsius units
  float tempF = tempC * 9.0 / 5.0 + 32;  //calculate Fahrenheit
  temperature[i] = tempF;             // Select Fahrenheit units
  delay(500);                      
}
//*******************************
void readSignalStrength(int i){
  #ifdef testMode
    Serial.println(F("*readSignalStrength*"));
  #endif      
  dbm[i] = 0;
  if (cellularMode ==1){
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
  } 
}
//*************************************
int sendATCmd(char ATCode[], char parm1[], char parm2[]){  //220622
  /*send AT  to modem, check for response and return 0 if expected response true else return -1
  *ATCode = <20 character AT comd example: AT+HTTPPARA
  *parm1, parm 2 = parameters as defined in SimComm AT Command manual
   */
  #ifdef printMode
    Serial.println(F("*sendATCmd*******************************"));
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
checkFona_:                          //I know, I know, bad structure.. fix it later
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
      
      //3b. Return -1 if OK not in response, else return  if HTTPACTION and ,200, found
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
//*************************************
static int sensorTimeIsUp(long msec){    //Read data at timed intervals  
//  #ifdef testMode
//    Serial.print(F("*SensorTimeIsUp*"));
//  #endif      
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
void setupOledDisplay(){
  #ifdef testMode
        Serial.println(F("*setupOledDisplay*"));
  #endif
  Wire.begin();
  oled.begin(&Adafruit128x64,I2C_ADDRESS); 
}
//*********************************
void setupPinModes(){                //Set Pin Modes INPUT / OUTPUT
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
  turnOffBoardLED();
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
  //if (testMode==1) {Serial.print(F("Seeking FONA"));}
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
//***********************************
int setupWifi(int i){
#ifdef WIFI  
  if (sensors[i]=="local"){return 0;}
  #ifdef printMode 
    Serial.print(F("*setupWiFi("));Serial.print(i);Serial.println(F(") *")); 
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
  if (attempt >4){
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
    if (simSleepMode ==1){
      #ifdef testMode
        Serial.print(F(" Powering down SIM module"));
      #endif
      //fona.powerDown(); //crashes
#ifdef BOTLETICS
    //
    digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
    delay(2000);                          // 100 msec pulse for SIM7000
    digitalWrite(pinFONA_PWRKEY, HIGH);
#endif  
#ifdef LILLYGO
    //fona.powerDown();
    
    digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
    delay(1500);                          // 100 msec pulse for SIM7000
    digitalWrite(pinFONA_PWRKEY, HIGH);
#endif  
    }
    delay(5000);                         // give it time to take hold - long delay
   #ifdef testMode
      Serial.print(F(" "));
   #endif        
}

//**************************************
void turnOnBoardLED(){              // Turn LED on if in test mode
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
//*********************************
void wakeUpSensors(){             // Wake up the MCP9808 if it was sleeping
  #ifdef testMode
     Serial.println(F("*WakeUpSensors*"));
  #endif      
  if (!tempsensor.begin()) {
    #ifdef testMode
       Serial.println(F(" Couldn't find the MCP9808!"));
    #endif  
  }  
  bool bme280=bme.begin(0x76);
  if (!bme280){
     #ifdef testMode
        Serial.println(F("Failed to initiate bme280"));
     #endif
  }
}
