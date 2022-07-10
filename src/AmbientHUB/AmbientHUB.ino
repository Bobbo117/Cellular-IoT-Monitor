
const char* VERSION = "AmbientHUB 2022 v7.09";

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
The code within the '#ifdef ESPNOW to #endif 
compiler directives is adapted from the following:
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
*////////////////////////////////////////////////////////////////////////////////////

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
/* The FONA library (link provided below) was enhanced to handle SIM7000 functions by Tim Woo of Botletics.
 * He releases his software under the GNU General Public License v3.0.
*/
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
*/    
//////////////////////////////////////////////////////////////  
//*******     Setup IFTTT webhook API Key        ***********//
//////////////////////////////////////////////////////////////

  //OPTIONS: 1. Disable the #include <secrets.h> statement below, enable the #define statement below that, and replace the YOUR_IFTTT_KEY_GOES_HERE phrase.
  //   or    2. Create a secrets.h file with the following line and replace the YOUR_IFTTT_KEY_GOES_HERE phrase:    
  //#define SECRET_IFTTT_HB_URL "http://maker.ifttt.com/trigger/HB/with/key/YOUR_IFTTT_KEY_GOES_HERE"  
  //

  #include <secrets.h>                  
  //#define SECRET_IFTTT_HB_URL "http://maker.ifttt.com/trigger/HB/with/key/YOUR_IFTTT_KEY_GOES_HERE"  //heartbeat event HB

//////////////////////////////////////////////////////////////  
//*******         Compile-time Options           ***********//
//////////////////////////////////////////////////////////////

  //*****************
  // 1. Assign a short prefix to the readings being sent to the cloud 
  // to identify the source especially if you have more then one HUB
  //*****************
  
  char dataTag[] = "ME";    // <<< unique prefix to identify source of data packet

  //*****************
  // 2. Select one hardware device below and comment out the others: **/
  // Board selection impacts pin assignments, setup of pin modes, 
  // pwr on & off pulse duration, and onboard LED on/off states*/
  //*****************
  
  #define BOTLETICS  //Use for Botletics or and-global
  //#define LILLYGO  

  //*****************
  // 3. Select one method of communication between HUB and sensor platforms; 
  // ESPNOW is preferred; WIFI is a memory hog:
  //*****************
  
  #define ESPNOW
  //#define WIFI
  
  //*****************
  // 4. Select one HUB temp sensor if using a HUB temp sensor:
  //****************
  
  //#define DHT_      // DHT11,21,22, etc.
  #define BME_        // BME280
  //#define MCP9808_  // botletics shield sensor

  //*****************
  // 5. Operatiing Modes
  //*****************
  
  //#define testCellular                // comment out if you dont want to enable test transmission to dweet, ifttt at reboot
  #define printMode                   // comment out if you dont want brief status display of sensor readings, threshholds and cellular interactions to pc
  //#define testMode                    // uncomment for verbose test messages to status monitor
  //#define ATMode                      // uncomment to use AT commands to reduce cellular return traffic instead of higher level FONA commands 
  #define cellularMode               //  enables a cellular connection; // no connectiod (good for debugging code without tripping cellular comms)
  //#define alarmMode                  //  enables alarm ifttt when a measurement crosses threshhold; // no alarm
  #define iFTTTMode                  //  enables use of ifttt; // no IFTTT
  #define heartbeatMode             // turns on periodic heartbeat reporting of all readings; // no heartbeat
  #define simSleepMode               // shut off sim shield between readings 
  #define OLED_                     //128 x 32 pixel OLED display if hub uses one

  //*****************
  // 6. Timing Parameters
  //*****************
  
  const int espSleepSeconds = 48*60;  // 48*60; //or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
  const long espAwakeSeconds = 12*60; // 12*60 interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
  int heartbeatMinutes = 10;          // 10 heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less thab espAwakeSeconds 

  const long serialMonitorSeconds = 5*60;      // interval in seconds to refresh serial monitor sensor readings (enhance readability)            
  const long samplingRateSeconds = 10; //1*60; // interval in seconds between sensor readings in alert mode

  //*****************    
  // 7. Sensor Inventory
  //*****************
  
  #define numberOfPlatforms 4         // number of sensor platforms available as Access Points + 1 for HUB
                                      // example: if hub has sensors, and there are 3 wireless platforms, numberOfPlatforms = 4
                                      
  // arrays to indicate types of sensors aboard each sensor platform (1=presnt, 0 = absent)
  // HUB  id=0; boards are 1,2,3..
  uint8_t temps[] = {1,1,1,1}, hums[]={1,1,1,1}, dbms[]={1,0,0,0}, pres[]={1,1,1,1}, bat[]={1,0,0,0};
  uint8_t luxs[] = {0,1,1,1}, h2os[] = {0,1,0,0},doors[]={0,0,1,1};
  
  //*****************
  // 8. platform sensor data structure
  //*****************
  
  typedef struct platforms {   // create an definition of platformm sensors
      uint8_t id;              // id must be unique for each sender; HUB  id=0; boards are 1,2,3..
      float temperature;    
      float humidity;    
      float pressure;   
      int lux;     
      int aH2o;      
      int dH2o;     
      int door; 
  } platforms;
  
  platforms sensorData; // Create a structure called sensorData to receive data from sensor platforms
  RTC_DATA_ATTR platforms platform[numberOfPlatforms]; //store the reaadings persistently in an array of structures
  RTC_DATA_ATTR int dbm[] = {-99,0,0,0};      //signal strenth not included in the platform structure
  RTC_DATA_ATTR uint16_t vbat[] = {2500,0,0,0}; //battery voltage (mv) not included in the platform structure
  //**********************

  uint8_t sensorStatus[numberOfPlatforms];  //wifi success 1 or 0 to display : or ?? on oled header in displayStatus() or to stop at 1 reading if limited awake time
  const char* sensors[]={"local","wifi","wifi","wifi"};
  
  char buf[200],buf2[20]; //formatData() sets up buf with data to be sent to the cloud. Buf2 is working buffer.
  
  #define luxThreshhold  50         // threshhold % which will cause display or IFTTT alert if enabled in hub
  #define h2oThreshhold  20         // threshhold % which will cause display or IFTTT alert if enabled in hub
  #define doorThreshhold  2000      // threshhold which will cause IFTTT alert ??
    
  //Arrays to indicate which sensors can be used to generate IFTTT alert
  uint8_t alert[numberOfPlatforms];         // used to accumulate # sensors that have alert condition

  #ifdef alarmMode
    uint8_t iftTemps[] = {0,0,0,0}, iftHums[]={0,0,0,0};
    #define tempHighLimit 90            // upper limit which will cause IFTTT alert
    #define tempLowLimit 45             // lower limit which will cause IFTTT alert
    #define tempIncrement 5             // Increment by which temp threshholds are moved to prevent multiple alarms caused by temp fluctuations around the threshhold
    int tempHighThreshReset[numberOfPlatforms] ,tempLowThreshReset [numberOfPlatforms];
    RTC_DATA_ATTR int tempHighThresh[]={tempHighLimit,tempHighLimit,tempHighLimit,tempHighLimit};
    RTC_DATA_ATTR int tempLowThresh []={tempLowLimit,tempLowLimit,tempLowLimit,tempLowLimit};

    int luxReportedLow[numberOfPlatforms];               
    int luxReportedHigh[numberOfPlatforms];
    int luxReportedHighReset[numberOfPlatforms];
    int luxReportedLowReset[numberOfPlatforms];
    
    int h2oReportedLow[numberOfPlatforms];
    int h2oReportedHigh[numberOfPlatforms];
    int h2oReportedHighReset[numberOfPlatforms];
    int h2oReportedLowReset[numberOfPlatforms];
    
    
    RTC_DATA_ATTR int doorReportedLow[]={1,1,1,1};
    RTC_DATA_ATTR int doorReportedHigh[]={0,0,0,0};
    int doorReportedHighReset[numberOfSensors];
    int doorReportedLowReset[numberOfSensors];

  #endif  

  //*****************
  // Miscellaneous
  //*****************

  uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds  
  RTC_DATA_ATTR int bootCount = 0;    //Incremented after first setup so delay only happen on initial startup gives time for user to open Status monitor 
  RTC_DATA_ATTR int heartbeatToggle=1; //used by postHeartbeat to alternate destination to ifttt.com (when value=1) and dweet.io (when value=0)

  uint8_t type;
  char imei[16] = {0};             // 16 character buffer for IMEI
  char URL[255];                   // buffer for request URL
  char body[255];                  // buffer for POST 
  
  //*****************
  // Timer stuff
  //*****************
  
  extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h" 
  }
  
  //*****************
  // Botletics SIM7000A Arduino Shield Connections to ESP32
  //*****************

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
  // Pin connections used for sensors
  //*****************
  
  #define pinDleak 34                 // esp32 analog input                              
  #define pinAleak 39                 // esp32 analog input
  #define pinPhotoCell 36             // esp32 analog input
  #ifdef DHT_
    #define pinDHT 13  //5, 27 not 4    // DHT temperature & Humidity sensor
  #endif
  
  //*****************
  // Sensor Libraries
  //*****************

  #include <Wire.h>                 // 12C
  #include <Adafruit_Sensor.h>
  #ifdef BME_
    #include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
    Adafruit_BME280 bme;            // temp hum pressure sensor
  #elif MCP9808_
    #include "Adafruit_MCP9808.h"   // shield temp sensor (on 12C bus)
    Adafruit_MCP9808 tempsensor = 
         Adafruit_MCP9808();        // Create the MCP9808 temperature sensor object
  #elif DHT_
    #include "DHT.h"
    #define DHTTYPE DHT11  //or DHT22
    DHT dht(pinDHT,DHTTYPE);
  #endif 
        
  //*****************
  // OLED Libraries
  //*****************
  #ifdef OLED_
    #include "SSD1306Ascii.h"     // low overhead library text only
    #include "SSD1306AsciiWire.h"
    #define I2C_ADDRESS 0x3C      // 0x3C+SA0 - 0x3C or 0x3D
    #define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
    SSD1306AsciiWire oled;
  #endif   

  //*****************
  // WIFI Libraries
  //*****************
  
  #ifdef WIFI
    #include <WiFi.h>
    #include <HTTPClient.h>
    const char* wifiSSID[] = {"n/a","AMBIENT_1","AMBIENT_2","AMBIENT_3"};
    const char* wifiPassword[] = {"","","",""};
    //don't need all this, but it is convenient for debug purposes:
    //Your IP address or domain name with URL path
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
  // FONA Library
  //*****************

  #include "Adafruit_FONA.h"          //from https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
  //#include <Adafruit_FONA.h>
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
  #include <HardwareSerial.h>
  HardwareSerial fonaSS(1);
  #define SIMCOM_7000                // SIM7000A/C/E/G

  //*****************
  // ESPNow Libraries
  //*****************
  
  #ifdef ESPNOW
    #include <esp_now.h>
    #include <WiFi.h>
  #endif

//*****************
// *** Setup() ***
//*****************
  
void setup() {
   delay(500);                  //ESP32 Bug workaround -- don't reference rtc ram too soon.
   setupPinModes();             // Set Pin Modes INPUT / OUTPUT (necessary to turn board LED on, etc.) 
   if (bootCount ==0) {blinkBoardLED(5);}    // blink LED for 5 sec for feedback and to give user time to activate serial console
   initializeArrays();          // Initialize sensor threshholds
   setupOledDisplay(); 
   for (uint8_t i=0; i< numberOfPlatforms; i++){  // for each sensor:
      displayStatus(i);         // update OLED display with new or prior readings      
   }
   #ifdef printMode
      Serial.begin(115200);          // Initialize Serial Monitor 
      Serial.print(F("***** ")); Serial.print(F(VERSION));Serial.println(F(" *****starts after 5 sec ***"));
      Serial.print(F("bootCount= "));Serial.println(bootCount);
      printWakeupReason();
      printSensorData();
   #endif
   wakeUpSensors();                 // Wake up the HUB temp sensors 
   #ifdef cellularMode              // If cellular comms is enabled
     if (bootCount ==0){            // if first boot (not wakeup)
       powerUpSimModule();          // Power up the SIM7000 module by goosing the PWR pin 
       delay (5000);
       simModuleOffIfSimSleepMode();// Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
       #ifdef testCellular          //if we want to test cellular data on first startup
          activateCellular(); 
          readSignalStrength(0);
          postHeartbeatData();       // upload test data to ifttt.com if enabled
          postHeartbeatData();       // upload test data to dweet.io if enabled
          displayStatus(0);          // update OLED display to show test data
          simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
       #endif  
     }
   #endif
   delay (5000);
   setupEspNow();                   //initiaalize communication with with external sensor platforms if enabled
   bootCount++;
}

//*****************
// ***  Loop()  ***
//*****************
  
void loop() {    
  //0. Go to sleep if snooze mode nap time
    if (espSleepSeconds >0 && espAwakeSeconds >0 && espAwakeTimeIsUp(espAwakeSeconds*1000) ==1 ){
      #ifdef printMode
        Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
      #endif
      ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);
    }
 
  //1. read local and remote sensors 
  if (sensorTimeIsUp(samplingRateSeconds*1000) ==1 ){
    for (uint8_t i=0; i< numberOfPlatforms; i++){  // for each sensor:
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
      displayStatus(i);         // update OLED display if thee is one 
    }
    delay(5000);                // allow latest display time before toggle timer takes over
    int alerts = 0;             // determine whether we have one or more alerts:    
    for (int i=0;i<numberOfPlatforms;i++){  //determine if an alarm threshhold was crossed
      alerts=alerts+alert[i];
    }
    if (alerts>0){              // if we have alerts, activate cellular network and report via ifttt if alarmMode set
      #ifdef iFTTTMode 
        #ifdef alarmMode
          #ifdef cellularMode
            activateCellular(); 
            readSignalStrength(0);
            postToIfTTT();
            displayStatus(0);             // update OLED display 
            simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode 
          #endif
        #endif  
      #endif  
    }
  }

  
  //2. heartbeat: upload latest data if its time
  #ifdef cellularMode
    if (heartbeatTimeIsUp(heartbeatMinutes*60000L) ==1 ){
      activateCellular();
      readSignalStrength(0);
      postHeartbeatData();          // upload sensor data to dweet.io or IFTTT if heartbeatMode enabled
      displayStatus(0);             // update OLED display here to update signal strength
      simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode
    }
  #endif
  
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
  #ifdef cellularMode
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
int connectToCellularNetwork() {  
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

  #ifdef OLED_
    if (numberOfPlatforms>=2){
      oled.setFont(Arial_14);
      oled.setCursor(0,16*i);
      if (sensorStatus[i] ==0){
        oled.print("?");  //indicates old or no measurement
      }else{  
        oled.print(":");  //indicates current measurement
      }
    }else{
      oled.setFont(Arial_bold_14);
      oled.setCursor(0,0);
    }
    int value = platform[i].temperature + .5;
    oled.print(value);
    oled.print(" ");  
    value = platform[i].humidity + .5;
    oled.print(value);
    oled.print("% ");
  
    if (i>=1){
      //if (dbms[i]==1){oled.print(9+dbm[i]/10);}else{oled.print("-");}
      if (dbms[i]==1){oled.print(dbm[i]/10);}else{oled.print("-");}
    }else{
      oled.print(dbm[i]); 
    }
  
    if (h2os[i]==0){oled.print("--- ");
      }else{
      if (platform[i].aH2o - h2oThreshhold > 0){
        oled.print("Wet ");
      }else{
        oled.print("Dry ");
      }
    } 
    if (doors[i]==0){oled.print("---- "); 
      }else{
      if(platform[i].door==0){
        oled.print("Shut ");
      }else{
        oled.print("Open ");
      }
    }
    if (luxs[i]==0){oled.println("---"); 
    }else{
      oled.print(platform[i].lux);
      oled.println("%");
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
  // ESPNOW callback function that will be executed when data is received
  #ifdef ESPNOW
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

      //transfer incoming data to array
      platform[sensorData.id].id = sensorData.id;
      platform[sensorData.id].temperature = sensorData.temperature;
      platform[sensorData.id].humidity = sensorData.humidity;
      platform[sensorData.id].pressure = sensorData.pressure;
      platform[sensorData.id].lux = sensorData.lux;
      platform[sensorData.id].aH2o = sensorData.aH2o;
      platform[sensorData.id].dH2o = sensorData.dH2o;
      platform[sensorData.id].door = sensorData.door;
      
      #ifdef printMode    
        Serial.println("id      temp    hum    pres    lux   aH2o    dH2o    door");
        Serial.print(sensorData.id);Serial.print("\t");
        Serial.print(sensorData.temperature);Serial.print("\t");
        Serial.print(sensorData.humidity);Serial.print("\t");
        Serial.print(sensorData.pressure);Serial.print("\t");  
        Serial.print(sensorData.lux);Serial.print("\t");     
        Serial.print(sensorData.aH2o);Serial.print("\t"); 
        Serial.print(sensorData.dH2o);Serial.print("\t"); 
        Serial.println(sensorData.door);
      #endif 
  #endif
}

//*********************************
void formatData(){                   //transfer comma separated sensor data to buf
  #ifdef testMode 
    Serial.println(F("*formatData*"));
  #endif

 // buf[0]={0};                           //Initialize data buffer
  strcpy (buf,dataTag);strcat(buf,"_"); //enter data TAG and separator

  //process temperature
  for(int i=0;i<numberOfPlatforms;i++){
    if(temps[i]==1){
      dtostrf(platform[i].temperature, 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
        strcat(buf,"F_");
      }else{
          strcat(buf,",");   //comma separator causes error in http get in sendAT as written 
      }
    }
  }

  //process humidity
  for(int i=0;i<numberOfPlatforms;i++){
    if(hums[i]==1){
      dtostrf(platform[i].humidity, 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
        strcat(buf,"%_");
      }else{
        strcat(buf,","); 
      }
    }
  }

  //process photocell lux
  for(int i=0;i<numberOfPlatforms;i++){
    if(luxs[i]==1){
      dtostrf(platform[i].lux, 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
        strcat(buf,"_");
      }else{
        strcat(buf,","); 
      }
    }
  }

  //process door
  for(int i=0;i<numberOfPlatforms;i++){
    if(doors[i]==1){
      dtostrf(platform[i].door, 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
        strcat(buf,"_");
      }else{
        strcat(buf,","); 
      }
    }
  }
  
  //process ah2o
  for(int i=0;i<numberOfPlatforms;i++){
    if(h2os[i]==1){
      dtostrf(platform[i].aH2o, 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
        strcat(buf,"_");
      }else{
        strcat(buf,","); 
      }
    }
  }
  
  //process signal strength last
  for(int i=0;i<numberOfPlatforms-1;i++){
    if(dbms[i]==1){
      dtostrf(dbm[i], 1, 0, buf2); 
      strcat(buf, buf2);
      if (i==(numberOfPlatforms-1)){
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
String httpGETRequest(const char* serverName) {
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
    return payload;
  #endif 
}

//************************************
void initializeArrays(){
  #ifdef testMode 
    Serial.println(F("*initializeArrays*"));
  #endif  
  for (int i=0; i<numberOfPlatforms; i++){ 
    #ifdef alarmMode
      tempHighThreshReset[i] = tempHighThresh[i];  //do it this way to preserve sleep vars
      tempLowThreshReset[i] = tempLowThresh[i];
    #endif  
    sensorStatus[i]=0;  //This causes "?" to display on OLED, indicating no real data
    platform[i].id = numberOfPlatforms+1;
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
  #ifdef  heartbeatMode  
    formatData();            // transfer sensor data to buf
    int i = 0;               // Count the number of failed attempts 
    if(heartbeatToggle==0){  //if 0, dweet:
      // GET request use the IMEI as device ID
      //sprintf(URL, "http://dweet.io/dweet/for/%s?T=%s",imei,buf);  //option provides T= prefix to data
      //sprintf(URL, "http://dweet.io/dweet/for/%s?%s",imei,buf);
      snprintf(URL, sizeof(URL),"http://dweet.io/dweet/for/%s?%s",imei,buf);

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
    }else{               // if 1, ifttt:
      #ifdef iFTTTMode
        snprintf(body, sizeof(body),"{\"value1\":\"%s\"}", buf);
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
        #endif //ATMode
      #endif   //IFTTTMode
      heartbeatToggle=0; 
    }   
    #ifdef printMode
      Serial.print(F("failed attempts = "));Serial.println(i);
    #endif  
  #endif //heartbeatMode 
}

//************************************
void postToIfTTT(){                //Post alert to IFTTT if iFTTTMode is enabled
  #ifdef printMode 
     Serial.println(F("*postToIfttt*"));
  #endif
  #ifdef iFTTTMode
  #ifdef alarmMode
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
        for (int i=0; i<numberOfPlatforms;i++){
          tempHighThresh[i] = tempHighThreshReset[i];
          tempLowThresh[i] = tempLowThreshReset[i];
          #ifdef printMode
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
        }  
      } 
    #else  //send AT codes instead of fona
      int err = sendATCmd("AT+HTTPTERM","","");
      err = sendATCmd("AT+HTTPINIT","","");
      err = sendATCmd("AT+HTTPPARA","=CID","1");
      sprintf(URL, "%s?value1=%s",SECRET_IFTTT_HB_URL,buf);
      err = sendATCmd("AT+HTTPPARA","=URL",URL);
      delay(200);
      err = sendATCmd("AT+HTTPACTION","=0","");
      err = sendATCmd("AT+HTTPTERM","","");         
    #endif  
    
  #endif         //alarmMode  
  #endif         //iFtttMode
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
  #ifdef printMode
    Serial.println(F("*printSensorData*"));
    for (int i=0;i<numberOfPlatforms;i++){ 
       Serial.print(platform[i].id);Serial.print(F(": "));
       Serial.print(platform[i].temperature);Serial.print(F(" F ")); 
       Serial.print(platform[i].humidity);Serial.print(F(" % ")); 
       Serial.print(platform[i].pressure);Serial.print(F(" mb ")); 
       Serial.print(platform[i].lux);Serial.print(F(" lux "));
       Serial.print(platform[i].aH2o);Serial.print(F(" aH2o "));
       Serial.print(platform[i].dH2o);Serial.print(F(" dH2o "));
       Serial.print(platform[i].door);Serial.println(F(" door "));
    }
  #endif
}

//********************************
void printWakeupReason(){
  #ifdef printMode  //print_wakeup_reason()
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

//*********************************
int processIfTTT(uint8_t i) {              // set flag to post alert to iFTTT if temp outside limit
                                       // and adjust temp threshholds 
  #ifdef printMode
    Serial.print(F("*processIfTTT*")); Serial.print (i); Serial.print(F(": "));
    #ifdef alarmMode 
     //Serial.print("temp = " + String(temperature[i]));
     Serial.print(" Thresh: " + String(tempLowThresh[i]));
     Serial.print(" - " + String(tempHighThresh[i]));
     Serial.print(" lux: = " + String(luxReportedLow[i]));
     Serial.print(" - " + String(luxReportedHigh[i]));
     Serial.print(" h2o: " + String(h2oReportedLow[i]));
     Serial.print(" - " + String(h2oReportedHigh[i]));
     Serial.print(" door: " + String(doorReportedLow[i]));
     Serial.print(" - " + String(doorReportedHigh[i]));
    #endif  
  #endif

  int flag=0;           //flag preset tp indicate no alerts

  #ifdef alarmMode  
    tempHighThreshReset[i] = tempHighThresh[i];
    tempLowThreshReset[i] = tempLowThresh[i];
    luxReportedLowReset[i] = luxReportedLow[i];
    luxReportedHighReset[i] = luxReportedHigh[i];
    h2oReportedLowReset[i] = h2oReportedLow[i];
    h2oReportedHighReset[i] = h2oReportedHigh[i];
    doorReportedHighReset[i] = doorReportedHigh[i];
    doorReportedLowReset[i] = doorReportedLow[i];
        
    if (iftTemps[i]==1){
    if (platform[i].temperature - tempHighThresh[i] > 0) {
      #ifdef testMode
        Serial.print(F(" ;1-Temp exceeds ")); Serial.print (tempHighThresh[i]);
      #endif    
      tempHighThresh[i] = tempHighThresh[i] + tempIncrement;
      flag=1; //postToIfTTT();
      goto lux;    
    }  
    if (platform[i].temperature - (tempLowThresh[i] )<0) {      
      #ifdef testMode
        Serial.print(F(" ;2-Temp is less than ")); Serial.print(tempLowThresh[i]);
      #endif
      tempLowThresh[i] = tempLowThresh[i] - tempIncrement;  
      flag=1; //postToIfTTT(); 
      goto lux;    
    }
    if (platform[i].temperature - (tempHighThresh[i] -1.5*tempIncrement)<0) {  //not sure about this
      if(tempHighThresh[i]-tempHighLimit>0){
        #ifdef testMode
          Serial.print(F(" ;3-Temp is less than ")); Serial.print(tempHighThresh[i]);  
        #endif  
        tempHighThresh[i] = tempHighThresh[i] - tempIncrement;  
        flag=1;  //postToIfTTT(); 
        goto lux;    
      }
    }
    if (platform[i].temperature - (tempLowThresh[i] + 1.5*tempIncrement)>0) {
      if(tempLowThresh[i]-tempLowLimit<0){
        #ifdef testMode
          Serial.print(F(" ;4-Temp exceeds ")); Serial.print (tempLowThresh[i]);  
        #endif  
        tempLowThresh[i] = tempLowThresh[i] + tempIncrement;  
        flag=1;  //postToIfTTT(); 
        goto lux;    
      }
    }
    }  //iftTemps

lux:
    if (iftLuxs[i]==1){
    //if (testMode==1) {Serial.println(lux[i]);  }
    if(lux[i] - luxThreshhold > 0){
      //if (testMode==1) {Serial.println("lights are ON");}
      if (luxReportedHigh[i]==0) {
        luxReportedHigh[i]=1 ;
        luxReportedLow[i] = 0;
        flag=1; //postToIfTTT(); 
        if (testMode==1) {Serial.print(F("; Lights on alert")); }
      }
    }else{
      //if (testMode==1) {Serial.println("Lights are OFF");}
       if (luxReportedLow[i]==0) {
        luxReportedLow[i]=1 ;
        luxReportedHigh[i] = 0;
        flag=1; //postToIfTTT(); 
        if (testMode==1) {Serial.print(F("; Lights off alert")); }
       }
    }
    }
doors:
    if(iftDoors[i]==1){
    if(door[i] == 1){
      ////if (testMode==1) {Serial.println("door open");}
      if (doorReportedHigh[i]==0) {
        doorReportedHigh[i]=1 ;
        doorReportedLow[i] = 0;
        flag=1; //postToIfTTT(); 
        if (testMode==1) {Serial.print(F(" ;door open alert")); }
      }
    }else{
     // //if (testMode==1) {Serial.print("door closed");}
       if (doorReportedLow[i]==0) {
        doorReportedLow[i]=1 ;
        doorReportedHigh[i] = 0;
        flag=1; //postToIfTTT();
        if (testMode==1) {Serial.println(F(" ;door closed alert")); }
       }
    }
    }
h2o:
    if(iftH2os[i]==1){
    if(aH2o[i] -100 > 0){
      if (h2oReportedHigh[i]==0) {
        h2oReportedHigh[i]=1 ;
        h2oReportedLow[i] = 0;
        flag=1; //postToIfTTT(); 
       if (testMode==1) {Serial.print(F("; flood alert")); }
      }
    }else{
      if (h2oReportedLow[i]==0) {
        h2oReportedLow[i]=1 ;
        h2oReportedHigh[i] = 0;
        flag=1; //postToIfTTT();
        if (testMode==1) {Serial.print(F("; NO flood alert")); }
       }
    }
    }
    
      #ifdef testMode
        Serial.println(F(" "));
      #endif  
  #endif  //alarmMode
  
  return(flag);
}

//***********************************
void readBatteryVoltage (uint8_t i){  //crashes if using fona - send 2500 for now
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
    Serial.print(F("*readSensorData*"));Serial.print(i);Serial.println(F("*"));
  #endif      

  uint8_t returnFlag = 1;  //return success causes : display on OLED
  if (sensors[i]=="local"){
    // readSignalStrength(i); causes crash
     readBatteryVoltage(i);

    platform[i].id=i;
    
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
  
    #ifdef BME_
      float x = 1.8*bme.readTemperature()+32.0;
      if (isnan(x)){
        returnFlag=0;         //causes ? on OLED
        //x=temperature[i];
        x=platform[i].temperature;
      }
      //temperature[i]=x;
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
         //x=temperature[i];
         x=platform[i].temperature;
       }
       //temperature[i]=x;
       platform[i].temperature = x;
       
       x = (dht.readHumidity());
       if (isnan(x)){
         x=platform[i].humidity;
       }
       platform[i].humidity = x;
     #endif  
  
     if(luxs[i]==1){         //read photocell if present
       platform[i].lux =  2.44*(analogRead(pinPhotoCell)/100); //whole number 1-99 %
     }  
  
     if(h2os[i]==1){         //read water detecter if present
       platform[i].aH2o = 2.44*(4095-analogRead(pinAleak))/100;   //whole number 1-99 %
       platform[i].dH2o = digitalRead(pinDleak);
     }
      
  }else if (sensors[i]=="wifi"){ 
    #ifdef WIFI
      // Check WiFi connection status
      #ifdef testMode
          Serial.print(F("WL_CONNECTED = "));Serial.println(WL_CONNECTED);
          Serial.print(F("WiFi.status() = "));Serial.println(WiFi.status());
      #endif
  
      if(WiFi.status()!=WL_CONNECTED ){ 
        setupWifi(i);
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
    #ifdef simSleepMode 
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
   #endif
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
  #ifdef MCP9808_      
    if (!tempsensor.begin()) {
      #ifdef testMode
         Serial.println(F(" Couldn't find the MCP9808!"));
      #endif  
    } 
  #endif
  #ifdef BME_ 
    bool bme280=bme.begin(0x76);
    if (!bme280){
       #ifdef testMode
          Serial.println(F("Failed to initiate bme280"));
       #endif
    }
  #endif
}
