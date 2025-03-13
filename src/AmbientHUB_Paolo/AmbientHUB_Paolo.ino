
const char* APP = "AmbientHUB ";
const char* VERSION = "2025 0313";

/* Mar 13, 2025 modifications
 *  set up HUB only w/bme280 or DHT11 only; OLED optional; dweet optional
 *  
 */

/*
  Copyright 2022 Robert Jessup  MIT License:
  https://github.com/Bobbo117/Cellular-IoT-Monitor/blob/main/LICENSE
*/

//////////////////////////////////////////////////////////////  
//*******         Compile-time Options           ***********//
//        (Disable unwanted options with leading //)
//////////////////////////////////////////////////////////////

  #define OLED_               // 64x128 pixel OLED display if hub uses one
  #define printMode           // comment out if you don't want brief status display of sensor readings, threshholds and cellular interactions to pc
  //#define testMode          // uncomment for verbose test messages to status monitor for heavy debug / verification
  #define cellularMode        //  enables a cellular connection; // = no connection (for debugging code without racking up cellular costs)
  #define dweetMode           // enables use of dweet. www.Dweet.com is free.  The cellular SIM imei is used as a key to
                              // log and monitor sensor values.
  char dataTag[] = "XX";   // <<< unique prefix to identify source of data >
  #ifdef cellularMode
    //#define simSleepMode   // shut off sim modem between readings 
  #endif
    
    //*****************
    // Timing Parameters
    //***************** 
    const int espSleepSeconds = 0; //48*60;     // or 0; heartbeat period OR wakes up out of sleepMode after sleepMinutes           
    const long espAwakeSeconds = 5*60;    // interval in seconds to stay awake as HUB; you dont need to change this if espSleepSeconds = 0 indicating no sleep
    int heartbeatMinutes = 6; //60; //10;             // (default 10) heartbeat delay after HUB awakens.  this needs to be at least 2 minutes less than espAwakeSeconds
                                           // IF espSleepSeconds - 0, then this is the heartbeat frequency.
    const long chillSeconds = 10;          // (not used)interval between readings if sleepSeconds = 0 slows serial monitor updates
    const long sensorSeconds = 1*60;        // interval in seconds to refresh time sensitive sensor readings 
  
    const long serialMonitorSeconds = 5*60; // (growth) interval in seconds to refresh serial monitor sensor readings (enhances readability)            
    const long samplingRateSeconds = 10;    // (growth) interval in seconds between sensor readings in alertMode
  
    #define bootResetCount 24         //reset ESP once per day

    //*****************
    // Temperature sensor 
    //****************
    #define BME_        // BME280 temperature, humidity, pressure sensor
    //#define DHT_      // DHT11,21,22, etc. temperature, humidity sensors

  //*****************    
  // 3. Sensor Inventory
  //*****************
  uint8_t sensorID = 0;               //HUB identifier = 0 see next comment  
  // arrays to indicate types of sensors aboard each sensor platform (1=presnt, 0 = absent)
  // HUB  sensorID=0; boards are 1,2,3.. example: temps[]={1,0,1,0} indicates hub and sensor #2 have temp sensors, sensor #1 and #3 do not.

    #define numberOfPlatforms 1         // number of sensor platforms available  + 1 for HUB
                                        // example: if hub has sensors, and there are 3 wireless platforms, numberOfPlatforms = 4
                                        //const char* location[] = {"hub","garage","sonic","hatch", "kitchen","bathroom","bathroom2,"spare"};
    uint8_t temps[] = {1,1,0,0, 1,1,1,0}, hums[]=  {1,1,0,0, 1,1,1,0}, dbms[]= {1,0,0,0, 0,0,0,0}, pres[]={0,0,0,0, 0,0,0,0}, bat[]=   {0,0,0,0, 0,0,0,0};
    uint8_t luxs[] =  {0,1,0,0, 0,0,0,0}, h2os[] = {0,0,0,0, 0,0,0,0}, doors[]={0,1,0,1, 0,0,0,0}, pirs[]={0,1,0,0, 1,1,1,0}, sonics[]={0,0,1,0, 0,0,0,0};
 
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
  //RTC_DATA_ATTR uint8_t priorDoor = 0; //last door status = 0 (closed) or 1 (open)
  RTC_DATA_ATTR int dbm[] = {-00,0,0,0, 0,0,0,0};      //hub signal strenth not included in the platform structure
  //RTC_DATA_ATTR int16_t vbat[] = {0,0,0,0, 0,0,0,0}; //hub battery voltage (mv) not included in the platform structure
  //**********************
  uint8_t platformStatus[numberOfPlatforms];
  uint8_t sensorStatus[numberOfPlatforms];  //wifi success 1 or 0 to display : or ?? on oled header in displayStatus() or to stop at 1 reading if limited awake time
  const char* sensors[]={"local","wifi","wifi","wifi","wifi","wifi","wifi","wifi"};
     
  uint8_t postTries = 3;  //allowed data post attempts
  
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
  // LillyGo SIM7000G Connections to on-board Wrover ESP32
  //*****************
        
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

  //*****************
  // 8. Pin connections used for sensors
  //*****************
  
  #define pinSDA 21                   // ESP 12C Bus SDA for temp sensor, OLED, etc
  #define pinSCL 22                   // ESP 12C Bus SCL for temp sensor, OLED, etc
  #ifdef DHT_
    #define pinDHT 32                 // DHT temperature & Humidity sensor
  #endif
   
  //*****************
  // 9. Temperature Sensor Libraries
  //*****************

  #include <Wire.h>                 // 12C
  #include <Adafruit_Sensor.h>

  #ifdef BME_
    #include <Adafruit_BME280.h>    // library for BME280 temp hum pressure sensor
    Adafruit_BME280 bme;            // temp hum pressure sensor
  #endif  

  #ifdef DHT_                       
    #include "DHT.h"                // library for DHTxx temp sensor
    #define DHTTYPE DHT11           // can also be DHT22, etc
    DHT dht(pinDHT,DHTTYPE);
  #endif 
  
  
  //*****************
  // 10. OLED Libraries
  //*****************
  #ifdef OLED_
    #include "SSD1306Ascii.h"     // low overhead library text only
    #include "SSD1306AsciiWire.h"
    #define I2C_ADDRESS 0x3C      // 0x3C+SA0 - 0x3C or 0x3D
    #define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
    SSD1306AsciiWire oled;
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

          oled.clear(); 
          //oled.setFont(Arial_bold_14);
          oled.setFont(fixed_bold10x15);
          oled.setCursor(0,0);
          oled.print("Temp: ");oled.print(int(platform[i].temperature+.5));oled.println(" F");
          oled.print("Hum:  ");oled.print(int(platform[i].humidity+.5));oled.println(" %");
          oled.print("Pres: ");oled.print(int(platform[i].pressure+.5));oled.println(" mb");
          oled.print("dbm:  ");oled.println(dbm[i]);
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

//************************************
void initializeArrays(){
  #ifdef testMode 
    Serial.println(F("*initializeArrays*"));
  #endif  
  for (uint8_t i=0; i<numberOfPlatforms; i++){ 
    sensorStatus[i]=0;  //This causes "?" to display on OLED, indicating no real data
    platformStatus[i]=0;
    platform[i].id = numberOfPlatforms+1;
  }
}

//************************************
void napTime(){
  if (espSleepSeconds >0 && espAwakeSeconds >0 && espAwakeTimeIsUp(espAwakeSeconds*1000) ==1 ){
    #ifdef printMode
      Serial.println(F(" "));Serial.print(F("sleeping "));Serial.print(espSleepSeconds);Serial.println(F(" seconds..zzzz"));
    #endif
    ESP.deepSleep(espSleepSeconds * uS_TO_S_FACTOR);
  }
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

  //dweet processing
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
    
    strcpy(prelude,"");    
    return 0;
    
    #endif //dweetMode  
  return retn;
}

//*********************************
void postSensorHeartbeatData(){
  if (heartbeatTimeIsUp(heartbeatMinutes*60000L) ==1 ){
    if (espSleepSeconds ==0){
      bootCount++;
    }
    #ifdef cellularMode
      activateCellular();
      readSignalStrength(0);
      postSensorData();          
      displayStatus(sensorID);      // update OLED display here to update signal strength
      simModuleOffIfSimSleepMode(); // Power off the SIM7000 module by goosing the PWR pin if simSleepMode
    #endif
  }  
}

//*********************************
void powerUpSimModule() {                // Power on the module
  #ifdef printMode
    Serial.println(F("*powerUpSimModule*"));
  #endif

    pinMode(pinFONA_PWRKEY,OUTPUT);
    digitalWrite(pinFONA_PWRKEY, LOW);
    delay(1000);                           // datasheet ton = 1 sec
    digitalWrite(pinFONA_PWRKEY, HIGH);
    delay(5000);

}     
                 
//*********************************
void printSensorData(){
  if(chillTimeIsUp(chillSeconds*1000)==1){   //slow down loop for serial monitor readability
    #ifdef printMode
      Serial.println(F("*printSensorData*"));
      Serial.println(F("id      temp    hum    "));
      
      for (int i=0;i<numberOfPlatforms;i++){ 
        //Serial.print(platform[i].id);Serial.print("\t");
        Serial.print(i);Serial.print("\t");
        Serial.print(platform[i].temperature);Serial.print("\t");
        Serial.println(platform[i].humidity);

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


//*************************************
void  processSensorPlatforms(){
  //Read local and remote sensors if available and display on OLED display and serial monitor
  for (uint8_t i=0; i< numberOfPlatforms; i++){  // for each sensor:

    #ifndef HTTPGET
      platformStatus[i]=0;
    #endif  
//Serial.print("processSensorPlatforms platformStatus(");Serial.print(i); Serial.print(") = ");Serial.println(platformStatus[i]);
    if (platformStatus[i]==0){    // if sensor has not been sampled yet:    
          platformStatus[i]= readSensorData(i); // need if no sleep time      
          if(platformStatus[i]==1){ //determine if sensor data has been updated
            sensorStatus[i]=1;    // indicate new data - display "%" after humidity in displayStatus()
            //#ifdef WIFI
              printSensorData();
            //#endif  
           }  
    }  
      displayStatus(i);                   // update OLED display if there is one
      platform[i].id=numberOfPlatforms+1; // indicate data has been processed; new data will update id
  }   
}    

   
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

//*********************************
uint8_t readSensorData(uint8_t i){             // temp & voltage 

  uint8_t returnFlag = 1;  //return success causes ':' display on OLED
  if (sensors[i]=="local"){
  
    platform[i].id=i;
  
   if(sensorTimeIsUp(sensorSeconds*1000)==1){   //slow down loop for sensor R&R   
      
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
  
    }else{
      returnFlag = 0;
    }
      
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
uint8_t readWakeupID(){
  //read wakeup reason and return code
  #ifdef printMode 
    Serial.println(F("*readWakupID*"));
  #endif   
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
       return 0;
    case ESP_SLEEP_WAKEUP_EXT1 :      
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
}

//*********************************
void setupSensors(){             // Wake up the MCP9808 if it was sleeping
  #ifdef printMode
     Serial.println(F("*WakeUpSensors*"));
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
 
}

//*********************************
int setupSimModule() {  
  #ifdef printMode
    Serial.println(F("*setupSimModule*"));
    Serial.println(F("fonaSS.begin(115200, SERIAL_8N1, pinFONA_TX, pinFONA_RX)"));
    Serial.println(F("fonaSS.println('AT+IPR=9600')"));    
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
void simModuleOffIfSimSleepMode(){    // Power off the SIM7000 module by goosing the PWR pin if simSleepMode = 1
  #ifdef printMode
    Serial.println(F("*simModuleOffIfSimSleepMode*"));  
  #endif    
  #ifdef simSleepMode 
    #ifdef testMode
      Serial.print(F(" Powering down SIM module"));
    #endif
    //fona.powerDown(); //crashes
    

     //fona.powerDown();
    
     digitalWrite(pinFONA_PWRKEY, LOW);   // turn it on
     delay(1500);                          
     digitalWrite(pinFONA_PWRKEY, HIGH);
   
 #endif
 delay(5000);                         // give it time to take hold - long delay
 #ifdef testMode
    Serial.print(F(" "));
 #endif        
}

//**************************************
void turnOnBoardLED(){              // Turn LED on 

    digitalWrite(pinBoardLED, LOW);

} 

//*********************************    
void turnOffBoardLED(){             // Turn LED off
 
    digitalWrite(pinBoardLED, HIGH);

}    

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
  setupSensors();                  // Wake up the HUB temp sensors 
  turnOffBoardLED();
}

/////////////////
// ***  Loop()  ***
////////////////// 
void loop() {

  processSensorPlatforms(); 

  postSensorHeartbeatData();

//Go to sleep if snooze mode time has elapsed
  napTime();

   
  #ifdef printMode
    delay(1000);    //delay before looping so that staus display is readable if printMode is enabled
  #endif  
}
