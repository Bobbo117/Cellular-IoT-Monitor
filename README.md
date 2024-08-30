# Cellular IoT Monitor  

*Transmit sensor data from remote locations without internet.*
*Transfer data to email, text, Google Sheets, IFTTT, Home Assistant, and more for less than $2 per month.*

<br>

## How It Works

An inexpensive IoT SIM card gains access to the cloud via a cellular connection.

The HUB connects to the cellular network or wifi or both.

The HUB can operate as a stand-alone unit with its own sensors.

The HUB can also collect data wirelessly from separate sensor platforms.

The hub software - **AmbientHUB** - and 
the sensor software - **AmbientAP** - 
are written in C++ with the using the **Arduino IDE**.

The **AmbientHUB** software controls an **ESP32**
micro-controller connected with onboard sensors 
and an optional **OLED** display.

<br>

## Hands on Technology presentation of 2/2/2022

Available in the doc folder (see Cellular IoT Homewatch.pdf).  
Additional development since then is reflected below and in the software.

<br>
<br>

## Tested With

-   [**Botletics** `SIM7000A` cellular modem Arduino shield]( 
    https://github.com/botletics/SIM7000-LTE-Shield/wiki)

-   [**LiLLYGO** `SIM7000G` board with 
    integrated **ESP32** Wrover](
    https://www.amazon.com/LILYGO-Development-ESP32-WROVER-B-Battery-T-SIM7000G/dp/B099RQ7BSR)
    
    ![20230607_105519](https://github.com/Bobbo117/gpsTracker/assets/58577175/be021b8f-4aa4-4a6b-95e0-d9151d2ecfd4)

-   [**and-global** `SIM7000A` module](
    https://www.and-global.com/)

<br>
<br>

## Supported Protocols

The **AmbientHUB** software can receive the **AmbientAP** 
sensor data using the listed protocols.

-  `http` where each sensor is configured 
   as a server which can be polled by the 
   hub using a **GET** command.

-  `ESP-Now` a lighter footprint protocol by 
   which the sensors push the data to the 
   hub based on its MAC address.
   https://www.espressif.com/en/products/software/esp-now/resources

<br>
<br>

## Formatting Options

The **AmbientHUB** software collects the sensor values 
for hourly transmission via the **Hologram** IoT cellular 
platform to the listed website services.

### Supported Services via Cellular

-    https://io.adafruit.com/ MQTT for dashboard and redundant IFTTT webhook route

-    https://ifttt.com/ for notifying to up to 20 email accounts plus Google sheets

-    https://dweet.io/ which simply saves the messages in scratchpad format

### Supported Services via Wifi

-    Home Assistant or any MQTT Broker using a local wifi router

-    Telegram for tailored commands


<br>
<br>

## Threshold Alert

The **AmbientHUB** software provides <br>
immediate alerts when sensors cross adaptive thresholds.

<br>

### Builtin Modules

-   An [ESP32](//www.amazon.com/MELIFE-Development-Dual-Mode-Microcontroller-Integrated/dp/B07Q576VWZ?th=1) or a D1 Mini 8266 micro-controller

-   temperature / humidity sensors
   `BME-280`, MCP9808, `DHTxx`, AHT10, SHT20

-   An optional `SSD1306` OLED display 
    https://www.amazon.com/gp/product/B07W1PT6VK

-   [door / window sensors](
    https://www.amazon.com/dp/B09BJLRK4S/)

-   [illumination sensors](
    https://www.amazon.com/eBoot-Photoresistor-Sensitive-Resistor-Dependent/dp/B01N7V536K/)

-   [flood sensors](
    https://www.amazon.de/-/en/Sensor-Moisture-Splash-Arduino-Raspberry/dp/B01MRIBI2M)
    
-   [PIR/ motion Sensors](
    https://www.amazon.com/DIYmall-HC-SR501-Motion-Infrared-Arduino/dp/B012ZZ4LPM)
    
-   [HC-SR04 or HC-SR04P Ultrasonic sensor](
    https://www.amazon.com/dp/B07VZBYSLX/ref=cm_sw_em_r_mt_dp_05X0C8N082YTTBTS895K?_encoding=UTF8&psc=1&pldnSite=1)
    
-   [DC 5V 1 Channel Relay Module Board Shield High/Low Level Trigger with Optocoupler](
    https://www.amazon.com/gp/product/B079FJSYGY/ref=ox_sc_act_title_11?smid=A11A70Q280RHPK&th=1)
    
    

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-808080.svg?style=for-the-badge

[License]: 5

