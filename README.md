# Cellular IoT Monitor   [![License]]

*Transmit sensor data in remote locations without internet.*

<br>

## How It Works

An inexpensive IoT SIM card gains access to the cloud via a cellular connection.

The HUB can operate as a stand-alone unit with its own sensors.

Alternatively, the HUB can collect data wirelessly from up to three separate local sensor platforms.

The hub software - **AmbientHUB** - and 
the sensor software - **AmbientAP** - 
are written in C++ with the using the **Arduino IDE**.

The **AmbientHUB** software controls an **ESP32**
micro-controller connected with onboard sensors 
and an optional **OLED** display.

<br>
<br>

## Tested With

-   **Botletics** `SIM7000A` cellular modem Arduino shield 
    https://github.com/botletics/SIM7000-LTE-Shield/wiki

-   **LiLLYGO** `SIM7000G` board with 
    integrated **ESP32** Wrover
    http://www.lilygo.cn/prod_view.aspx?TypeId=50033&Id=1246&FId=t3:50033:3

-   **and-global** `SIM7000A` module
    https://www.and-global.com/

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

### Supported Services

-    https://io.adafruit.com/ MQTT for dashboard and redundant IFTTT webhook route

-    https://ifttt.com/ for notifying to up to 20 email accounts plus Google sheets

-    https://dweet.io/ which simply saves the messages in scratchpad format

<br>

### Planned

- **Google Sheets** directly without **IFTTT**
    Anyone know how to do this?

- **ThingsBoard**

- **Motion detection**

- Remote **soil** sensing/**lawn sprinkler** control
 
- Smart **camera**

<br>
<br>

## Threshold Alert

The **AmbientHUB** software can be configured to provide <br>
immediate alerts when sensors cross adaptive thresholds.

<br>

### Builtin Modules

-   An ESP32 or a D1 Mini 8266 micro-controller
    https://www.amazon.com/MELIFE-Development-Dual-Mode-Microcontroller-Integrated/dp/B07Q576VWZ?th=1

-   A `BME-280`, an MCP9808, or a `DHTxx` temperature / humidity sensor

-   An optional `SSD1306` OLED display 

    https://www.amazon.com/gp/product/B07W1PT6VK

<br>

### Added July 2022

-   door / window sensors
    https://www.amazon.com/dp/B09BJLRK4S/

-   illumination sensors
    https://www.amazon.com/eBoot-Photoresistor-Sensitive-Resistor-Dependent/dp/B01N7V536K/

-   flood sensors
    https://www.amazon.de/-/en/Sensor-Moisture-Splash-Arduino-Raspberry/dp/B01MRIBI2M

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-808080.svg?style=for-the-badge

[License]: 5

