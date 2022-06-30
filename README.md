                                                               # Cellular IoT Monitor

The purpose of this project is to transmit sensor readings from a remote location without requiring internet service.  This is accomplished by using an inexpensive ioT SIM card along with commonly available components to gain access to the cloud via a cellular connection. In its simplest form, the HUB can operate as a stand-alone unit with its own sensors. Alternatively, the HUB can collect data wirelessly from up to three separate sensor platforms.  The hub software, AmbientHUB, and the sensor software, AmbientAP, and are written in C++ on an Arduino IDE.

The AmbientHUB software controls an ESP32 microcontroller connected to a temperature/humidity sensor and an optional OLED display over the I2C bus. It has been tested with the following cellular modems:
        1. Botletics SIM7000A cellular IoT modem;
        2. LiLLYGO SIM7000G board with integrated Esp32 Wrover;
        3. and-global SIM7000A module.  

The AmbientHUB software can poll the AmbientAP sensors, if there are any, using the following protocols:
        1. http, where each sensor is configured as a server which can be polled by the hub using a GET command;
        2. ESP-Now, a lighter footprint protocol by which the sensors push the data to the hub based on its MAC address.


The AmbientHUB software formats the sensor values for hourly transmission via the Hologram IoT cellular platform to the following website options
        1. to IFTTT.com for transmission to up to 20 email accounts plus Google sheets;
        2. to Dweet.io which simply saves the messages in scratchpad form;
        3. to ThingsBoard (future plans)
        4. to Google Sheets directly without IFTTT (future plans)
        5. via MQTT to TBD destination (future plans)

Additionally the AmbientHUB software can be configured to provide ifttt and immediate alert should a sensor exceed a predetermented threshhold.

The AmbientAP sensor platform software accomodates the following:
        1. an ESP32 or a D1 Mini 8266 microcontroller; 
        2. a BME-280 temperature/humidity/pressure sensor or a DHTxx temperature/humidity sensor;
        3. an (optional) SSD1306 OLED display over the I2C bus;
        4. a 0-99% illumination sensor (future plans);
        5. a flood sensor (future plans);
        6. a door/window open/closed sensor (future plans);
              
