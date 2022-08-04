# Cellular IoT Monitor   [![Badge License]][License]

*Transmit sensor data in remote locations without internet.*

<br>

## How It Works

An inexpensive IoT SIM card along with commonly available components gain access to the cloud via a cellular connection.

The HUB can operate as a stand-alone unit with its own sensors.

Alternatively, the HUB can collect data wirelessly from up to three separate local sensor platforms.

The hub software - **AmbientHUB** - and the sensor software - **AmbientAP** - are written in C++ with the help of the **Arduino IDE**.

The **AmbientHUB** software controls an **ESP32** micro-controller connected to a temperature / humidity sensor and an optional **OLED** display.

<br>
<br>

## Tested With

-   **Botletics** `SIM7000A` cellular modem Arduino shield

-   **LiLLYGO** `SIM7000G` board with integrated **ESP32** Wrover

-   **Global** `SIM7000A` module

<br>
<br>

## Supported Protocols

The **AmbientHUB** software can receive the **AmbientAP** sensor data using the listed protocols.

-  `http` where each sensor is configured as a server which can be polled by the hub using a **GET** command.

-   `ESP-Now` a lighter footprint protocol by which the sensors push the data to the hub based on its MAC address.

<br>
<br>

## Formatting Options

The **AmbientHUB** software collects the sensor values for hourly transmission via the **Hologram** IoT cellular platform to the listed website services.

### Supported Services

-    'Adafruit-io.com' MQTT for graphic display and redundant IFTTT webhook route

-   `IFTTT.com` for transmission to up to 20 email accounts plus Google sheets

-   `Dweet.io` which simply saves the messages in scratchpad format

<br>

### Planned

- **Google Sheets** directly without **IFTTT**

- **ThingsBoard**

<br>
<br>

## Threshold Alert

The **AmbientHUB** software can be configured to provide <br>
immediate alerts should a sensor exceed a predetermined threshold.

<br>

### Builtin Modules

-   An ESP32 or a D1 Mini 8266 micro-controller

-   A `BME-280`, an MCP9808, or a `DHTxx` temperature / humidity sensor

-   An optional `SSD1306` OLED display over the **I2C** bus

<br>

### Added July 2022

-   door / window sensors

-   illumination sensors

-   flood sensors

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-808080.svg?style=for-the-badge

[License]: #

