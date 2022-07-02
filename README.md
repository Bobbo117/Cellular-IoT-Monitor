# Cellular IoT Monitor   [![Badge License]][License]

*Transmit sensor data in remote locations without internet.*

<br>

## How It Works

This is accomplished by using an inexpensive IoT SIM card along with commonly available components to gain access to the cloud via a cellular connection.

In its simplest form, the HUB can operate as a stand-alone unit with its own sensors.

Alternatively, the HUB can collect data wirelessly from up to three separate sensor platforms.

The hub software - **AmbientHUB** - and the sensor software - **AmbientAP** - are written in C++ with the help of the **Arduino IDE**.

The **AmbientHUB** software controls an **ESP32** micro-controller connected to a temperature / humidity sensor and an optional **OLED** display over the **I2C** bus.

<br>
<br>

## Tested With

-   **Botletics** `SIM7000A` cellular IoT modem

-   **LiLLYGO** `SIM7000G` board with integrated **ESP32** Wrover

-   **Global** `SIM7000A` module

<br>
<br>

## Supported Protocols

The **AmbientHUB** software can poll the **AmbientAP** sensors, if there are any, using the listed protocols.

-  `http` where each sensor is configured <br>
    as a server which can be polled by the <br>
    hub using a **GET** command.

-   `ESP-Now` a lighter footprint protocol by <br>
    which the sensors push the data to the <br>
    hub based on its MAC address.

<br>
<br>

## Formatting Options

The **AmbientHUB** software formats the sensor values for hourly transmission via the **Hologram** IoT cellular platform to the listed website options.

### Supported Services

-   `IFTTT.com` for transmission to up to <br>
    20 email accounts plus Google sheets

-   `Dweet.io` which simply saves the <br>
    messages in scratchpad format

<br>

### Planned

- **Google Sheets** directly without **IFTTT**

- **MQTT** to **TBD** destination

- **ThingsBoard**

<br>
<br>

## Threshold Alert

The **AmbientHUB** software can be configured to provide<br>
**IFTTT** and immediate alert should a sensor exceed a predetermined threshold.

<br>

### Builtin Modules

-   An ESP32 or a D1 Mini 8266 micro-controller

-   Either a `BME-280` temperature / humidity / pressure <br>
    sensor or a `DHTxx` temperature / humidity sensor

-   An optional `SSD1306` OLED display over the **I2C** bus

<br>

### Planned

-   door / window open / closed sensors

-   `0 - 99%` illumination sensors

-   flood sensors

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-808080.svg?style=for-the-badge

[License]: #

