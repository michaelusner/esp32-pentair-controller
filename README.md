# ESP32 WIFI Pentair Pool Controller
* Send MQTT commands to your pool to control
  * Pool pump
  * Spa
  * Lights
  * Features

* Easily integrated with HomeAssistant
* FreeRTOS based

![image](https://github.com/michaelusner/esp32_pentair_controller/blob/master/images/overview.png?raw=true)

## Hardware
For this project, I used the following:
 * ESP32-DevKitC which I bought from [Amazon](https://www.amazon.com/gp/product/B0811LGWY2/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

 ![image](https://github.com/michaelusner/esp32_pentair_controller/blob/master/images/esp32.jpg?raw=true)
 
 * MAX485 Development Board also from [Amazon](https://www.amazon.com/gp/product/B014QNI0BC/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

 ![image](https://github.com/michaelusner/esp32_pentair_controller/blob/master/images/max485.jpg?raw=true)

## Wiring
 Here's the wiring diagram:
 
![image](https://github.com/michaelusner/esp32_pentair_controller/blob/master/images/wiring.jpg?raw=true)

 MAX485 Input:
 A - Pentair RS485
 B - Pentair RS485
 DE - RE Jumper
 
 | ESP32 Pin | MAX485 Pin |
 | --------- | ---------- |
 | 5v | VCC |
 | GND | GND |
 | 4 | DE |
 | 16 | R0 |
 | 17 | D1 |
 
## Configuration
This project uses the [esp32-wifi-manager](https://github.com/tonyp7/esp32-wifi-manager) to connect your ESP32 to your wifi network.
Please see this project for details on connecting to wifi after flashing.

## MQTT Server
This project relies on an MQTT server installed somewhere on your home network.
There are several options available (I chose Mosquitto on my Synology NAS which works fine).  Installation and configuration of the server will be up to you.  The ESP32 will need user/password access on the same subnet.

After installing and configuring MQTT, you will need to add the required hostname/ip/user/password to the ESP32 code before flashing.

TODO: This will be improved by a options available via a HTTP page on the ESP32.  

## Home Assistant
Once you setup your MQTT server, you will need to [configure Home Assistant](https://www.home-assistant.io/integrations/mqtt/) to connect to your MQTT server.

The esp32-pentair-controller uses Home Assistant MQTT auto-discovery to populate the controls.  They should appear automatically following the configuration and discovery process.

## TODO
Configure MQTT server settings via web interface
