# ESP32 MQTT Pentair Pool Controller
* Send MQTT commands to your pool to control
  * Pool pump
  * Spa
  * Lights
  * Features

* Easily integrated with HomeAssistant

![image](https://github.com/michaelusner/esp32_pentair_controller/blob/master/images/esp32_pentair_controller.jpg?raw=true)

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
TODO

## MQTT Server
TODO

## Home Assistant
TODO
