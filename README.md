# Arduino esp8266 home weather station

Clock with auto backlight, NTP time-sync, measuring CO2, humidity, temperature, pressure and Blynk synchronization.

## Components

* CO2 Sensor MH-Z19
* ESP8266 (NodeMCU ESP12+ based)
* ST7920 128x64 LCD display
* Humidity/Pressure/Temperature BME280
* Photoresistor 5528 LDR
* Buttons, resistors, wires, enclosure

## Libraries

* [Arduino-esp8266] (https://github.com/esp8266/Arduino)
* [Blynk](https://github.com/blynkkk/blynk-library)
* [u8g2](https://github.com/olikraus/u8g2)
* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [WiFiManager](https://github.com/tzapu/WiFiManager)
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
* [AsyncPing](https://github.com/akaJes/AsyncPing)
* [SimpleTimer](http://playground.arduino.cc/Code/SimpleTimer)
* [TimeLibrary](https://github.com/PaulStoffregen/Time)

## Wiring:
```
ST7920
RS - D8
R/W - D7
E - D6
backlight - D3

mh-z19
RX - D4
TX - D5

BME-280
SDA - D1
SCL - D2
VCC - 3V3
```
## Known issues:

* Temperature measurements seems to be higher due to heat from ESP8266
* ST7920 plate need to be cutted to fit enclosure

# Russian:
Часы с синхронизацией времени по WI-FI, функцией измерения CO2, температуры, влажности, давления. Также есть автоматическое изменение подсветки в зависимости от освещения.
Часы синхронизируются по NTP протоколу через WI-FI. Данные датчиков передаются на сервер Blynk.

## Подключение:
```
дисплей 12864 
RS - D8
R/W - D7
E - D6
катод подсветки - D3

датчик mh-z19
RX - D4
TX - D5

датчик BME-280
SDA - D1
SCL - D2
VCC - 3V3
```