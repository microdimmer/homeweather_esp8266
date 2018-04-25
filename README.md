# Arduino esp8266 home weather station

Clock with auto backlight, NTP time-sync, measuring CO2, humidity, temperature, atmospheric pressure and Blynk or Cayenne synchronization.

## Components

* CO2 Sensor MH-Z19
* ESP8266 (NodeMCU ESP12+ based)
* ST7920 128x64 LCD display
* Humidity/Pressure/Temperature BME280
* Photoresistor 5528 LDR
* Buttons, resistors, wires, enclosure

## Photos:
![PHOTO1](https://github.com/microdimmer/homeweather_esp8266/blob/master/readme/front.jpg)
![PHOTO2](https://github.com/microdimmer/homeweather_esp8266/blob/master/readme/disassembled.jpg)

## Blynk and Cayenne:
[Blynk docs](http://docs.blynk.cc/)

<img src="https://github.com/microdimmer/homeweather_esp8266/blob/master/readme/blynk.png" height="360">

[Cayenne docs](https://mydevices.com/cayenne/features/api/)

<img src="https://github.com/microdimmer/homeweather_esp8266/blob/master/readme/cayenne.png" height="360">

Data is bounded to this values:
* v1 - temperature;
* v2 - humidity;
* v4 - atmospheric pressure;
* v5 - CO2 sensor MH-Z19;
* v7 - light sensor;

## Libraries

* [Arduino-esp8266](https://github.com/esp8266/Arduino)
* [Blynk](https://github.com/blynkkk/blynk-library)
* [Cayenne](https://github.com/myDevicesIoT/Cayenne-MQTT-Arduino)
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

## Scheme:
![СХЕМА](https://github.com/microdimmer/homeweather_esp8266/blob/master/readme/scheme.png)

## Known issues:

* Heap leak with Blynk lybrary
* Temperature measurements seems to be higher due to heat from ESP8266
* ST7920 plate need to be cutted to fit enclosure
* Can't connect to my old ASUS WL-500gP V2

# Russian:
Часы с синхронизацией времени по WI-FI, функцией измерения CO2, температуры, влажности, давления. Также есть автоматическое изменение подсветки в зависимости от освещения.
Часы синхронизируются по NTP протоколу через WI-FI. Данные датчиков передаются на сервер Blynk или Cayenne.

## Компоненты

* Датчик CO2 MH-Z19
* NodeMCU v2
* ST7920 128x64 LCD-дисплей
* датчик давления/влажности/температуры BME280
* фоторезистор 5528 LDR
* кнопки, резисторы, провода, корпус

## Библиотеки

* [Arduino-esp8266](https://github.com/esp8266/Arduino)
* [Blynk](https://github.com/blynkkk/blynk-library)
* [Cayenne](https://github.com/myDevicesIoT/Cayenne-MQTT-Arduino
* [u8g2](https://github.com/olikraus/u8g2)
* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [WiFiManager](https://github.com/tzapu/WiFiManager)
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
* [AsyncPing](https://github.com/akaJes/AsyncPing)
* [SimpleTimer](http://playground.arduino.cc/Code/SimpleTimer)
* [TimeLibrary](https://github.com/PaulStoffregen/Time)

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
## Известные проблемы:

* Утечка памяти с библиотекой Blynk
* ESP8266 греется и вносит погрешность в показания температуры, пока изолировал датчик BME-280 вспененным полиэтиленом
* Пришлось немного подрезать плату дисплея ST7920 сверху, не входила по высоте
* Не удалось подключить к роутеру ASUS WL-500gP V2 (видимо нужно разбираться с типом WI-FI сети на роутере)