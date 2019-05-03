#include <FS.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h> // Wifi Manager https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <DNSServer.h>
#include <ESP8266HTTPClient.h> // HTTP requests
#include <WiFiUdp.h> //for NTP
#include <AsyncPing.h> //async pinger
#include <TimeLib.h> //timekeeping
#include <ESP8266httpUpdate.h> //OTA updates
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <SimpleTimer.h> // Handy timers
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>

//Preferences
// #define DEBUG //uncomment for debug messages
// #define BLYNK //uncomment for use Blynk, otherwise use Cayenne data aggregator

#ifdef BLYNK
#include <BlynkSimpleEsp8266.h> //Blynk
#else
#include <CayenneMQTTESP8266.h> //Cayenne
#endif

const uint16_t WIFI_TIMEOUT = 180;
const char * SSID = "YourHomeWeather"; // Network credentials
//String pass {"YHWBopka"}; // Wi-Fi AP password
// GPIO Defines
const uint8_t I2C_SDA = 5; // D1, SDA pin, GPIO5 for BME280
const uint8_t I2C_SCL = 4; // D2, SCL pin, GPIO4 for BME280
const uint8_t PWM_PIN = 0; //D3, GPIO0, for ST7920
const uint8_t SCLK_PIN = 12; // D6, E pin, GPIO12, for ST7920 acts like CLK (clock) input pin
const uint8_t RW_PIN = 13; // D7, R/W pin, GPIO13, for ST7920 acts like DATA pin
const uint8_t RS_PIN = 15; // D8, RS pin, GPIO15, for ST7920 acts like CS (Chip Select) pin
const uint8_t TX_PIN = 2; //D4, RX pin, GPIO2, for mz-h19
const uint8_t RX_PIN = 14; //D5, TX pin, GPIO14, for mz-h19

//display ST7920
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0,/*display-clock E,SCLK;esp-GPIO12,D6*/SCLK_PIN,/*display-data R/W;esp-GPIO13,D7*/RW_PIN,/*display-RS;esp-GPIO15,D8*/RS_PIN);
// Humidity/Temperature/Pressure/CO2
Adafruit_BME280 bme;
SoftwareSerial swSer(RX_PIN, TX_PIN, false, 256);// CO2 SERIAL
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char response[7];
#ifdef BLYNK
// Blynk token
char blynk_token[33] {"7ca0a9293079453499aa5453883510cf"};
char blynk_server[64] {"blynk-cloud.com"};
const uint16_t blynk_port {8442};
#else
// Cyenne credentials
char cayenne_username[] = "edc8aad0-d07d-11e7-8123-07faebe02555";
char cayenne_password[] = "3924eb177886b6280ee0b13b0045d2a5a7e66216";
char cayenne_clientID[] = "2dd4dd50-479e-11e8-bf56-db14f0c2b326";
#endif
// Device Id
char device_id[17] = "Home Weather";
const char fw_ver[17] = "0.1.21";
// Handy timer
SimpleTimer timer;
// Setup Wifi connection
WiFiManager wifiManager;

//sensors data
int16_t t { -100};
int16_t p { -1};
int16_t h { -1};
int co2 {0};
float tf {0};
float pf {0};
float hf {0};
uint16_t light;
uint16_t adc_data;
uint32_t uptime = 0;
//buttons
const uint8_t NUM_KEYS = 3;
int adc_key_val[NUM_KEYS] = {100, 200, 360};
uint32_t lastDebounceTime = 0;
uint16_t debounceTime = 250;
uint32_t lastIdleTimeMenu = 0;
uint32_t idleTimeMenu = 180000; // menu idle time 3 min
// Math data for pressure calculating, see http://bit.ly/1EXW1I9 http://bit.ly/1DIbvyj
const uint8_t P_LEN = 4;
float p_array[P_LEN];
float delta;
//flags
volatile bool connectedInetFlag = false; //flag if connected
bool timeSyncFlag = false;
bool cloudSyncFlag = false;
#ifndef BLYNK
bool CayenneConnectedFlag = false;
#endif
bool shouldSaveConfig = false; //flag for saving data if connectio estalished
//menu
bool timeSetFlag = false; //TODO del flags
bool timeSetMinFlag = false;
bool timeSetHourFlag = false;
bool timeSetTimeZoneFlag = false;
bool timeSetDayFlag = false;
bool timeSetMonthFlag = false;
bool timeSetYearFlag = false;
bool backlightSetFlag = false;
bool backlightSetLEDFlag = false;
bool backlightSetMinFlag = false;
bool backlightSetMaxFlag = false;
bool backlightSetThresoldFlag = false;

bool menuFlag = false;
int8_t curMenuItem = 3;
int8_t menuItemsCount = 4;

int32_t wifiRSSI = 0;

const uint16_t pwm_light[14] = {0, 0, 2, 4, 8, 16, 32, 64, 96, 128, 192, 256, 320, 384};
int8_t light_mode = 0;
int8_t light_auto_min = 2; //default min backlight
int8_t light_auto_max = 11; //default max backlight
uint16_t light_auto_thresold = 1023; //default thresold
uint32_t lightLastHysteresisTimeMin = 0;
uint32_t lightLastHysteresisTimeMax = 0;
uint16_t lightHysteresisTime = 2500; //2.5 secs hysteresis for light sensor

AsyncPing aping;

WiFiUDP Udp;
const char* ntpServerName = "0.ru.pool.ntp.org"; //NTP server (ntp2.stratum2.ru,ntp1.vniiftri.ru)
IPAddress ntpServerIP;
#ifdef BLYNK
IPAddress pingServerIP(139, 59, 206, 133); //blynk serv for ping
#endif
unsigned int localPort = 4567;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
const uint16_t timeZonesArr[11] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int8_t timeZone = 3;         // default timezone UTC +5

String timestring = "";//TODO

#include "graphics.h"

#ifdef DEBUG
#define PRINTLNF(s)   { Serial.println(F(s)); }
#define PRINTLN(s,v)  { Serial.print(F(s)); Serial.println(v); }
#else
#define PRINTLNF(s)
#define PRINTLN(s,v)
#endif


time_t getNtpTime() {
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  PRINTLNF("Transmit NTP Request");
  #ifdef BLYNK
  ntpServerIP = pingServerIP;
  #else
  WiFi.hostByName(ntpServerName, ntpServerIP); //get IP address
  #endif
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      PRINTLNF("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      timeSyncFlag = 1;
      return secsSince1900 - 2208988800UL + timeZonesArr[timeZone] * SECS_PER_HOUR;
    }
  }
  PRINTLNF("No NTP Response :-(");
  timeSyncFlag = 0;
  return 0; // return 0 if unable to get the time
}

void sendNTPpacket(IPAddress &address) { // send an NTP request to the time server at the given address
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void syncTime() {
  setTime(getNtpTime());
  #ifdef DEBUG
    timeStatus_t t_status = timeStatus();
    if (t_status == timeNotSet) {
      PRINTLNF("Time never been synced");
    }  
    else if (t_status == timeNeedsSync) {
      PRINTLNF("Time is need to sync");
    }  
    else if (t_status == timeSet) {
      PRINTLNF("Time is synced");
    }
  #endif    
}

String weekdayRus(byte weekday) {
  switch (weekday) {
    case 2:
      return "PON";
    case 3:
      return "VTR";
    case 4:
      return "SRD";
    case 5:
      return "CTV";
    case 6:
      return "PTN";
    case 7:
      return "SYB";
    case 1:
      return "VSK";
  }
  return "0";
}

void drawSignalQuality(uint8_t x, uint8_t y) {
  if (!connectedInetFlag) {
    u8g2.drawLine(x, y, x + 2, y + 2);
    u8g2.drawLine(x, y + 2, x + 2, y);
  }
  else {
    u8g2.drawLine(x, y + 2, x + 2, y);
    u8g2.drawLine(x, y, x, y + 2);
  }
  y = y + 6;
  if (wifiRSSI >= -90)
    u8g2.drawFrame(x, y, x + 2, y - 4);
  if (wifiRSSI >= -85)
    u8g2.drawFrame(x + 3, y - 2, x + 2, y - 2);
  if (wifiRSSI >= -75)
    u8g2.drawFrame(x + 6, y - 4, x + 2, y);
  if (wifiRSSI >= -65)
    u8g2.drawFrame(x + 9, y - 6, x + 2, y + 2);
}

const char *GetStringLine(uint8_t line_idx, const char *str ) { //Assumes strings, separated by '\n' in "str". Returns the string at index "line_idx". First strng has line_idx = 0
  char e;
  uint8_t line_cnt = 1;

  if ( line_idx == 0 )
    return str;

  for (;;)
  {
    e = *str;
    if ( e == '\0' )
      break;
    str++;
    if ( e == '\n' )
    {
      if ( line_cnt == line_idx )
        return str;
      line_cnt++;
    }
  }
  return NULL;  /* line not found */
}

const char* printDigits(uint16_t digits, bool blinking = false, bool leadingZero = true) { //prints preceding colon and leading 0, blinking
  timestring = String(digits);
  if (blinking && ((millis() / 500 % 2) == 0))
    return "";
  if (digits < 10 && leadingZero)
    timestring = "0" + String(digits);

  return timestring.c_str();
}

void drawMainScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(custom_font_14);
  if (timeSetFlag) {
    u8g2.drawStr(15, 14, printDigits(hour(), timeSetHourFlag));
    u8g2.drawStr(35, 14, ":");
    u8g2.drawStr(39, 14, printDigits(minute(), timeSetMinFlag));
    u8g2.setFont(custom_font_7);
    u8g2.drawStr(62, 10, "utc");
    u8g2.setFont(custom_font_14);
    u8g2.drawStr(76, 14, "+");
    u8g2.drawStr(84, 14, printDigits(timeZonesArr[timeZone], timeSetTimeZoneFlag, false));
    u8g2.drawStr(15, 30, printDigits(day(), timeSetDayFlag));
    u8g2.drawStr(35, 30, ".");
    u8g2.drawStr(39, 30, printDigits(month(), timeSetMonthFlag));
    u8g2.drawStr(59, 30, ".");
    u8g2.drawStr(64, 30, printDigits(year(), timeSetYearFlag));
  }
  else if (backlightSetFlag) {
    u8g2.setFont(custom_font_7);
    u8g2.drawStr(17, 14, "led:");
    if (light_mode == 0) {
      u8g2.drawStr(62, 14, "min:");
      u8g2.drawStr(13, 24, "thre");
      u8g2.drawStr(13, 30, "sold:");
      u8g2.drawStr(62, 30, "max:");
      u8g2.setFont(u8g2_font_7x13_mf);
      u8g2.drawStr(32, 14, "auto");
      u8g2.setFont(custom_font_14);
      u8g2.drawStr(32, 30, printDigits(light_auto_thresold / 10, backlightSetThresoldFlag, false));
      u8g2.drawStr(77, 14, printDigits(light_auto_min, backlightSetMinFlag, false));
      u8g2.drawStr(77, 30, printDigits(light_auto_max, backlightSetMaxFlag, false));
    }
    else if (light_mode == 1) {
      u8g2.setFont(u8g2_font_7x13_mf);
      u8g2.drawStr(32, 14, "off");
    }
    else {
      u8g2.setFont(custom_font_14);
      u8g2.drawStr(32, 14, printDigits(light_mode, backlightSetFlag, false));
    }
  }
  else { //draw time
    u8g2.setFont(custom_font30);
    u8g2.drawStr(15, 30 , printDigits(hour()));
    if ((millis() / 1000) % 2) u8g2.drawStr(57, 30 , ":");
    u8g2.drawStr(64, 30 , printDigits(minute()));
  }

  if (WiFi.status() == WL_CONNECTED) {
    drawSignalQuality(0, 0);
  }
  if (timeSyncFlag) {
    u8g2.drawXBMP(0, 10, 11, 9, clock_bitmap);
  }
  if (cloudSyncFlag) {
    u8g2.drawXBMP(0, 21, 11, 9, sync_bitmap);
  }

  u8g2.setFont(rus_font);
  u8g2.drawStr(107, 28, weekdayRus(weekday()).c_str());

  u8g2.setFont(custom_font_14);
  u8g2.drawStr(108, 14 , printDigits(day()));

  //CO2
  String co2String = String(co2);
  u8g2.drawXBMP(60, 52, 15, 12, co_bitmap);
  u8g2.drawStr(117 - (co2String.length() * 10), 64 , co2String.c_str());
  // Temp, Humidity, Pressure
  String tfString = String(tf, 1);
  u8g2.drawXBMP(2, 50, 10, 14, temp_bitmap);
  u8g2.drawStr(55 - (tfString.length() * 10), 64, tfString.c_str());

  String hfString = String(hf, 1);
  u8g2.drawXBMP(2, 33, 10, 14, humid_bitmap);
  u8g2.drawStr(55 - (hfString.length() * 10), 47, hfString.c_str());

  u8g2.drawXBMP(60, 36, 14, 12, p_bitmap);

  if (delta > 2) {
    u8g2.drawGlyph(76, 47, 30); //big arrow up
  }
  else if (delta > 1) {
    u8g2.drawGlyph(76, 47, 28); //small arrow up
  }
  else if (delta < -2) {
    u8g2.drawGlyph(76, 47, 31); //big arrow down
  }
  else if (delta < -1) {
    u8g2.drawGlyph(76, 47, 29); //small arrow down
  }

  u8g2.drawStr(87, 47, String(p).c_str());

  u8g2.setFont(custom_font_7);
  u8g2.drawGlyph(48, 64, 0xb0); //degree sign
  u8g2.drawStr(51, 64, "C");
  u8g2.drawGlyph(49, 48, 0x25); //percent
  u8g2.drawStr(117, 64 , "y.e.");
  u8g2.drawStr(117, 37 , "mm");
  u8g2.drawStr(117, 42 , "pt.");
  u8g2.drawStr(117, 47 , "ct.");

  u8g2.sendBuffer();
}

void drawMenu(const char *title, uint8_t start_pos, const char *line) {
  if ((millis() - lastIdleTimeMenu) > idleTimeMenu) {
    menuFlag = false;
    return;
  }

  u8g2.clearBuffer();
  byte x {0}; byte y {0};

  u8g2.setFont(u8g2_font_7x13_mf);

  x = (128 - u8g2.getStrWidth(title)) / 2;
  y = u8g2.getAscent() - u8g2.getDescent() - 2;
  u8g2.drawStr(x, y, title);
  y++;
  u8g2.drawHLine(0, y, 127);

  for (byte  i = 0; i < menuItemsCount; i++) {
    const char *msg = GetStringLine(i, line);
    if (i == start_pos) {
      u8g2.drawBox(0, y + 2, 127, u8g2.getAscent() - u8g2.getDescent() + 2);
      u8g2.setDrawColor(0);
    }
    else
      u8g2.setDrawColor(1);
    x = (128 - u8g2.getStrWidth(msg)) / 2;
    y = y + 2 + u8g2.getAscent() - u8g2.getDescent();
    u8g2.drawStr(x, y, msg);
  }
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

void drawBoot(String msg = "loading...") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13_mf);
  byte x {0}; byte y {0};
  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = 32 + u8g2.getAscent() / 2;
  u8g2.drawStr(x, y, msg.c_str());
  u8g2.sendBuffer();
}

void drawConnectionDetails(String ssid, String mins, String url) {
  String msg {""};
  u8g2.clearBuffer();
  byte x {0}; byte y {0};

  msg = "Connect to WiFi:";
  u8g2.setFont(u8g2_font_7x13_mf);
  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, msg.c_str());

  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = y + 1 + u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, ssid.c_str());

  msg = "waiting for " + mins;
  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = y + 1 + u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, msg.c_str());

  msg = "Open browser:";
  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = y + 1 + u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, msg.c_str());

  x = (128 - u8g2.getStrWidth(url.c_str())) / 2;
  y = y + 1 + u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, url.c_str());

  u8g2.sendBuffer();
}

void saveConfigCallback() { //callback notifying when need to save config
  PRINTLNF("Should save config");
  shouldSaveConfig = true;
}

void factoryReset() {
  PRINTLNF("Resetting to factory settings");
  wifiManager.resetSettings();
  SPIFFS.format();
  ESP.reset();
}

void readCO2() {
  bool header_found {false};

  swSer.write(cmd, 9);
  memset(response, 0, 7);

  // Looking for packet start
  while (swSer.available() && (!header_found)) {
    if (swSer.read() == 0xff ) {
      if (swSer.read() == 0x86 ) header_found = true;
    }
  }

  if (header_found) {
    swSer.readBytes(response, 7);

    byte crc = 0x86;
    for (byte i = 0; i < 6; i++) {
      crc += response[i];
    }
    crc = 0xff - crc;
    crc++;

    if ( !(response[6] == crc) ) {
      PRINTLN("CO2: CRC error: ", String(crc) + " / " + String(response[6]));
    } 
    else {
      unsigned int responseHigh = (unsigned int) response[0];
      unsigned int responseLow = (unsigned int) response[1];
      unsigned int ppm = (256 * responseHigh) + responseLow;
      co2 = ppm;
      PRINTLN("CO2:",co2);
    }
  } else {
    PRINTLNF("CO2: Header not found");
  }
}

void readMeasurements() {
  tf = bme.readTemperature(); //Temperature
  t = static_cast<int>(tf);
  hf = bme.readHumidity(); //Humidity
  h = static_cast<int>(hf);
  pf = bme.readPressure() * 760.0 / 101325; //Pressure (in mmHg)
  p = static_cast<int>(floor(pf + 0.5));
  readCO2();// CO2
  if (WiFi.status() == WL_CONNECTED) {
    wifiRSSI = WiFi.RSSI(); //WiFi signal strength (RSSI)
    PRINTLN("Wi-Fi RSSI: ", String(wifiRSSI) + "dBm"); //Write to debug console
  }  
  if (adc_data > 370)
    light = adc_data;
  uptime+=10;  //uptime
  
  PRINTLN("H: ", String(hf) + "%"); //Write to debug console
  PRINTLN("T: ", String(tf) + "C");
  PRINTLN("Pf: ", String(pf, 1) + "mmHg");
  PRINTLN("CO2: ", String(co2) + "ppm");
  PRINTLN("Light sensor: ", String(light));
  PRINTLN("Free Heap: ",ESP.getFreeHeap());
}

void read_p_arr() {
  for (uint8_t i = 0; i < P_LEN - 1; i++) {
    p_array[i] = p_array[i + 1];
  }
  p_array[P_LEN - 1] = bme.readPressure() * 760.0 / 101325;

  float sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;
  for (uint8_t i = 0; i < P_LEN; i++) {
    sumX += i + 1;
    sumY += p_array[i];
    sumX2 += (i + 1) * (i + 1);
    sumXY += (i + 1) * p_array[i];
  }
  float a = 0;
  a = sumX * sumY;
  a = a - P_LEN * sumXY;
  a = a / (sumX * sumX - P_LEN * sumX2);
  delta = a * 3 ; // delta of changing pressure for 3 hours
  PRINTLN("Pressure delta ", delta);
}

void sendMeasurements() {   // send to server
  if (connectedInetFlag) {
    if (timeStatus() == timeNotSet)
      syncTime();
  #ifdef BLYNK    
    if (connectBlynk()) {
      Blynk.virtualWrite(V1, tf);
      Blynk.virtualWrite(V2, h);
      Blynk.virtualWrite(V4, p);
      Blynk.virtualWrite(V5, co2);
      Blynk.virtualWrite(V6, delta); //pressure delta
      Blynk.virtualWrite(V7, light); //light sensor
      Blynk.virtualWrite(V8, millis());
      Blynk.virtualWrite(V9, ESP.getFreeHeap());
      Blynk.run();
  #else    
    if (CayenneConnectedFlag) {
      Cayenne.virtualWrite(1, tf);
      Cayenne.virtualWrite(2, hf);
      Cayenne.virtualWrite(4, pf);
      Cayenne.virtualWrite(5, co2);
      Cayenne.virtualWrite(6, delta); //pressure delta
      Cayenne.virtualWrite(7, light); //light sensor
      Cayenne.virtualWrite(8, numberOfHours(uptime)); //uptime hours
      Cayenne.virtualWrite(9, ESP.getFreeHeap());
      Cayenne.loop();
  #endif  
      cloudSyncFlag = 1;
      PRINTLNF("Send to data server");
    }
    else {
      cloudSyncFlag = 0;
      PRINTLNF("Send data to server fails!");
    }
  }
  else
    cloudSyncFlag = 0;
}

#ifdef BLYNK 
bool connectBlynk() {
  if (!Blynk.connected())
    return Blynk.connect();
  return true;
}
#endif

void asyncPing() {
  if (WiFi.status() == WL_CONNECTED)
    aping.begin(ntpServerIP, 5, 1500);
  else
    connectedInetFlag = false;  
}

bool loadConfig() {
  PRINTLNF("Load config...");
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    PRINTLNF("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    PRINTLNF("Config file size is too large");
    return false;
  }

  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<200> jsonBuffer;
//  JsonObject json = jsonBuffer.parseObject(buf.get());
  auto error = deserializeJson(jsonBuffer, buf.get());
  if (error) {
    PRINTLNF("Failed to parse config file");
    return false;
  }

  timeZone = jsonBuffer["timeZone"];  // load parameters
  light_auto_thresold = jsonBuffer["light_auto_thresold"];
  light_auto_min = jsonBuffer["light_auto_min"];
  light_auto_max = jsonBuffer["light_auto_max"];
  return true;
}

bool writeConfig() {
  PRINTLNF("Load config...");
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
    return false;

  DynamicJsonDocument jsonBuffer(1024);
//  JsonObject json = jsonBuffer.createObject();
  jsonBuffer["timeZone"] = timeZone;
  jsonBuffer["light_auto_thresold"] = light_auto_thresold;
  jsonBuffer["light_auto_min"] = light_auto_min;
  jsonBuffer["light_auto_max"] = light_auto_max;

//  jsonBuffer.printTo(configFile);
  serializeJson(jsonBuffer, configFile);
  configFile.close();
  return true;
}

bool loadConfigWiFI() {
  PRINTLNF("Load config...");
  File configFile = SPIFFS.open("/configwifi.json", "r");
  if (!configFile) {
    PRINTLNF("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    PRINTLNF("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<200> jsonBuffer;
//  JsonObject json = jsonBuffer.parseObject(buf.get());
  auto error = deserializeJson(jsonBuffer, buf.get());
  if (error) {
    PRINTLNF("Failed to parse config file");
    return false;
  }
  // Save parameters
  #ifdef BLYNK 
  strcpy(device_id, jsonBuffer["device_id"]);
  strcpy(blynk_server, jsonBuffer["blynk_server"]);
  strcpy(blynk_token, jsonBuffer["blynk_token"]);
  #else
  strcpy(cayenne_username, jsonBuffer["cayenne_username"]);
  strcpy(cayenne_password, jsonBuffer["cayenne_password"]);
  strcpy(cayenne_clientID, jsonBuffer["cayenne_clientID"]);
  #endif
  return true;
}

bool setupWiFi() {
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  // Custom parameters
  #ifdef BLYNK 
  WiFiManagerParameter custom_device_id("device_id", "Device name", device_id, 16);
  WiFiManagerParameter custom_blynk_server("blynk_server", "Blynk server", blynk_server, 64);
  WiFiManagerParameter custom_blynk_token("blynk_token", "Blynk token", blynk_token, 34);
  wifiManager.addParameter(&custom_blynk_server);
  wifiManager.addParameter(&custom_blynk_token);
  wifiManager.addParameter(&custom_device_id);
  #else
  WiFiManagerParameter custom_cayenne_username("cayenne_username", "Username", cayenne_username, 37);
  WiFiManagerParameter custom_cayenne_password("cayenne_password", "Password", cayenne_password, 41);
  WiFiManagerParameter custom_cayenne_clientID("cayenne_clientID", "Client ID", cayenne_clientID, 37);
  wifiManager.addParameter(&custom_cayenne_username);
  wifiManager.addParameter(&custom_cayenne_password);
  wifiManager.addParameter(&custom_cayenne_clientID);
  #endif
  drawConnectionDetails(SSID, String(static_cast<int>(WIFI_TIMEOUT / 60)) + " mins", "http://192.168.4.1");
  wifiManager.setTimeout(WIFI_TIMEOUT);

  if (!wifiManager.autoConnect(SSID)) {
    //  if (!wifiManager.autoConnect(ssid.c_str(), pass.c_str())) { \\ с паролем иногда не пускает, пока будем без
    PRINTLNF("failed to connect and hit timeout");
    return false;
  }

  if (shouldSaveConfig) { //save the custom parameters to FS
    PRINTLNF("saving config");
    DynamicJsonDocument  jsonBuffer(1024);
    #ifdef BLYNK 
    jsonBuffer["device_id"] = custom_device_id.getValue();
    jsonBuffer["blynk_server"] = custom_blynk_server.getValue();
    jsonBuffer["blynk_token"] = custom_blynk_token.getValue();
    #else
    jsonBuffer["cayenne_username"] = custom_cayenne_username.getValue();
    jsonBuffer["cayenne_password"] = custom_cayenne_password.getValue();
    jsonBuffer["cayenne_clientID"] = custom_cayenne_clientID.getValue();
    #endif
    File configFile = SPIFFS.open("/configwifi.json", "w");
    #ifdef DEBUG
    if (!configFile) {
      PRINTLNF("failed to open config file for writing");
    }
    serializeJson(jsonBuffer, Serial);
//    jsonBuffer.printTo(Serial);
    #endif
    serializeJson(jsonBuffer, configFile);
//    jsonBuffer.printTo(configFile);
    configFile.close();
  }

  PRINTLNF("WiFi connected"); //if you get here you have connected to the WiFi
  PRINTLN("IP address: ",WiFi.localIP());
  return true;
}

#ifdef BLYNK 
// Virtual pin update FW
BLYNK_WRITE(V22) {
  if (param.asInt() == 1) {
    PRINTLNF("Got a FW update request");

    char full_version[34] {""};
    strcat(full_version, device_id);
    strcat(full_version, "::");
    strcat(full_version, fw_ver);

    t_httpUpdate_return ret = ESPhttpUpdate.update("http://romfrom.space/get", full_version);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        PRINTLNF("[update] Update failed.");
        break;
      case HTTP_UPDATE_NO_UPDATES:
        PRINTLNF("[update] Update no Update.");
        break;
      case HTTP_UPDATE_OK:
        PRINTLNF("[update] Update ok.");
        break;
    }
  }
}
// Virtual pin reset
BLYNK_WRITE(V23) {
  factoryReset();
}
// Virtual pin PWM mode
BLYNK_WRITE(V25) {
  if (++light_mode >= sizeof(pwm_light) / sizeof(*pwm_light)) light_mode = 0;
  analogWrite(PWM_PIN, pwm_light[light_mode]);
}
#else
//cayenne connected callback
CAYENNE_CONNECTED() {
  CayenneConnectedFlag = true;
}
//cayenne disconnected callback
CAYENNE_DISCONNECTED() {
  CayenneConnectedFlag = false;
}
#endif

void adcDecode() {
  if (adc_data < 370) { // buttons decode
    if (adc_data < adc_key_val[0]) {
      if ((millis() - lastDebounceTime) > debounceTime) {
        lastDebounceTime = millis();
        buttonTwo();
      }
    }
    else if (adc_data < adc_key_val[1]) {
      if ((millis() - lastDebounceTime) > debounceTime) {
        lastDebounceTime = millis();
        buttonOne();
      }
    }
    else if (adc_data < adc_key_val[2]) {
      if ((millis() - lastDebounceTime) > debounceTime) {
        lastDebounceTime = millis();
        buttonThree();
      }
    }
  }
  else if (light_mode == 0) { //light sensor, auto backlight
    if (adc_data > light_auto_thresold) {
      if ((millis() - lightLastHysteresisTimeMin) > lightHysteresisTime) { //hysteresis logic
        lightLastHysteresisTimeMax = millis();
        analogWrite(PWM_PIN, pwm_light[light_auto_min]);
      }
    }
    else if ((millis() - lightLastHysteresisTimeMax) > lightHysteresisTime) { //hysteresis logic
      lightLastHysteresisTimeMin = millis();
      analogWrite(PWM_PIN, pwm_light[light_auto_max]);
    }
  }
}

void buttonOne() {
  PRINTLNF("button one is pressed!");
  if (menuFlag)
    if (++curMenuItem > menuItemsCount - 1) curMenuItem = 0;
  if (timeSetMinFlag) {
    if (minute() == 0)
      setTime(hour(), 59, second(), day(), month(), year());
    else
      setTime(hour(), minute() - 1, second(), day(), month(), year());
  }
  if (timeSetHourFlag) {
    if (hour() == 0)
      setTime(23, minute(), second(), day(), month(), year());
    else
      setTime(hour() - 1, minute(), second(), day(), month(), year());
  }
  if (timeSetTimeZoneFlag) {
    if (--timeZone < 0 ) timeZone = sizeof(timeZonesArr) / sizeof(*timeZonesArr) - 1;
  }
  if (timeSetDayFlag)
    setTime(hour(), minute(), second(), day() - 1, month(), year());
  if (timeSetMonthFlag) {
    if (month() == 1)
      setTime(hour(), minute(), second(), day(), 12, year());
    else
      setTime(hour(), minute(), second(), day(), month() - 1, year());
  }
  if (timeSetYearFlag)
    setTime(hour(), minute(), second(), day(), month(), year() - 1);
  if (backlightSetLEDFlag) {
    if (--light_mode < 0) light_mode = sizeof(pwm_light) / sizeof(*pwm_light) - 1;
    if (light_mode != 0)
      analogWrite(PWM_PIN, pwm_light[light_mode]);
  }
  if (backlightSetThresoldFlag) {
    light_auto_thresold = light_auto_thresold - 10;
    if (light_auto_thresold < 700) light_auto_thresold = 1023;
  }
  if (backlightSetMinFlag) {
    if (--light_auto_min < 1 ) light_auto_min = sizeof(pwm_light) / sizeof(*pwm_light);
  }
  if (backlightSetMaxFlag) {
    if (--light_auto_max < 1) light_auto_max = sizeof(pwm_light) / sizeof(*pwm_light);
  }
}

void buttonTwo() {
  PRINTLNF("button two is pressed!");
  if (menuFlag)
    if (--curMenuItem < 0) curMenuItem = menuItemsCount - 1;
  if (timeSetMinFlag) {
    if (minute() == 59)
      setTime(hour(), 0, second(), day(), month(), year());
    else
      setTime(hour(), minute() + 1, second(), day(), month(), year());
  }
  if (timeSetHourFlag) {
    if (hour() == 23)
      setTime(0, minute(), second(), day(), month(), year());
    else
      setTime(hour() + 1, minute(), second(), day(), month(), year());
  }
  if (timeSetTimeZoneFlag) {
    if (++timeZone >= sizeof(timeZonesArr) / sizeof(*timeZonesArr)) timeZone = 0;
  }
  if (timeSetDayFlag)
    setTime(hour(), minute(), second(), day() + 1, month(), year());
  if (timeSetMonthFlag) {
    if (month() == 12)
      setTime(hour(), minute(), second(), day(), 1, year());
    else
      setTime(hour(), minute(), second(), day(), month() + 1, year());
  }
  if (timeSetYearFlag)
    setTime(hour(), minute(), second(), day(), month(), year() + 1);
  if (backlightSetLEDFlag)  {
    if (++light_mode >= sizeof(pwm_light) / sizeof(*pwm_light)) light_mode = 0;
    if (light_mode != 0 )
      analogWrite(PWM_PIN, pwm_light[light_mode]);
  }
  if (backlightSetThresoldFlag) {
    light_auto_thresold = light_auto_thresold + 10;
    if (light_auto_thresold >= 1023) light_auto_thresold = 700;
  }
  if (backlightSetMinFlag) {
    if (++light_auto_min >= sizeof(pwm_light) / sizeof(*pwm_light)) light_auto_min = 1;
  }
  if (backlightSetMaxFlag) {
    if (++light_auto_max >= sizeof(pwm_light) / sizeof(*pwm_light)) light_auto_max = 1;
  }
}

void buttonThree() {
  PRINTLNF("button three is pressed!");
  if (!menuFlag && !timeSetFlag && !backlightSetFlag) {
    menuFlag = true;
    lastIdleTimeMenu = millis(); //to control idle menu time
  }
  else if (menuFlag) {  //main menu
    switch (curMenuItem) {
      case 0:           //go to time set, minute set
        if (year() < 2018) setTime(hour(), minute(), second(), day(), month(), 2018); //set year of last firmware
        timeSetFlag = true;
        timeSetMinFlag = true;
        menuFlag = false;
        break;
      case 1: //set backlight
        backlightSetFlag = true;
        backlightSetLEDFlag = true;
        menuFlag = false;
        break;
      case 2:
        factoryReset();
        break;
      case 3:
        menuFlag = false; //exit from menu
        break;
    }
    curMenuItem = 0;
  }
  else if (timeSetFlag) {
    if (timeSetYearFlag) { //end time set
      timeSetMinFlag = timeSetHourFlag = timeSetDayFlag = timeSetMonthFlag = timeSetYearFlag = timeSetFlag = false;
      setTime(hour(), minute(), 0, day(), month(), year()); //zero seconds
    }
    else if (timeSetMinFlag) { //go to hour set
      timeSetMinFlag = false;
      timeSetHourFlag = true;
    }
    else if (timeSetHourFlag) { //go to day set
      timeSetHourFlag = false;
      timeSetTimeZoneFlag = true;
    }
    else if (timeSetTimeZoneFlag) { //go to timezone set
      timeSetTimeZoneFlag = false;
      timeSetDayFlag = true;
    }
    else if (timeSetDayFlag) { //go to month set
      timeSetDayFlag = false;
      timeSetMonthFlag = true;
    }
    else if (timeSetMonthFlag) { //go to year set
      timeSetMonthFlag = false;
      timeSetYearFlag = true;
    }
  }
  else if (backlightSetFlag) {
    if (light_mode == 0) {
      if (backlightSetMaxFlag) { //end of backlight set
        backlightSetMaxFlag = backlightSetFlag = false;
        writeConfig();
      }
      else if (backlightSetLEDFlag) {
        backlightSetLEDFlag = false;
        backlightSetThresoldFlag = true;
      }
      else if (backlightSetThresoldFlag) {
        backlightSetThresoldFlag = false;
        backlightSetMinFlag = true;
      }
      else if (backlightSetMinFlag) {
        backlightSetMinFlag = false;
        backlightSetMaxFlag = true;
      }
    }
    else
      backlightSetFlag = backlightSetLEDFlag = false;
  }
}

void setup() {
  analogWrite(PWM_PIN, pwm_light[light_mode]); //set backlight
  u8g2.begin();// init display
  drawBoot();
 
  #ifdef DEBUG
    Serial.begin(115200); //debug sensor serial port
  #else
    wifiManager.setDebugOutput(false); //disable wifiManager debug output
  #endif
  
  swSer.begin(9600); // init CO2 sensor serial port
  Wire.begin(I2C_SDA, I2C_SCL);  // init I2C interface

  if (!bme.begin(0x76)) { // init Pressure/Temperature sensor
    PRINTLNF("Could not find a valid BME280 sensor, check wiring!");
  }
  
  if (!SPIFFS.begin()) {  // init filesystem
    PRINTLNF("Failed to mount file system");
    ESP.reset();
  }
  loadConfig();
  delay(500);
  readMeasurements();
  for (byte i = 0; i < P_LEN; i++) { //generating p array to predict pressure dropping
    p_array[i] = pf;
  }

  if (setupWiFi()) {
    if (!loadConfigWiFI()) {
      PRINTLNF("Failed to load config");
      factoryReset();
    } 
    else {
      PRINTLNF("Config loaded");
    }
    drawBoot();

    Udp.begin(localPort);// Setup time
    PRINTLN("Local port: ", Udp.localPort());

    setSyncProvider(getNtpTime);
    for (byte i = 0; i <= 2; i++) { //trying to sync 3 times
      if (timeStatus() == timeSet) {
        PRINTLN("Time status: ", timeStatus());
        break;
      }
      PRINTLNF("Trying to sync");
      setTime(getNtpTime());
    }

    aping.on(false, [](const AsyncPingResponse & response) {
      #ifdef DEBUG
      IPAddress addr(response.addr); //to prevent with no const toString() in 2.3.0
      Serial.printf("total answer from %s sent %d recevied %d time %d ms\n", addr.toString().c_str(), response.total_sent, response.total_recv, response.total_time);
      if (response.mac)
        Serial.printf("detected eth address " MACSTR "\n", MAC2STR(response.mac->addr));
      #endif
      if (response.total_recv > 0)
        connectedInetFlag = true;
      else
        connectedInetFlag = false;
      return true;
    });
    asyncPing();

    #ifdef BLYNK
    // Start Blynk
    Blynk.config(blynk_token, blynk_server, blynk_port);
    connectBlynk();
    #else
    // Start Cayenne
    Cayenne.begin(cayenne_username,cayenne_password,cayenne_clientID);
    #endif

    setSyncInterval(SECS_PER_DAY); // NTP time sync interval
    timer.setInterval(30000L, sendMeasurements);
    timer.setInterval(51000L, asyncPing); //pinger
  }
  else
    WiFi.mode(WIFI_STA);
  timer.setInterval(SECS_PER_HOUR * 1000L, read_p_arr); //fill Pressure array
  timer.setInterval(10000L, readMeasurements);

}

void loop() {
  timer.run();

  if (menuFlag)
    drawMenu("MAIN MENU", curMenuItem, "Time set\nBacklight set\nErase data & reset\nExit");
  else
    drawMainScreen();

  adc_data = analogRead(A0);
  adcDecode();

  // if (connectedInetFlag)
    // Blynk.run();
}
