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
#include <WiFiUdp.h> //NTP
#include <TimeLib.h>
#include <ESP8266httpUpdate.h> // OTA updates
#include <BlynkSimpleEsp8266.h> // Blynk
#include <Bounce2.h> // Debounce https://github.com/thomasfredericks/Bounce2
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <SimpleTimer.h> // Handy timers
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8g2lib.h>
// GPIO Defines
#define I2C_SDA 5 // D1, SDA pin, GPIO5 for BME280
#define I2C_SCL 4 // D2, SCL pin, GPIO4 for BME280
#define PWM_PIN 0 //D3, GPIO0, for ST7920
#define SCLK_PIN 12 // D6, E pin, GPIO12, for ST7920
#define RW_PIN 13 // D7, R/W pin, GPIO13, for ST7920
#define RS_PIN 15 // D8, RS pin, GPIO15, for ST7920
#define TX_PIN 2 //D4, RX pin, GPIO2, for mz-h19
#define RX_PIN 14 //D5, TX pin, GPIO14, for mz-h19

//#define HW_RESET 12 //TODO
// Debounce interval in ms
//#define DEBOUNCE_INTERVAL 10
//Bounce hwReset {Bounce()};

//display ST7920
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0,/*display-clock E,SCLK;esp-GPIO12,D6*/SCLK_PIN,/*display-data R/W;esp-GPIO13,D7*/RW_PIN,/*display-RS;esp-GPIO15,D8*/RS_PIN);

// Humidity/Temperature/Pressure/CO2
Adafruit_BME280 bme;
SoftwareSerial swSer(RX_PIN, TX_PIN, false, 256);// CO2 SERIAL
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char response[7];

// Blynk token
char blynk_token[33] {"7ca0a9293079453499aa5453883510cf"};
char blynk_server[64] {"blynk-cloud.com"};
const uint16_t blynk_port {8442};
// Device Id
char device_id[17] = "Home Weather";
const char fw_ver[17] = "0.1.0";
// Handy timer
SimpleTimer timer;
// Setup Wifi connection
WiFiManager wifiManager;
#define WIFI_TIMEOUT 180
// Network credentials
String ssid {"YourHomeWeather"};
//String pass {"YHWBopka"}; // пока без пароля

// Sensors data
int t { -100};
int p { -1};
int h { -1};
int co2 { 0};
float tf {0};
float pf {0};
float hf {0};
// Math data
//http://mathhelpplanet.com/static.php?p=onlayn-mnk-i-regressionniy-analiz
//http://bit.ly/1DIbvyj
#define P_LEN 4
float p_array[P_LEN];
float delta;
//flags
bool timeSyncFlag = false;
bool cloudSyncFlag = false;
long wifiRSSI = 0;
//flag for saving data, flag if connected
bool shouldSaveConfig, connectedFlag = false;

char dots {':'};

const uint16_t pwm_light[6] = {0,64,96,128,192,256};
uint8_t light_mode {0};

WiFiUDP Udp;
const char* ntpServerName = "ntp1.vniiftri.ru"; //NTP server
const int timeZone = 5;         // GMT +5
unsigned int localPort = 4567;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

#include "graphics.h"


void drawMainScreen() {
  u8g2.clearBuffer();
  //draw time
  u8g2.setFont(custom_font30);
  u8g2.drawStr(15, 30 , String(printDigits(hour()) + dots + printDigits(minute())).c_str());
  //update dots
  ((millis() / 1000) % 2) == 0 ? dots = ':' : dots = ' ';

  if (connectedFlag) {
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
  u8g2.drawStr(108, 14 , printDigits(day()).c_str());

  //  t = 23; tf = 8.0; h = 55; hf = 85.0;  p = 740; pf = 740.0;
  //    co2 = 3526;
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
    u8g2.drawGlyph(76, 47, 30); //big arrow up (28-29) small arrow up/down (30-31) arrow up/down
  }
  else if (delta > 1) {
    u8g2.drawGlyph(76, 47, 28); //small arrow up
  }
  else if (delta < -2) {
    u8g2.drawGlyph(76, 47, 31); //big arrow down
  }
  else if (delta < -1) {
    u8g2.drawGlyph(76, 47, 29); //big arrow down
  }

  //  u8g2.drawGlyph(76, 47, 29); //arrow up   (28-29) small arrow up/down (30-31) arrow up/down
  u8g2.drawStr(87, 47, String(p).c_str());

  u8g2.setFont(custom_font7);
  u8g2.drawGlyph(48, 64, 0xb0); //degree sign
  u8g2.drawStr(51, 64, String("C").c_str());
  u8g2.drawGlyph(49, 48, 0x25); //percent
  u8g2.drawStr(117, 64 , String("y.e.").c_str());
  u8g2.drawStr(117, 37 , String("mm").c_str());
  u8g2.drawStr(117, 42 , String("pt.").c_str());
  u8g2.drawStr(117, 47 , String("ct.").c_str());

  u8g2.sendBuffer();
}

// utility for digital clock display: prints preceding colon and leading 0
String printDigits(int digits) {
  String formattedstr = "";
  if (digits < 10) {
    formattedstr = + "0";
  }
  formattedstr = formattedstr + String(digits);
  return formattedstr;
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
    case 0:
      return "VSK";
    default:
      return "0";
  }
}

void drawBoot(String msg = "Loading...") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x18_mf);
  byte x {0}; byte y {0};
  x = (128 - u8g2.getStrWidth(msg.c_str())) / 2;
  y = 32 + u8g2.getAscent() / 2;
  u8g2.drawStr(x, y, msg.c_str());
  u8g2.sendBuffer();
}

void drawSignalQuality(uint8_t x, uint8_t y) {
  y = y + 6;
  if (wifiRSSI >= -90)
    u8g2.drawFrame(x, y, 2, 2);
  if (wifiRSSI >= -80)
    u8g2.drawFrame(x + 3, y - 2, 2, 4);
  if (wifiRSSI >= -70)
    u8g2.drawFrame(x + 6, y - 4, 2, 6);
  if (wifiRSSI >= -60)
    u8g2.drawFrame(x + 9, y - 6, 2, 8);
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

  // URL
  x = (128 - u8g2.getStrWidth(url.c_str())) / 2;
  y = y + 1 + u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, url.c_str());

  u8g2.sendBuffer();
}

//callback notifying when need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void factoryReset() {
  Serial.println("Resetting to factory settings");
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
    for (char i = 0; i < 6; i++) {
      crc += response[i];
    }
    crc = 0xff - crc;
    crc++;

    if ( !(response[6] == crc) ) {
      Serial.println("CO2: CRC error: " + String(crc) + " / " + String(response[6]));
    } else {
      unsigned int responseHigh = (unsigned int) response[0];
      unsigned int responseLow = (unsigned int) response[1];
      unsigned int ppm = (256 * responseHigh) + responseLow;
      co2 = ppm;
      Serial.println("CO2:" + String(co2));
    }
  } else {
    Serial.println("CO2: Header not found");
  }
}

void readMeasurements() {
  // Read data, Temperature
  tf = bme.readTemperature();
  t = static_cast<int>(tf);

  // Humidity
  hf = bme.readHumidity();
  h = static_cast<int>(hf);

  // Pressure (in mmHg)
  pf = bme.readPressure() * 760.0 / 101325;
  p = static_cast<int>(floor(pf + 0.5));

  // CO2
  readCO2();

  //WiFi SSID
  wifiRSSI = WiFi.RSSI();

  // Write to debug console
  Serial.println("H: " + String(hf) + "%");
  Serial.println("T: " + String(tf) + "C");
  Serial.println("Pf: " + String(pf, 1) + "mmHg");
  Serial.println("CO2: " + String(co2) + "ppm");
  Serial.println("Wi-Fi RSSI: " + String(wifiRSSI) + "dBm");
}

void read_p_arr() {
  for (byte i = 0; i < P_LEN - 1; i++) {
    p_array[i] = p_array[i + 1];
  }
  p_array[P_LEN - 1] = bme.readPressure() * 760.0 / 101325;

  float sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;
  for (int i = 0; i < P_LEN; i++) {
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
  Serial.println("delta ");
  Serial.print(delta);
  Blynk.virtualWrite(V5, p);
}

void sendMeasurements() {   // Send to server
  if (connectBlynk()) {
    Blynk.virtualWrite(V1, tf);
    Blynk.virtualWrite(V2, h);
    Blynk.virtualWrite(V4, p);
    //    Blynk.virtualWrite(V5, co2);
    Blynk.virtualWrite(V6, delta);

    cloudSyncFlag = 1;
    Serial.println("Send to Blynk server");
  }
  else {
    cloudSyncFlag = 0;
    Serial.println("Send to Blynk server fails!");
  }
}

bool connectBlynk() {
  if (!Blynk.connected()) {
    Serial.println("Blync is not connectd, trying to connect...");
    if (!Blynk.connect()) {
      Serial.println("Failed to connect blynk...");
      return false;
    }
    else {
      Serial.println("Connected blynk");
      return true;
    }
  }
  else return true;
}

bool loadConfig() {
  Serial.println("Load config...");
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject &json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  // Save parameters
  strcpy(device_id, json["device_id"]);
  strcpy(blynk_server, json["blynk_server"]);
  strcpy(blynk_token, json["blynk_token"]);
}

bool setupWiFi() {
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Custom parameters
  WiFiManagerParameter custom_device_id("device_id", "Device name", device_id, 16);
  WiFiManagerParameter custom_blynk_server("blynk_server", "Blynk server", blynk_server, 64);
  WiFiManagerParameter custom_blynk_token("blynk_token", "Blynk token", blynk_token, 34);
  wifiManager.addParameter(&custom_blynk_server);
  wifiManager.addParameter(&custom_blynk_token);
  wifiManager.addParameter(&custom_device_id);

  drawConnectionDetails(ssid, "3 mins", "http://192.168.4.1");
  wifiManager.setTimeout(WIFI_TIMEOUT);
  //  wifiManager.setTimeout(1);
  //  wifiManager.setAPCallback(configModeCallback);

  if (!wifiManager.autoConnect(ssid.c_str())) {
    //  if (!wifiManager.autoConnect(ssid.c_str(), pass.c_str())) { \\ с паролем иногда не пускает, пока будем без
    Serial.println("failed to connect and hit timeout");
    connectedFlag = 0;
    return false;
  }
  else connectedFlag = true;

  //save the custom parameters to FS
  if (shouldSaveConfig && connectedFlag) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["device_id"] = custom_device_id.getValue();
    json["blynk_server"] = custom_blynk_server.getValue();
    json["blynk_token"] = custom_blynk_token.getValue();
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
    return 1;
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Virtual pin update FW
BLYNK_WRITE(V22) {
  if (param.asInt() == 1) {
    Serial.println("Got a FW update request");

    char full_version[34] {""};
    strcat(full_version, device_id);
    strcat(full_version, "::");
    strcat(full_version, fw_ver);

    t_httpUpdate_return ret = ESPhttpUpdate.update("http://romfrom.space/get", full_version);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.println("[update] Update failed.");
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[update] Update no Update.");
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[update] Update ok.");
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
  if (++light_mode >= 6) light_mode = 0; 
  analogWrite(PWM_PIN, pwm_light[light_mode]);
  Serial.println();
  Serial.println(light_mode);
  Serial.println(pwm_light[light_mode]);
  Serial.println();
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  IPAddress timeServer;
  WiFi.hostByName(ntpServerName, timeServer);
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      timeSyncFlag = 1;
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  timeSyncFlag = 0;
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
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

void setup() {
  analogWrite(PWM_PIN, 64);
  // Init serial ports
  Serial.begin(115200);
  swSer.begin(9600);
  // Init I2C interface
  Wire.begin(I2C_SDA, I2C_SCL);

  // Setup HW reset
  //  pinMode(HW_RESET, INPUT_PULLUP);
  //  hwReset.interval(DEBOUNCE_INTERVAL);
  //  hwReset.attach(HW_RESET);

  // Init display
  u8g2.begin();
  drawBoot();
  
  if (!bme.begin(0x76)) { // Init Pressure/Temperature sensor
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  if (!SPIFFS.begin()) {  // Init filesystem
    Serial.println("Failed to mount file system");
    ESP.reset();
  }

  timer.setInterval(SECS_PER_HOUR * 1000L, read_p_arr);
  timer.setInterval(10000L, readMeasurements);
  readMeasurements();
  for (byte i = 0; i < P_LEN; i++) { // generating p array to predict pressure dropping
    p_array[i] = pf;
  }
  
  drawBoot("WiFi..."); // Setup WiFi
  if (setupWiFi()) {
    // Load config
    drawBoot();
    if (!loadConfig()) {
      Serial.println("Failed to load config");
      factoryReset();
    } else {
      Serial.println("Config loaded");
    }

    //set NTP time
    Udp.begin(localPort);
    Serial.println("Local port: ");
    Serial.print(String(Udp.localPort()));
    // Setup time
    setSyncProvider(getNtpTime);
    setSyncInterval(SECS_PER_HOUR); // once a hour sync

    // Start blynk
    Blynk.config(blynk_token, blynk_server, blynk_port);
    Serial.print("blynk server: ");
    Serial.println(blynk_server);
    Serial.print("port: ");
    Serial.println(blynk_port);
    Serial.print("token: ");
    Serial.println(blynk_token);

    drawBoot("Connect to Blynk");
    connectBlynk();
    timer.setInterval(30000L, sendMeasurements);// Setup a function to be called every n second
  }
}

void loop() {
  timer.run();
  drawMainScreen();

  Serial.println(analogRead(A0));

  if (Blynk.connected()) {
    Blynk.run();
  }

  //  hwReset.update();
  //  if (hwReset.fell()) {
  //    factoryReset();
  //  }
}
