#include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
// Wifi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
// HTTP requests
#include <ESP8266HTTPClient.h>
//for NTP
#include <WiFiUdp.h>
//Time lib
#include <TimeLib.h>

// OTA updates
#include <ESP8266httpUpdate.h>
// Blynk
#include <BlynkSimpleEsp8266.h>
// Debounce
#include <Bounce2.h> //https://github.com/thomasfredericks/Bounce2
// JSON
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

// GPIO Defines
#define I2C_SDA 5 // D1 Orange
#define I2C_SCL 4 // D2 Yellow
//#define HW_RESET 12 //TODO
// Debounce interval in ms
//#define DEBOUNCE_INTERVAL 10
//Bounce hwReset {Bounce()};

// Humidity/Temperature/Pressure
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#include <SPI.h>
#include <U8g2lib.h>
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock E=*/ 14, /* data R/W=*/ 13, /* RS=*/ 15);
byte x {0}; byte y {0};

// Handy timers
#include <SimpleTimer.h>

// SW Serial
#include <SoftwareSerial.h>
SoftwareSerial swSer(0, 2, false, 256); // GPIO15 (TX) and GPIO13 (RX)

// CO2 SERIAL
#define SENSOR_SERIAL swSer

byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char response[7];

// Pressure and temperature
Adafruit_BME280 bme;

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

// Network credentials
String ssid {"YourHomeWeather"};
//String pass {"YHWBopka"}; // пока без пароля

// Sensors data
int t { -100};
int p { -1};
int h { -1};
int co2 { -1};
float tf {0};
float pf {0};
float hf {0};
//flags
bool timeSyncFlag = false;
bool cloudSyncFlag = false;
long wifiRSSI = 0;
//flag for saving data, flag if connected
bool shouldSaveConfig, connectedFlag = false;

char loader[4] {'.'};
char dots {':'};

WiFiUDP Udp;
const char* ntpServerName = "ntp1.vniiftri.ru"; //NTP server
const int timeZone = 5;         // GMT +5
unsigned int localPort = 4567;  // local port to listen for UDP packets

const uint8_t custom_font30[358] U8G2_FONT_SECTION("custom_font30") =
  "\14\0\4\5\5\5\5\5\6\22\37\0\0\37\367\37\0\0\0\0\0\1M \6\0\300\210\2\60$\362"
  "CX\303\37\4\231\36*\267\30S\15\21\221\14!\211\20\223\4AM\61\266\234\202I\246\313\17>x"
  "\0\61\21\347cXK\271\330,(\201\206!\244\374\377\17\62$\362CX\3Q\5\21t\14\61\211\20"
  "\262\4\21Q\246e\321\345\372\203\17&'\227\351\362\3\42 y\246\0\63 \362CXCB\317@\22"
  "\305\226iYty\324\24c\313)\17H\321\245N\62]~\360\301\3\64>\362CX\3a\305\20V"
  "\14a\305\20V\14a\305\20V\14a\305\20V\14a\305\20V\14a\305\20V\14a\305\20V\14a"
  "\305\20V\14a\305\20V\14A\316<\2E\243E\227\277\1\0\65!\362CXCB\317@\22\305\226"
  "\351\234\314?\370\200\353\262N\62\275`\202\20E\10I\206\20\204\0\66\32\362CXCB\317@\22\305"
  "\226\351\234\34\271\346\21(\242L\371\301\7\17\67\30\362CX\3Q\15\71\363\10\24Q\246e\321\345"
  "\215;\42\234\374\237\3\70\30\362CXCB\317@\22\305\226\351/\213qGD\246\277\374\340\203\7\71"
  "\32\362CX\303\37\4\231~\371\201\25\220<StY'\231^D\2\315C\12:\11\304A\214\2\365"
  "@P\0\0\0";

static const uint8_t clock_bitmap[] U8X8_PROGMEM = { //размер 11x10
  0xF0, 0xF9, 0xF8, 0xFB, 0x0C, 0xFE, 0x4C, 0xFE, 0x4C, 0xFE, 0xCC, 0xFE,
  0x0C, 0xFE, 0xBF, 0xFF, 0xDE, 0xFB, 0x0C, 0xF8
};

static const uint8_t sync_bitmap[] U8X8_PROGMEM = { //размер 11x10
  0x7C, 0x00, 0xFE, 0x00, 0x83, 0x01, 0xE0, 0x07, 0xCC, 0x03, 0x9E, 0x01,
  0x3F, 0x00, 0x0C, 0x06, 0xF8, 0x03, 0xF0, 0x01
};

void drawMainScreen() {
  u8g2.clearBuffer();
  //draw time
  u8g2.setFont(custom_font30); //30 px height
  String timestr = printDigits(hour()) + dots + printDigits(minute());
  //  String timestr = printDigits(minute()) + dots + printDigits(second());
  u8g2.drawStr(16, 30 , timestr.c_str());
  //update dots
  ((millis() / 1000) % 2) == 0 ? dots = ':' : dots = ' ';
  //wi-fi signal quality
  //  static uint32_t m = millis();
  //  if ((millis() - m) > 500) {
  //    wifiRSSI=wifiRSSI+10;
  //    m = millis();
  //  }
  //  if (wifiRSSI > 90) wifiRSSI = 20;
  if (connectedFlag) {
    drawSignalQuality(0, 0);
  }
  if (timeSyncFlag) {
    u8g2.drawXBMP(0, 10, 11, 10, clock_bitmap);
  }
  if (cloudSyncFlag) {
    u8g2.drawXBMP(0, 22, 11, 10, sync_bitmap);
  }

  //  if (co2 > -1) { // CO2
  //    char co2a [5];
  //    sprintf (co2a, "%i", co2);
  //
  //    u8g2.setFont(u8g2_font_inb19_mf);
  //    x = (128 - u8g2.getStrWidth(co2a)) / 2;
  //    y = u8g2.getAscent() - u8g2.getDescent();
  //    u8g2.drawStr(x, y - 4 , co2a);
  //
  //    const char ppm[] {"ppm CO2"};
  //    u8g2.setFont(u8g2_font_6x12_mf);
  //    x = (128 - u8g2.getStrWidth(ppm)) / 2;
  //    y = y - 4 + u8g2.getAscent() - u8g2.getDescent();
  //    u8g2.drawStr(x, y, ppm);
  //  } else {
  //    loading();
  //    u8g2.setFont(u8g2_font_inb19_mf);
  //    x = (128 - u8g2.getStrWidth(loader)) / 2;
  //    y = u8g2.getAscent() - u8g2.getDescent();
  //    u8g2.drawStr(x, y, loader);
  //  }

  // Cycle Temp, Humidity, Pressure
  String measurementT {"..."};
  String measurementH {"..."};
  String measurementP {"..."};
  const char degree {176};

  t = 23; tf = 23.4; h = 55; hf = 55;
  if (t > -100) measurementT = String(tf, 1) + degree + "C";
  if (h > -1) measurementH = String(h) + "%";
  if (p > -1) measurementP =  "P: " + String(p) + " mmHg";

  u8g2.setFont(u8g2_font_9x18_mf);
  char measurementa [12];
  measurementT.toCharArray(measurementa, 12);
  u8g2.drawStr(0, 46, measurementa);

  measurementH.toCharArray(measurementa, 12);
  u8g2.drawStr(0, 64, measurementa);

  y = 46;
  measurementP.toCharArray(measurementa, 12);
  x = (128 - u8g2.getStrWidth(measurementa)) / 2;
  //  u8g2.drawStr(x, y, measurementa);

  u8g2.sendBuffer();
}

void loading() {
  long unsigned int count {(millis() / 500) % 4};
  memset(loader, '.', count);
  memset(&loader[count], 0, 1);
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

void drawBoot(String msg = "Loading...") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x18_mf);
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

//callback notifying the need to save config
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

  SENSOR_SERIAL.write(cmd, 9);
  memset(response, 0, 7);

  // Looking for packet start
  while (SENSOR_SERIAL.available() && (!header_found)) {
    if (SENSOR_SERIAL.read() == 0xff ) {
      if (SENSOR_SERIAL.read() == 0x86 ) header_found = true;
    }
  }

  if (header_found) {
    SENSOR_SERIAL.readBytes(response, 7);

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
  Serial.println("Getting Temperature from BME280");
  tf = bme.readTemperature();
  t = static_cast<int>(tf);

  // Humidity
  Serial.println("Getting Humidity from BME280");
  hf = bme.readHumidity();
  h = static_cast<int>(hf);

  // Pressure (in mmHg)
  Serial.println("Getting Pressure from BME280");
  pf = bme.readPressure() * 760.0 / 101325;
  p = static_cast<int>(pf);

  // CO2
  Serial.println("Getting CO2");
  readCO2();

  //WiFi SSID
  wifiRSSI = WiFi.RSSI();

  // Write to debug console
  Serial.println("H: " + String(hf) + "%");
  Serial.println("T: " + String(tf) + "C");
  Serial.println("P: " + String(pf) + "mmHg");
  Serial.println("CO2: " + String(co2) + "ppm");
  Serial.println("Wi-Fi RSSI: " + String(wifiRSSI) + "dBm");
}

void sendMeasurements() {
  // Send to server
  if (connectBlynk()) {
    Blynk.virtualWrite(V1, tf);
    Blynk.virtualWrite(V2, h);
    Blynk.virtualWrite(V4, p);
    Blynk.virtualWrite(V5, co2);

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

  drawConnectionDetails(ssid, "2 mins", "http://192.168.4.1");
  wifiManager.setTimeout(120);
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

/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

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
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
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
  // Init serial ports
  Serial.begin(115200);
  SENSOR_SERIAL.begin(9600);

  // Init I2C interface
  Wire.begin(I2C_SDA, I2C_SCL);

  // Setup HW reset
  //  pinMode(HW_RESET, INPUT_PULLUP);
  //  hwReset.interval(DEBOUNCE_INTERVAL);
  //  hwReset.attach(HW_RESET);

  // Init display
  u8g2.begin();
  drawBoot();

  // Init Pressure/Temperature sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  // Init filesystem
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    ESP.reset();
  }

  // Setup WiFi
  drawBoot("WiFi...");
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
    Serial.println(String(Udp.localPort()));
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

    drawBoot("Connecting...");
    Serial.println("Connecting to blynk...");
    if (!Blynk.connect()) {
      Serial.println("Failed to connect blynk...");
    }

    // Setup a function to be called every n second
    timer.setInterval(10000L, readMeasurements);
    timer.setInterval(30000L, sendMeasurements);
    readMeasurements();
  }
  else {
    timer.setInterval(10000L, readMeasurements);
    readMeasurements();
  }
}

void loop() {

  timer.run();
  drawMainScreen();

  if (Blynk.connected()) {
    Blynk.run();
  }

  //      Switch every 5 seconds
  //  switch ((millis() / 5000) % 2) {
  //    case 0:
  //      drawTime();
  //      break;
  //    default:

  //  }

  //  hwReset.update();
  //  if (hwReset.fell()) {
  //    factoryReset();
  //  }
}
