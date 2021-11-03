#define DEBUG true

/*
 Emulator DCF77
 Simulate a DCF77 radio receiver with a ESP8266, esp01 model
 Emits a complete three minute pulses train from the GPIO2 output
 the train is preceded by a single pulse at the lacking 59° pulse to allow some clock model synchronization
 of the beginning frame
 after the three pulses train one more single pulse is sent to safely close the frame
 get the time from the ntp service

 Uses Time library to facilitate time computation

 
 26/10/2021 fix the complete script to use ESP's internal mechanism to fetch the current time from an NTP server

 
 Known issue:
 -When the DayLightSaving mode change, the three minutes packet is not checked for possible changes of hour across the frame
  itself, moreover the daylight saving is changed normally at 3 o clock in the morning (here in italy), while I don't correct for it
  so the time will be probably incorrect before at least 03:03 then dayLightSaving changes
 -the exact "second" precision is not guaranteed because of the simplicity of the NTP implementation
  normally the packet transit delay would be taken into account, but here is not

 Fuso68 05/12/2015

 19/12/2015 added disconnect and reconnect if wlan fail, reconnect also after three failed ntp requests
 20/12/2015 increased wlan connect timeout to 30 seconds

 
   
 Based upon:
 Udp NTP Client

 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol

 created 4 Sep 2010
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 updated for the ESP8266 12 Apr 2015 
 by Ivan Grokhotkov
 updated to use ESP NTP client solution
 by Michael Raith

 This code is in the public domain.
 */

#include "LittleFS.h"

#include <ESP8266WiFi.h>

#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFiManager.h>

#include <ArduinoJson.h>

#include <Ticker.h>
#include "time.h"

#define HOSTNAME "ESP-DCF77"

char ntpServer[40] = "de.pool.ntp.org";
char timezone[40] = "CET-1CEST,M3.5.0/02,M10.5.0/03"; // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
int timeCorrectionOffset = 0; // Define a time correction offset in seconds

// OTA settings
unsigned int otaPort = 8266;
char otaPassword[32] = ""; // Set OTA password via WiFi manager!

const unsigned long checkInterval = 60000;
unsigned long lastCheck = 0;

// Flag for saving data
bool shouldSaveConfig = false;
// Flag for starting on demand wifi config portal
bool shouldStartConfigPortal = false;

// Routine timer 100 msec
Ticker dcfOutTimer;

// #define DCF_OUT_PIN LED_BUILTIN
#define DCF_OUT_PIN 2
#define WIFI_PORTAL_PIN D5 // use this pin to manually trigger the wifi portal

// How many total pulses we have
// Three complete minutes + 2 head pulses and one tail pulse
#define MaxPulseNumber 183
#define FirstMinutePulseBegin 2
#define SecondMinutePulseBegin 62
#define ThirdMinutePulseBegin 122

// Complete array of pulses for three minutes
// 0 = no pulse, 1=100msec, 2=200msec
int pulseArray[MaxPulseNumber];

int pulseCount = 0;
int dcfOutputOn = 0;
int partialpulseCount = 0;

void printLocalTime()
{
#ifdef DEBUG
  time_t now;
  struct tm *timeinfo;

  time(&now);
  // timeinfo = gmtime(&now); // returns GMT time!
  timeinfo = localtime(&now);

  // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println(asctime(timeinfo));
#endif
}

/**
 * Callback notifying us of the need to save config
 */
void saveConfigCallback()
{
#ifdef DEBUG
  Serial.println("Should save config");
#endif

  shouldSaveConfig = true;
}

void prepareFileSystem()
{
  // Clean FS, for testing
  // LittleFS.format();

  // Read configuration from FS json
#ifdef DEBUG
  Serial.println("mounting FS...");
#endif

  if (LittleFS.begin())
  {
#ifdef DEBUG
    Serial.println("mounted file system");
#endif

    if (LittleFS.exists("/config.json"))
    {
      // File exists, reading and loading
#ifdef DEBUG
      Serial.println("reading config file");
#endif

      File configFile = LittleFS.open("/config.json", "r");
      if (configFile)
      {
#ifdef DEBUG
        Serial.println("opened config file");
#endif

        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

#if ARDUINOJSON_VERSION_MAJOR >= 6
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if (!deserializeError)
        {
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
#endif
#ifdef DEBUG
          Serial.println("\nparsed json");
#endif

          strcpy(ntpServer, json["ntpServer"]);
          timeCorrectionOffset = json["timeCorrectionOffset"];
          strcpy(timezone, json["timezone"]);
          strcpy(otaPassword, json["otaPassword"]);
          otaPort = json["otaPort"];
        }
        else
        {
#ifdef DEBUG
          Serial.println("failed to load json config");
#endif
        }

        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  //end read
}

void connectToWiFi()
{
  char otaPort_buffer[5];
  itoa(otaPort, otaPort_buffer, 10);

  char timeCorrectionOffset_buffer[5];
  itoa(timeCorrectionOffset, timeCorrectionOffset_buffer, 10);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_ntp_server("ntp server", "NTP Server", ntpServer, 40);
  WiFiManagerParameter custom_timezone("timezone", "timezone", timezone, 40);
  WiFiManagerParameter custom_timeCorrectionOffset("time correction offset", "time correction offset in seconds", timeCorrectionOffset_buffer, 5);
  WiFiManagerParameter custom_ota_password("ota password", "OTA password", otaPassword, 32);
  WiFiManagerParameter custom_ota_port("ota port", "OTA port", otaPort_buffer, 5);

  // WiFiManager
  // Local initialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // Set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_ntp_server);
  wifiManager.addParameter(&custom_timezone);
  wifiManager.addParameter(&custom_timeCorrectionOffset);
  wifiManager.addParameter(&custom_ota_password);
  wifiManager.addParameter(&custom_ota_port);

  // Reset saved settings
  // wifiManager.resetSettings();

  // Sets timeout until configuration portal gets turned off. Useful to make it all retry or go to sleep in seconds.
  wifiManager.setTimeout(180);

#ifdef DEBUG
  wifiManager.setDebugOutput(true);
#else
  wifiManager.setDebugOutput(false);
#endif

// Set WiFi DNS hostname
#ifdef ESP8266
  WiFi.hostname(HOSTNAME);
#elif ESP32
  WiFi.setHostname(HOSTNAME);
#else
#warning("Cannot set hostname for unknown chip (it is not a ESP8266 or ESP32!)")
#endif

  // Fetches ssid and pass from eeprom and tries to connect. If it does not connect it starts an access point with the specified name and goes into a blocking loop awaiting configuration.
  if ((shouldStartConfigPortal && !wifiManager.startConfigPortal(HOSTNAME)) || !wifiManager.autoConnect(HOSTNAME))
  {
#ifdef DEBUG
    Serial.println("failed to connect and hit timeout");
#endif
    delay(3000);
    // Reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  shouldStartConfigPortal = false;

  //read updated parameters
  strcpy(ntpServer, custom_ntp_server.getValue());
  strcpy(timezone, custom_timezone.getValue());
  strcpy(timeCorrectionOffset_buffer, custom_timeCorrectionOffset.getValue());
  strcpy(otaPassword, custom_ota_password.getValue());
  strcpy(otaPort_buffer, custom_ota_port.getValue());
#ifdef DEBUG
  Serial.println("The values in the file are: ");
  Serial.println("\tntp server : " + String(ntpServer));
  Serial.println("\ttimezone : " + String(timezone));
  Serial.println("\ttime correction offset (sec) : " + String(timeCorrectionOffset_buffer));
  Serial.println("\tota password : " + String(otaPassword));
  Serial.println("\tota port : " + String(otaPort_buffer));
#endif

  // Save the custom parameters to FS
  if (shouldSaveConfig)
  {
#ifdef DEBUG
    Serial.println("saving config");
#endif

#if ARDUINOJSON_VERSION_MAJOR >= 6
    DynamicJsonDocument json(1024);
#else
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
#endif
    json["ntpServer"] = ntpServer;
    json["timezone"] = timezone;
    json["timeCorrectionOffset"] = atoi(timeCorrectionOffset_buffer);
    json["otaPassword"] = otaPassword;
    json["otaPort"] = atoi(otaPort_buffer);

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile)
    {
#ifdef DEBUG
      Serial.println("failed to open config file for writing");
#endif
    }

#if ARDUINOJSON_VERSION_MAJOR >= 6
#ifdef DEBUG
    serializeJson(json, Serial);
#endif
    serializeJson(json, configFile);
#else
#ifdef DEBUG
    json.printTo(Serial);
#endif
    json.printTo(configFile);
#endif
    configFile.close();
    //end save
  }

#ifdef DEBUG
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
}

/**
 * Called every 100 msec for DCF77 output
 */
void dcfOut()
{
  if (dcfOutputOn == 1)
  {
    switch (partialpulseCount++)
    {
    case 0:
      if (pulseArray[pulseCount] != 0)
        digitalWrite(DCF_OUT_PIN, 0);
      break;
    case 1:
      if (pulseArray[pulseCount] == 1)
        digitalWrite(DCF_OUT_PIN, 1);
      break;
    case 2:
      digitalWrite(DCF_OUT_PIN, 1);
      break;
    case 9:
      if (pulseCount++ == (MaxPulseNumber - 1))
      {
        // One less because we FIRST tx the pulse THEN count it
        pulseCount = 0;
        dcfOutputOn = 0;
      };

      partialpulseCount = 0;
      break;
    };
  };
}

int bin2Bcd(int dato)
{
  int msb, lsb;

  if (dato < 10)
    return dato;
  msb = (dato / 10) << 4;
  lsb = dato % 10;

  return msb + lsb;
}

void calculateArray(int ArrayOffset, tm *timeinfo)
{
  int n, Tmp, TmpIn;
  int ParityCount = 0;

  //first 20 bits are logical 0s
  for (n = 0; n < 20; n++)
    pulseArray[n + ArrayOffset] = 1;

  //DayLightSaving bit
  if (timeinfo->tm_isdst == 1)
    pulseArray[17 + ArrayOffset] = 2;
  else
    pulseArray[18 + ArrayOffset] = 2;

  //bit 20 must be 1 to indicate time active
  pulseArray[20 + ArrayOffset] = 2;

  //calculate minutes bits
  TmpIn = bin2Bcd(timeinfo->tm_min);
  for (n = 21; n < 28; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  };
  if ((ParityCount & 1) == 0)
    pulseArray[28 + ArrayOffset] = 1;
  else
    pulseArray[28 + ArrayOffset] = 2;

  //calculate hour bits
  ParityCount = 0;
  TmpIn = bin2Bcd(timeinfo->tm_hour);
  for (n = 29; n < 35; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  if ((ParityCount & 1) == 0)
    pulseArray[35 + ArrayOffset] = 1;
  else
    pulseArray[35 + ArrayOffset] = 2;
  ParityCount = 0;
  //calculate day bits
  TmpIn = bin2Bcd(timeinfo->tm_mday);
  for (n = 36; n < 42; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //calculate weekday bits
  TmpIn = bin2Bcd(timeinfo->tm_wday);
  for (n = 42; n < 45; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //calculate month bits
  TmpIn = bin2Bcd(timeinfo->tm_mon);
  for (n = 45; n < 50; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //calculate year bits
  TmpIn = bin2Bcd(timeinfo->tm_year - 2000);
  for (n = 50; n < 58; n++)
  {
    Tmp = TmpIn & 1;
    pulseArray[n + ArrayOffset] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //date parity
  if ((ParityCount & 1) == 0)
    pulseArray[58 + ArrayOffset] = 1;
  else
    pulseArray[58 + ArrayOffset] = 2;

  //last missing pulse
  pulseArray[59 + ArrayOffset] = 0;

#ifdef DEBUG
  /* for debug: print the whole 180 secs array
   * Serial.print(':');
  for (n=0;n<60;n++)
    Serial.print(pulseArray[n+ArrayOffset]);*/
#endif
}

void readAndDecodeTime()
{
  time_t now;
  struct tm *timeinfo;

  time(&now);
  timeinfo = localtime(&now);

  // Add time correction offset e.g. if DCF77 is send a little bit to late and the clock is 1-2 minutes behind.
  timeinfo->tm_sec += timeCorrectionOffset;
  mktime(timeinfo);

  // If we are over about the 56° second we risk to begin the pulses too late, so it's better
  // to skip at the half of the next minute and NTP+recalculate all again
  if (timeinfo->tm_sec > 56)
  {
    // delay(30000);
    // Do a passive wait
    lastCheck += 30000;

    return;
  }

  // Calculate bis array for the first minute
  calculateArray(FirstMinutePulseBegin, timeinfo);

  // Add one minute and calculate array again for the second minute
  timeinfo->tm_min += 1;
  mktime(timeinfo);

  calculateArray(SecondMinutePulseBegin, timeinfo);

  // One minute more for the third minute
  timeinfo->tm_min += 1;
  mktime(timeinfo);

  calculateArray(ThirdMinutePulseBegin, timeinfo);

  // How much seconds to the minute's end?
  // Don't forget that we begin transmission at second 58°
  int SkipSeconds = 58 - timeinfo->tm_sec;
  delay(SkipSeconds * 1000);

  // DCF begin
  dcfOutputOn = 1;

  // Three minutes are needed to transmit all the packet and
  // then wait more 30 secs to locate safely at the half of minute.
  // NB 150+60=210sec, 60secs are lost from main routine.
  delay(150000);
  // Do a passive wait
  // lastCheck += millis() + 150000;
}

void setupDcf()
{
  // DCF output pin
  pinMode(DCF_OUT_PIN, OUTPUT);
  digitalWrite(DCF_OUT_PIN, LOW);

  // Handle DCF pulses
  dcfOutTimer.attach_ms(100, dcfOut);

  // First 2 pulses: 1 + blank to simulate the packet beginning
  pulseArray[0] = 1;
  // Missing pulse indicates start of minute
  pulseArray[1] = 0;

  // Last pulse after the third 59° blank
  pulseArray[MaxPulseNumber - 1] = 1;

  pulseCount = 0;
  dcfOutputOn = 0; //we begin with the output OFF
}

void setupOta()
{
  // Port defaults to 8266
  ArduinoOTA.setPort(otaPort);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)otaPassword);

#ifdef DEBUG
  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                     });
#endif
  ArduinoOTA.begin();
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.println("INIT DCF77 emulator");
#endif
  /*** DCF ***/
  setupDcf();

  /*** WIFI ***/
  // Wifi portal trigger pin
  pinMode(WIFI_PORTAL_PIN, INPUT_PULLUP);

  prepareFileSystem();
  connectToWiFi();

  /*** OTA ***/
  setupOta();

  /*** NTP time ***/
  // Get time from NTP server
  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  configTime(timezone, ntpServer);
#ifdef DEBUG
  printLocalTime();
#endif
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectToWiFi();
  }
  else if (digitalRead(WIFI_PORTAL_PIN) == LOW)
  {
    shouldStartConfigPortal = true;

    connectToWiFi();
  }

  // Async wait without using blocking "delay"
  if ((millis() - lastCheck) > checkInterval)
  {
    lastCheck = millis();

#ifdef DEBUG
    printLocalTime();
#endif

    readAndDecodeTime();
  }
}
