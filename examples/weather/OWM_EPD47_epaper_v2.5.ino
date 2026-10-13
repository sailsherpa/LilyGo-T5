// ESP32 Weather Display and a LilyGo EPD 4.7" Display, obtains Open Weather Map data, decodes and then displays it.
// This software, the ideas and concepts is Copyright (c) David Bird 2021. All rights to this software are reserved.
// #################################################################################################################

#include <Arduino.h>            // In-built
#include <esp_task_wdt.h>       // In-built
#include "freertos/FreeRTOS.h"  // In-built
#include "freertos/task.h"      // In-built
#include "epd_driver.h"         // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "esp_adc_cal.h"        // In-built

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <ArduinoWebsockets.h>  // hhttps://github.com/gilmaimon/ArduinoWebsockets
#include <HTTPClient.h>         // In-built

#include <WiFi.h>               // In-built
#include <SPI.h>                // In-built
#include <time.h>               // In-built

#include "owm_credentials.h"
#include "forecast_record.h"
#include "lang.h"

using namespace websockets;

#define SCREEN_WIDTH   EPD_WIDTH
#define SCREEN_HEIGHT  EPD_HEIGHT

//################  VERSION  ##################################################
String version = "2.5 / 4.7in";  // Programme version, see change log at end
//################ VARIABLES ##################################################

enum alignment {LEFT, RIGHT, CENTER};
#define White         0xFF
#define LightGrey     0xBB
#define Grey          0x88
#define DarkGrey      0x44
#define Black         0x00

#define autoscale_on  true
#define autoscale_off false
#define barchart_on   true
#define barchart_off  false

boolean LargeIcon   = true;
boolean SmallIcon   = false;
#define Large  20           // For icon drawing
#define Small  8            // For icon drawing
String  Time_str = "--:--:--";
String  Date_str = "-- --- ----";
int     _wifi_signal, _CurrentHour = 0, _CurrentMin = 0, _CurrentSec = 0, _EventCnt = 0, _vref = 1100;
//################ PROGRAM VARIABLES and OBJECTS ##########################################
#define max_readings 24 // Limited to 3-days here, but could go to 5-days = 40  

Forecast_record_type  WxConditions[1];
Forecast_record_type  WxForecast[max_readings];

float _pressure_readings[max_readings]    = {0};
float _temperature_readings[max_readings] = {0};
float _humidity_readings[max_readings]    = {0};
float _rain_readings[max_readings]        = {0};
float _snow_readings[max_readings]        = {0};

long _SleepDuration   = 60; // Sleep time in minutes, aligned to the nearest minute boundary, so if 30 will always update at 00 or 30 past the hour
int  _WakeupHour      = 8;  // Don't wakeup until after 07:00 to save battery power
int  _SleepHour       = 23; // Sleep after 23:00 to save battery power
long _StartTime       = 0;
long _SleepTimer      = 0;
long _Delta           = 30; // ESP32 rtc speed compensation, prevents display at xx:59:yy and then xx:00:yy (one minute later) to save power

//fonts
#include "opensans8b.h"
#include "opensans10b.h"
#include "opensans12b.h"
#include "opensans18b.h"
#include "opensans24b.h"
#include "opensans32b.h"
#include "opensans36b.h"

GFXfont  currentFont;
uint8_t *framebuffer;

// #########################################################################################
// Get the current time from the NTP server and update the CurrentHour, CurrentMin, CurrentSec variables
// Also update the Date_str and Time_str strings for display on the screen
// #########################################################################################
void BeginSleep() {
  
  // Power off the EPD display and enter deep sleep for SleepDuration minutes, to save power
  epd_poweroff_all();

  // Update the time before going to sleep, so that the ESP32 RTC is accurate
  UpdateLocalTime();
  _SleepTimer = (_SleepDuration * 60 - ((_CurrentMin % _SleepDuration) * 60 + _CurrentSec)) + _Delta; //Some ESP32 have a RTC that is too fast to maintain accurate time, so add an offset

  // Enter deep sleep for SleepDuration minutes, to save power
  esp_sleep_enable_timer_wakeup(_SleepTimer * 1000000LL); // in Secs, 1000000LL converts to Secs as unit = 1uSec
  Serial.println("Awake for : " + String((millis() - _StartTime) / 1000.0, 3) + "-secs");
  Serial.println("Entering " + String(_SleepTimer) + " (secs) of sleep time");
  Serial.println("Starting deep-sleep period...");
  esp_deep_sleep_start();     // Sleep for e.g. 30 minutes
}

// #########################################################################################
// Setup the NTP time server and timezone, then update the CurrentHour, CurrentMin, CurrentSec variables
// Also update the Date_str and Time_str strings for display on the screen
// #########################################################################################
boolean SetupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov"); //(gmtOffset_sec, daylightOffset_sec, ntpServer)
  setenv("TZ", Timezone, 1);  //setenv()adds the "TZ" variable to the environment with a value TimeZone, only used if set to 1, 0 means no change
  tzset(); // Set the TZ environment variable
  delay(100);
  return UpdateLocalTime();
}

// #########################################################################################
// Get the current time from the NTP server and update the CurrentHour, CurrentMin, CurrentSec variables
// Also update the Date_str and Time_str strings for display on the screen
// #########################################################################################
uint8_t StartWiFi() 
{
  Serial.println("\r\nConnecting to: " + String(ssid));
  IPAddress dns(8, 8, 8, 8); // Use Google DNS
  WiFi.disconnect();
  WiFi.mode(WIFI_STA); // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("STA: Failed!\n");
    WiFi.disconnect(false);
    delay(500);
    WiFi.begin(ssid, password);
  }
  if (WiFi.status() == WL_CONNECTED) {
    _wifi_signal = WiFi.RSSI(); // Get Wifi Signal strength now, because the WiFi will be turned off to save power!
    Serial.println("WiFi connected at: " + WiFi.localIP().toString());
  }
  else Serial.println("WiFi connection *** FAILED ***");
  return WiFi.status();
}

// #########################################################################################
// Switch off the WiFi to save power, as it is no longer needed
// #########################################################################################
void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi switched Off");
}

// #########################################################################################
// Initialise the system, setup the serial port, initialise the EPD display and allocate memory for the framebuffer
// #########################################################################################
void InitialiseSystem() 
{
  // Start the serial port for debugging
  _StartTime = millis();
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(String(__FILE__) + "\nStarting...");

  // Initialise the EPD display and allocate memory for the framebuffer
  epd_init();
  framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
  if (!framebuffer) Serial.println("Memory alloc failed!");
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);

}

void weather_loop() {
    // Nothing to do here
    delay(1000);
}

// #########################################################################################
// Main program, setup the system, connect to WiFi, get the weather data, display it on the EPD and then go to sleep
// #########################################################################################
void weather_setup() 
{
  InitialiseSystem();

  // Connect to WiFi at sleep cycle time, then get weather and forecast data,  then disconnect to save power
  if (StartWiFi() == WL_CONNECTED && SetupTime() == true) 
  {
    bool WakeUp = false;                
    if (_WakeupHour > _SleepHour)
      WakeUp = (_CurrentHour >= _WakeupHour || _CurrentHour <= _SleepHour); 
    else                             
      WakeUp = (_CurrentHour >= _WakeupHour && _CurrentHour <= _SleepHour);                              
    
    if (WakeUp) 
    {
      byte Attempts = 1;
      bool RxWeather  = false;
      bool RxForecast = false;

      WiFiClient wifiClient;   // wifi client object
      while ((RxWeather == false || RxForecast == false) && Attempts <= 2) 
      { 
        // Try up-to 2 time for Weather and Forecast data
        if (RxWeather  == false) RxWeather  = obtainWeatherData(wifiClient, "weather");
        if (RxForecast == false) RxForecast = obtainWeatherData(wifiClient , "forecast"); 
        Attempts++;
      }

      Serial.println("Received all weather data...");
      
      if (RxWeather && RxForecast) 
      { // Only if received both Weather or Forecast proceed
        StopWiFi();         // Reduces power consumption
        epd_poweron();      // Switch on EPD display
        epd_clear();        // Clear the screen
        DisplayWeather();   // Display the weather data
        edp_update();       // Update the display to show the information
        epd_poweroff_all(); // Switch off all power to EPD
      }
    }
  }

  // Sleep until next update, to save power, only if the current time is between WakeupHour and SleepHour
  BeginSleep();
}

// #########################################################################################
// Convert the readings to Imperial units, only the first 3-hours are used
// #########################################################################################
void Convert_Readings_to_Imperial() { // Only the first 3-hours are used
  WxConditions[0].Pressure = hPa_to_inHg(WxConditions[0].Pressure);
  WxForecast[0].Rainfall   = mm_to_inches(WxForecast[0].Rainfall);
  WxForecast[0].Snowfall   = mm_to_inches(WxForecast[0].Snowfall);
}

// #########################################################################################
// Decode the JSON data received from the Open Weather Map API, and store it in the WxConditions and WxForecast arrays
// #########################################################################################
bool DecodeWeather(WiFiClient& json, String Type) {
  Serial.print(F("\nCreating object...and "));

  DynamicJsonDocument doc(64 * 1024);                      // allocate the JsonDocument
  DeserializationError error = deserializeJson(doc, json); // Deserialize the JSON document
  if (error) {                                             // Test if parsing succeeds.
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return false;
  }
  // convert it to a JsonObject
  JsonObject root = doc.as<JsonObject>();
  Serial.println(" Decoding " + Type + " data");
  if (Type == "weather") {
    // All Serial.println statements are for diagnostic purposes and some are not required, remove if not needed with //
    //WxConditions[0].lon         = root["coord"]["lon"].as<float>();              Serial.println(" Lon: " + String(WxConditions[0].lon));
    //WxConditions[0].lat         = root["coord"]["lat"].as<float>();              Serial.println(" Lat: " + String(WxConditions[0].lat));
    WxConditions[0].Main0       = root["weather"][0]["main"].as<const char*>();        Serial.println("Main: " + String(WxConditions[0].Main0));
    WxConditions[0].Forecast0   = root["weather"][0]["description"].as<const char*>(); Serial.println("For0: " + String(WxConditions[0].Forecast0));
    //WxConditions[0].Forecast1   = root["weather"][1]["description"].as<const char*>(); Serial.println("For1: " + String(WxConditions[0].Forecast1));
    //WxConditions[0].Forecast2   = root["weather"][2]["description"].as<const char*>(); Serial.println("For2: " + String(WxConditions[0].Forecast2));
    WxConditions[0].Icon        = root["weather"][0]["icon"].as<const char*>();        Serial.println("Icon: " + String(WxConditions[0].Icon));
    WxConditions[0].Temperature = root["main"]["temp"].as<float>();              Serial.println("Temp: " + String(WxConditions[0].Temperature));
    WxConditions[0].Pressure    = root["main"]["pressure"].as<float>();          Serial.println("Pres: " + String(WxConditions[0].Pressure));
    WxConditions[0].Humidity    = root["main"]["humidity"].as<float>();          Serial.println("Humi: " + String(WxConditions[0].Humidity));
    WxConditions[0].Low         = root["main"]["temp_min"].as<float>();          Serial.println("TLow: " + String(WxConditions[0].Low));
    WxConditions[0].High        = root["main"]["temp_max"].as<float>();          Serial.println("THig: " + String(WxConditions[0].High));
    WxConditions[0].Windspeed   = root["wind"]["speed"].as<float>();             Serial.println("WSpd: " + String(WxConditions[0].Windspeed));
    WxConditions[0].Winddir     = root["wind"]["deg"].as<float>();               Serial.println("WDir: " + String(WxConditions[0].Winddir));
    WxConditions[0].Cloudcover  = root["clouds"]["all"].as<int>();               Serial.println("CCov: " + String(WxConditions[0].Cloudcover)); // in % of cloud cover
    WxConditions[0].Visibility  = root["visibility"].as<int>();                  Serial.println("Visi: " + String(WxConditions[0].Visibility)); // in metres
    WxConditions[0].Rainfall    = root["rain"]["1h"].as<float>();                Serial.println("Rain: " + String(WxConditions[0].Rainfall));
    WxConditions[0].Snowfall    = root["snow"]["1h"].as<float>();                Serial.println("Snow: " + String(WxConditions[0].Snowfall));
    //WxConditions[0].Country     = root["sys"]["country"].as<const char*>();            Serial.println("Ctry: " + String(WxConditions[0].Country));
    WxConditions[0].Sunrise     = root["sys"]["sunrise"].as<int>();              Serial.println("SRis: " + String(WxConditions[0].Sunrise));
    WxConditions[0].Sunset      = root["sys"]["sunset"].as<int>();               Serial.println("SSet: " + String(WxConditions[0].Sunset));
    WxConditions[0].Timezone    = root["timezone"].as<int>();                    Serial.println("TZon: " + String(WxConditions[0].Timezone));
  }
  if (Type == "forecast") {
    //Serial.println(json);
    Serial.print(F("\nReceiving Forecast period - ")); //------------------------------------------------
    JsonArray list                  = root["list"];
    for (byte r = 0; r < max_readings; r++) {
      Serial.println("\nPeriod-" + String(r) + "--------------");
      WxForecast[r].Dt                = list[r]["dt"].as<int>();
      WxForecast[r].Temperature       = list[r]["main"]["temp"].as<float>();              Serial.println("Temp: " + String(WxForecast[r].Temperature));
      WxForecast[r].Low               = list[r]["main"]["temp_min"].as<float>();          Serial.println("TLow: " + String(WxForecast[r].Low));
      WxForecast[r].High              = list[r]["main"]["temp_max"].as<float>();          Serial.println("THig: " + String(WxForecast[r].High));
      WxForecast[r].Pressure          = list[r]["main"]["pressure"].as<float>();          Serial.println("Pres: " + String(WxForecast[r].Pressure));
      WxForecast[r].Humidity          = list[r]["main"]["humidity"].as<float>();          Serial.println("Humi: " + String(WxForecast[r].Humidity));
      //WxForecast[r].Forecast0         = list[r]["weather"][0]["main"].as<const char*>();        Serial.println("For0: " + String(WxForecast[r].Forecast0));
      //WxForecast[r].Forecast1         = list[r]["weather"][1]["main"].as<const char*>();        Serial.println("For1: " + String(WxForecast[r].Forecast1));
      //WxForecast[r].Forecast2         = list[r]["weather"][2]["main"].as<const char*>();        Serial.println("For2: " + String(WxForecast[r].Forecast2));
      WxForecast[r].Icon              = list[r]["weather"][0]["icon"].as<const char*>();        Serial.println("Icon: " + String(WxForecast[r].Icon));
      //WxForecast[r].Description       = list[r]["weather"][0]["description"].as<const char*>(); Serial.println("Desc: " + String(WxForecast[r].Description));
      //WxForecast[r].Cloudcover        = list[r]["clouds"]["all"].as<int>();               Serial.println("CCov: " + String(WxForecast[r].Cloudcover)); // in % of cloud cover
      //WxForecast[r].Windspeed         = list[r]["wind"]["speed"].as<float>();             Serial.println("WSpd: " + String(WxForecast[r].Windspeed));
      //WxForecast[r].Winddir           = list[r]["wind"]["deg"].as<float>();               Serial.println("WDir: " + String(WxForecast[r].Winddir));
      WxForecast[r].Rainfall          = list[r]["rain"]["3h"].as<float>();                Serial.println("Rain: " + String(WxForecast[r].Rainfall));
      WxForecast[r].Snowfall          = list[r]["snow"]["3h"].as<float>();                Serial.println("Snow: " + String(WxForecast[r].Snowfall));
      WxForecast[r].Period            = list[r]["dt_txt"].as<const char*>();                    Serial.println("Peri: " + String(WxForecast[r].Period));
    } 
    //------------------------------------------
    float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure; // Measure pressure slope between ~now and later
    pressure_trend = ((int)(pressure_trend * 10)) / 10.0; // Remove any small variations less than 0.1
    WxConditions[0].Trend = "=";
    if (pressure_trend > 0)  WxConditions[0].Trend = "+";
    if (pressure_trend < 0)  WxConditions[0].Trend = "-";
    if (pressure_trend == 0) WxConditions[0].Trend = "0";

    if (Units == "I") Convert_Readings_to_Imperial();
  }
  return true;
}

//#########################################################################################
// Convert Unix time to a readable format
//#########################################################################################
String ConvertUnixTime(int unix_time) {
  // Returns either '21:12  ' or ' 09:12pm' depending on Units mode
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
  if (Units == "M") {
    strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
  }
  else {
    strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
  }
  return output;
}

//#########################################################################################
// Obtain the weather data from the Open Weather Map API, decode it and store it in the WxConditions and WxForecast arrays
//#########################################################################################
bool obtainWeatherData(WiFiClient & client, const String & RequestType) {
  const String units = (Units == "M" ? "metric" : "imperial");
  client.stop(); // close connection before sending a new request
  HTTPClient http;
  String uri = "/data/2.5/" + RequestType + "?q=" + City + "," + Country + "&APPID=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
  if (RequestType != "weather")
  {
    uri += "&cnt=" + String(max_readings);
  }
  http.begin(client, server, 80, uri); //http.begin(uri,test_root_ca); //HTTPS example connection
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    if (!DecodeWeather(http.getStream(), RequestType)) return false;
    client.stop();
    http.end();
    return true;
  }
  else
  {
    Serial.printf("connection failed, error: %s", http.errorToString(httpCode).c_str());
    client.stop();
    http.end();
    return false;
  }
  http.end();
  return true;
}

// #########################################################################################
// Convert the readings to Imperial units, only the first 3-hours are used
// #########################################################################################
float mm_to_inches(float value_mm) {
  return 0.0393701 * value_mm;
}

// #########################################################################################
// Convert the readings to Imperial units, only the first 3-hours are used
// #########################################################################################
float hPa_to_inHg(float value_hPa) {
  return 0.02953 * value_hPa;
}

// #########################################################################################
// Convert the readings to Imperial units, only the first 3-hours are used
// #########################################################################################
int JulianDate(int d, int m, int y) {
  int mm, yy, k1, k2, k3, j;
  yy = y - (int)((12 - m) / 10);
  mm = m + 9;
  if (mm >= 12) mm = mm - 12;
  k1 = (int)(365.25 * (yy + 4712));
  k2 = (int)(30.6001 * mm + 0.5);
  k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
  // 'j' for dates in Julian calendar:
  j = k1 + k2 + d + 59 + 1;
  if (j > 2299160) j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
  return j;
}

// #########################################################################################
// Calculate the sum of precipitation readings  
// #########################################################################################
float SumOfPrecip(float DataArray[], int readings) {
  float sum = 0;
  for (int i = 0; i <= readings; i++) {
    sum += DataArray[i];
  }
  return sum;
}

// #########################################################################################
// Convert the first character of a string to uppercase, and return the modified string
// #########################################################################################
String TitleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); // Title-case the string
  }
  else return text;
}

// #########################################################################################
// Calculate the current moon phase, returns a value between 0 and 1, where
// 0 = New Moon, 0.25 = First Quarter, 0.5 = Full Moon, 0.75 = Last Quarter
// #########################################################################################  
double NormalizedMoonPhase(int d, int m, int y) {
  int j = JulianDate(d, m, y);
  //Calculate approximate moon phase
  double Phase = (j + 4.867) / 29.53059;
  return (Phase - (int) Phase);
}

// #########################################################################################
// Display the weather data on the EPD display, including the current conditions, forecast and other information
// #########################################################################################
void DisplayWeather() 
{                          // 4.7" e-paper display is 960x540 resolution
  DisplayStatusSection(600, 15, _wifi_signal,false);    // Wi-Fi signal strength and Battery voltage
  DisplayGeneralInfoSection();                   // Top line of the display
  DisplayDisplayWindSection(137, 150, WxConditions[0].Winddir, WxConditions[0].Windspeed, 100);
  DisplayAstronomySection(5, 255);               // Astronomy section Sun rise/set, Moon phase and Moon icon
  DisplayMainWeatherSection(320, 105);           // Centre section of display for Location, temperature, Weather report, current Wx Symbol
  DisplayWeatherIcon(810, 130);                  // Display weather icon    scale = Large;
  DisplayForecastSection(320, 217);              // 3hr forecast boxes
}

// #########################################################################################
// Display the top line of the display, including the city name, date and time
// #########################################################################################
void DisplayGeneralInfoSection() {
  setFont(OpenSans10B);
  drawString(15, 20, City, LEFT);
  setFont(OpenSans8B);
  drawString(500, 20, Date_str + "  @   " + Time_str, LEFT);
}

// #########################################################################################
// Display the current weather conditions, including temperature, pressure, humidity and wind speed
// #########################################################################################
void DisplayWeatherIcon(int x, int y) {
  DisplayConditionsSection(x, y, WxConditions[0].Icon, LargeIcon);
}

// #########################################################################################
// Display the current weather conditions, including temperature, pressure, humidity and wind speed 
// #########################################################################################
void DisplayMainWeatherSection(int x, int y) {
  setFont(OpenSans8B);
  DisplayTemperatureSection(x, y - 40);
  DisplayForecastTextSection(x - 55, y + 25);
  DisplayPressureSection(x - 15, y + 90, WxConditions[0].Pressure, WxConditions[0].Trend);
}

// #########################################################################################
// Display the wind direction and speed, including a compass rose and an arrow indicating the wind direction
// #########################################################################################
void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius) 
{
  arrow(x, y, Cradius - 22, angle, 18, 33); // Show wind direction on outer circle of width and length
  setFont(OpenSans8B);
  int dxo, dyo, dxi, dyi;
  drawCircle(x, y, Cradius, Black);       // Draw compass circle
  drawCircle(x, y, Cradius + 1, Black);   // Draw compass circle
  drawCircle(x, y, Cradius * 0.7, Black); // Draw compass inner circle
  
  // Draw the compass rose and the cardinal direction labels
  for (float a = 0; a < 360; a = a + 22.5) 
  {
    dxo = Cradius * cos((a - 90) * PI / 180);
    dyo = Cradius * sin((a - 90) * PI / 180);
    if (a == 45)  drawString(dxo + x + 15, dyo + y - 18, TXT_NE, CENTER);
    if (a == 135) drawString(dxo + x + 20, dyo + y - 2,  TXT_SE, CENTER);
    if (a == 225) drawString(dxo + x - 20, dyo + y - 2,  TXT_SW, CENTER);
    if (a == 315) drawString(dxo + x - 15, dyo + y - 18, TXT_NW, CENTER);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
    dxo = dxo * 0.7;
    dyo = dyo * 0.7;
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
  }
  drawString(x, y - Cradius - 20,     TXT_N, CENTER);
  drawString(x, y + Cradius + 10,     TXT_S, CENTER);
  drawString(x - Cradius - 15, y - 5, TXT_W, CENTER);
  drawString(x + Cradius + 10, y - 5, TXT_E, CENTER);
  drawString(x + 3, y + 50, String(angle, 0) + "°", CENTER);
  
  // Draw the wind speed and direction in text format
  setFont(OpenSans12B);
  drawString(x, y - 50, WindDegToOrdinalDirection(angle), CENTER);
  setFont(OpenSans24B);
  drawString(x + 3, y - 18, String(windspeed, 1), CENTER);
  setFont(OpenSans12B);
  drawString(x, y + 25, (Units == "M" ? "m/s" : "mph"), CENTER);
}

// #########################################################################################
// Convert the wind direction in degrees to an ordinal direction (N, NE, E, SE, S, SW, W, NW) and return it as a string
// #########################################################################################
String WindDegToOrdinalDirection(float winddirection) {
  if (winddirection >= 348.75 || winddirection < 11.25)  return TXT_N;
  if (winddirection >=  11.25 && winddirection < 33.75)  return TXT_NNE;
  if (winddirection >=  33.75 && winddirection < 56.25)  return TXT_NE;
  if (winddirection >=  56.25 && winddirection < 78.75)  return TXT_ENE;
  if (winddirection >=  78.75 && winddirection < 101.25) return TXT_E;
  if (winddirection >= 101.25 && winddirection < 123.75) return TXT_ESE;
  if (winddirection >= 123.75 && winddirection < 146.25) return TXT_SE;
  if (winddirection >= 146.25 && winddirection < 168.75) return TXT_SSE;
  if (winddirection >= 168.75 && winddirection < 191.25) return TXT_S;
  if (winddirection >= 191.25 && winddirection < 213.75) return TXT_SSW;
  if (winddirection >= 213.75 && winddirection < 236.25) return TXT_SW;
  if (winddirection >= 236.25 && winddirection < 258.75) return TXT_WSW;
  if (winddirection >= 258.75 && winddirection < 281.25) return TXT_W;
  if (winddirection >= 281.25 && winddirection < 303.75) return TXT_WNW;
  if (winddirection >= 303.75 && winddirection < 326.25) return TXT_NW;
  if (winddirection >= 326.25 && winddirection < 348.75) return TXT_NNW;
  return "?";
}

// #########################################################################################
// Display the current temperature and humidity, as well as the forecast high and low temperatures  
// #########################################################################################
void DisplayTemperatureSection(int x, int y) {
  setFont(OpenSans18B);
  drawString(x - 30, y, String(WxConditions[0].Temperature, 1) + "°    " + String(WxConditions[0].Humidity, 0) + "%", LEFT);
  setFont(OpenSans12B);
  drawString(x + 10, y + 35, String(WxConditions[0].High, 0) + "° | " + String(WxConditions[0].Low, 0) + "°", CENTER); // Show forecast high and Low
}

// #########################################################################################
// Display the current weather conditions, including temperature, pressure, humidity and wind speed 
// #########################################################################################
void DisplayForecastTextSection(int x, int y) 
{
#define lineWidth 34

setFont(OpenSans12B);
  //Wx_Description = WxConditions[0].Main0;          // e.g. typically 'Clouds'
  String Wx_Description = WxConditions[0].Forecast0; // e.g. typically 'overcast clouds' ... you choose which
  Wx_Description.replace(".", ""); // remove any '.'
  int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
  while (p < Wx_Description.length()) 
  {
    if (Wx_Description.substring(p, p + 1) == " ") spaceRemaining = p;
    if (charCount > Width - 1) { // '~' is the end of line marker
      Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~" + Wx_Description.substring(spaceRemaining + 1);
      charCount = 0;
    }
    p++;
    charCount++;
  }
  if (WxForecast[0].Rainfall > 0) Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Units == "M" ? "mm" : "in")) + ")";
  //Wx_Description = wordWrap(Wx_Description, lineWidth);
  String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
  String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
  drawString(x + 30, y + 5, TitleCase(Line1), LEFT);
  if (Line1 != Line2) drawString(x + 30, y + 30, Line2, LEFT);
}

// #########################################################################################
// Display the current pressure and trend, as well as the visibility and cloud cover
// #########################################################################################
void DisplayPressureSection(int x, int y, float pressure, String slope) 
{
  setFont(OpenSans12B);
  DrawPressureAndTrend(x - 25, y + 10, pressure, slope);
  if (WxConditions[0].Visibility > 0) {
    Visibility(x + 145, y, String(WxConditions[0].Visibility) + "M");
    x += 150; // Draw the text in the same positions if one is zero, otherwise in-line
  }
  if (WxConditions[0].Cloudcover > 0) CloudCover(x + 145, y, WxConditions[0].Cloudcover);
}

// #########################################################################################
// Display the forecast weather for a given index, including the icon, time and high/low  
// #########################################################################################
void DisplayForecastWeather(int x, int y, int index) {
  int fwidth = 90;
  x = x + fwidth * index;
  DisplayConditionsSection(x + fwidth / 2, y + 90, WxForecast[index].Icon, SmallIcon);
  setFont(OpenSans10B);
  drawString(x + fwidth / 2, y + 30, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone).substring(0, 5)), CENTER);
  drawString(x + fwidth / 2, y + 125, String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°", CENTER);
}

// #########################################################################################
// Display the astronomy section, including sunrise/sunset times, moon phase and moon icon
// #########################################################################################
void DisplayAstronomySection(int x, int y) {
  setFont(OpenSans10B);
  drawString(x + 15, y + 30, ConvertUnixTime(WxConditions[0].Sunrise).substring(0, 5) + " " + TXT_SUNRISE, LEFT);
  drawString(x + 15, y + 50, ConvertUnixTime(WxConditions[0].Sunset).substring(0, 5) + " " + TXT_SUNSET, LEFT);
  time_t now = time(NULL);
  struct tm * now_utc  = gmtime(&now);
  const int day_utc    = now_utc->tm_mday;
  const int month_utc  = now_utc->tm_mon + 1;
  const int year_utc   = now_utc->tm_year + 1900;
  drawString(x + 15, y + 70, MoonPhase(day_utc, month_utc, year_utc, Hemisphere), LEFT);
  DrawMoon(x + 165, y - 15, day_utc, month_utc, year_utc, Hemisphere);
}

// #########################################################################################
// Draw the moon phase icon, based on the current date and hemisphere 
// #########################################################################################
void DrawMoon(int x, int y, int dd, int mm, int yy, String hemisphere) 
{
  const int diameter = 75;
  double Phase = NormalizedMoonPhase(dd, mm, yy);
  hemisphere.toLowerCase();
  if (hemisphere == "south") 
  {
    Phase = 1 - Phase;
  }
  // Draw dark part of moon
  fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, LightGrey);

  const int number_of_lines = 90;
  for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++) 
  {
    double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
    // Determine the edges of the lighted part of the moon
    double Rpos = 2 * Xpos;
    double Xpos1, Xpos2;
    if (Phase < 0.5) {
      Xpos1 = -Xpos;
      Xpos2 = Rpos - 2 * Phase * Rpos - Xpos;
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = Xpos - 2 * Phase * Rpos + Rpos;
    }
    // Draw light part of moon
    double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW1y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW2y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW3y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW4y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    drawLine(pW1x, pW1y, pW2x, pW2y, White);
    drawLine(pW3x, pW3y, pW4x, pW4y, White);
  }
  drawCircle(x + diameter - 1, y + diameter, diameter / 2, Black);
}

// #########################################################################################
// Calculate the current moon phase, and return a string describing the phase
// #########################################################################################
String MoonPhase(int d, int m, int y, String hemisphere) {
  int c, e;
  double jd;
  int b;
  if (m < 3) {
    y--;
    m += 12;
  }
  ++m;
  c   = 365.25 * y;
  e   = 30.6  * m;
  jd  = c + e + d - 694039.09;     /* jd is total days elapsed */
  jd /= 29.53059;                        /* divide by the moon cycle (29.53 days) */
  b   = jd;                              /* int(jd) -> b, take integer part of jd */
  jd -= b;                               /* subtract integer part to leave fractional part of original jd */
  b   = jd * 8 + 0.5;                /* scale fraction from 0-8 and round by adding 0.5 */
  b   = b & 7;                           /* 0 and 8 are the same phase so modulo 8 for 0 */
  if (hemisphere == "south") b = 7 - b;
  if (b == 0) return TXT_MOON_NEW;              // New;              0%  illuminated
  if (b == 1) return TXT_MOON_WAXING_CRESCENT;  // Waxing crescent; 25%  illuminated
  if (b == 2) return TXT_MOON_FIRST_QUARTER;    // First quarter;   50%  illuminated
  if (b == 3) return TXT_MOON_WAXING_GIBBOUS;   // Waxing gibbous;  75%  illuminated
  if (b == 4) return TXT_MOON_FULL;             // Full;            100% illuminated
  if (b == 5) return TXT_MOON_WANING_GIBBOUS;   // Waning gibbous;  75%  illuminated
  if (b == 6) return TXT_MOON_THIRD_QUARTER;    // Third quarter;   50%  illuminated
  if (b == 7) return TXT_MOON_WANING_CRESCENT;  // Waning crescent; 25%  illuminated
  return "";
}

// #########################################################################################
// Display the forecast section, including the forecast weather for the next 5 periods, as well
// as the graphs for pressure, temperature, humidity and precipitation
// #########################################################################################
void DisplayForecastSection(int x, int y) 
{
  int f = 0;
  do {
    DisplayForecastWeather(x, y, f);
    f++;
  } while (f < max_readings);
  int r = 0;
  do { // Pre-load temporary arrays with with data - because C parses by reference and remember that[1] has already been converted to I units
    if (Units == "I") _pressure_readings[r] = WxForecast[r].Pressure * 0.02953;   else _pressure_readings[r] = WxForecast[r].Pressure;
    if (Units == "I") _rain_readings[r]     = WxForecast[r].Rainfall * 0.0393701; else _rain_readings[r]     = WxForecast[r].Rainfall;
    if (Units == "I") _snow_readings[r]     = WxForecast[r].Snowfall * 0.0393701; else _snow_readings[r]     = WxForecast[r].Snowfall;
    _temperature_readings[r]                = WxForecast[r].Temperature;
    _humidity_readings[r]                   = WxForecast[r].Humidity;
    r++;
  } while (r < max_readings);
  int gwidth = 175, gheight = 100;
  int gx = (SCREEN_WIDTH - gwidth * 4) / 5 + 8 + 5;
  int gy = (SCREEN_HEIGHT - gheight - 27);
  int gap = gwidth + gx;
  // (x,y,width,height,MinValue, MaxValue, Title, Data Array, AutoScale, ChartMode)
  DrawGraph(gx + 0 * gap, gy, gwidth, gheight, 900, 1050, Units == "M" ? TXT_PRESSURE_HPA : TXT_PRESSURE_IN, _pressure_readings, max_readings, autoscale_on, barchart_off);
  DrawGraph(gx + 1 * gap, gy, gwidth, gheight, 10, 30,    Units == "M" ? TXT_TEMPERATURE_C : TXT_TEMPERATURE_F, _temperature_readings, max_readings, autoscale_on, barchart_off);
  DrawGraph(gx + 2 * gap, gy, gwidth, gheight, 0, 100,   TXT_HUMIDITY_PERCENT, _humidity_readings, max_readings, autoscale_off, barchart_off);
  if (SumOfPrecip(_rain_readings, max_readings) >= SumOfPrecip(_snow_readings, max_readings))
    DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_RAINFALL_MM : TXT_RAINFALL_IN, _rain_readings, max_readings, autoscale_on, barchart_on);
  else
    DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_SNOWFALL_MM : TXT_SNOWFALL_IN, _snow_readings, max_readings, autoscale_on, barchart_on);
}

// #########################################################################################
// Display the current weather conditions, including temperature, pressure, humidity and wind speed 

void DisplayConditionsSection(int x, int y, String IconName, bool IconSize) {
  Serial.println("Icon name: " + IconName);
  if      (IconName == "01d" || IconName == "01n")  Sunny(x, y, IconSize, IconName);
  else if (IconName == "02d" || IconName == "02n")  MostlySunny(x, y, IconSize, IconName);
  else if (IconName == "03d" || IconName == "03n")  Cloudy(x, y, IconSize, IconName);
  else if (IconName == "04d" || IconName == "04n")  MostlySunny(x, y, IconSize, IconName);
  else if (IconName == "09d" || IconName == "09n")  ChanceRain(x, y, IconSize, IconName);
  else if (IconName == "10d" || IconName == "10n")  Rain(x, y, IconSize, IconName);
  else if (IconName == "11d" || IconName == "11n")  Tstorms(x, y, IconSize, IconName);
  else if (IconName == "13d" || IconName == "13n")  Snow(x, y, IconSize, IconName);
  else if (IconName == "50d")                       Haze(x, y, IconSize, IconName);
  else if (IconName == "50n")                       Fog(x, y, IconSize, IconName);
  else                                              Nodata(x, y, IconSize, IconName);
}

// #########################################################################################
// Draw an arrow at a given position, size and angle, with a given width and length
// #########################################################################################
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
  float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
  float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, Black);
}

// #########################################################################################
// Draw a line segment between two points, with an offset for the start and end points 
// #########################################################################################
void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14) {
  drawLine(x + o1,  y + o2,  x + o3,  y + o4,  Black);
  drawLine(x + o11, y + o12, x + o13, y + o14, Black);
}

// #########################################################################################
// Draw the current pressure and trend, as well as the visibility and cloud cover
// #########################################################################################
void DrawPressureAndTrend(int x, int y, float pressure, String slope) {
  drawString(x + 25, y - 10, String(pressure, (Units == "M" ? 0 : 1)) + (Units == "M" ? "hPa" : "in"), LEFT);
  if      (slope == "+") {
    DrawSegment(x, y, 0, 0, 8, -8, 8, -8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, -8, 8, -8, 16, 0);
  }
  else if (slope == "0") {
    DrawSegment(x, y, 8, -8, 16, 0, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 8, -8, 16, 0, 8, 8, 16, 0);
  }
  else if (slope == "-") {
    DrawSegment(x, y, 0, 0, 8, 8, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, 8, 8, 8, 16, 0);
  }
}

// #########################################################################################
// Display the Wi-Fi signal strength and battery voltage on the display
// #########################################################################################
void DisplayStatusSection(int x, int y, int rssi, bool partialUpdate) {
  setFont(OpenSans8B);
  DrawRSSI(x + 305, y + 20, rssi, partialUpdate);
  DrawBattery(x + 150, y + 20);
}

// #########################################################################################
// Draw the Wi-Fi signal strength as a series of bars, based on the RSSI value
// #########################################################################################
void DrawRSSI(int x, int y, int rssi, bool partialUpdate) 
{
  static int last_rssi = 0;

  if (rssi == last_rssi && partialUpdate) 
  {
    return; // No change in RSSI, so no need to redraw the bars
  }
  last_rssi = rssi;

  Rect_t area;
  area.x = x; 
  area.y = y - 30;
  area.width =  100;
  area.height = 30;

// Clear only this area (partial refresh)
  if ( partialUpdate )
    epd_clear_area(area);


  int WIFIsignal = 0;
  int xpos = 1;
  for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) 
  {
    if (_rssi <= -20)  WIFIsignal = 30; //            <-20dbm displays 5-bars
    if (_rssi <= -40)  WIFIsignal = 24; //  -40dbm to  -21dbm displays 4-bars
    if (_rssi <= -60)  WIFIsignal = 18; //  -60dbm to  -41dbm displays 3-bars
    if (_rssi <= -80)  WIFIsignal = 12; //  -80dbm to  -61dbm displays 2-bars
    if (_rssi <= -100) WIFIsignal = 6;  // -100dbm to  -81dbm displays 1-bar
    fillRect(x + xpos * 8, y - WIFIsignal, 6, WIFIsignal, Black);
    xpos++;
  }
}

// #########################################################################################
// Get the current time from the NTP server and update the _CurrentHour, _CurrentMin, _CurrentSec variables
// Also update the Date_str and Time_str strings for display on the screen
// #########################################################################################
boolean UpdateLocalTime() {
  struct tm timeinfo;
  char   time_output[30], day_output[30], update_time[30];
  while (!getLocalTime(&timeinfo, 5000)) { // Wait for 5-sec for time to synchronise
    Serial.println("Failed to obtain time");
    return false;
  }
  _CurrentHour = timeinfo.tm_hour;
  _CurrentMin  = timeinfo.tm_min;
  _CurrentSec  = timeinfo.tm_sec;
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S");      // Displays: Saturday, June 24 2017 14:05:49
  if (Units == "M") {
    sprintf(day_output, "%s, %02u %s %04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
    strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo);  // Creates: '@ 14:05:49'   and change from 30 to 8 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    sprintf(time_output, "%s", update_time);
  }
  else
  {
    strftime(day_output, sizeof(day_output), "%a %b-%d-%Y", &timeinfo); // Creates  'Sat May-31-2019'
    strftime(update_time, sizeof(update_time), "%r", &timeinfo);        // Creates: '@ 02:05:49pm'
    sprintf(time_output, "%s", update_time);
  }
  Date_str = day_output;
  Time_str = time_output;
  return true;
}

// #########################################################################################
// Draw the ESP32 battery voltage and percentage on the display
// ######################################################################################### 
void DrawBattery(int x, int y) 
{
  uint8_t percentage = 100;
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    _vref = adc_chars.vref;
  }
  float voltage = analogRead(36) / 4096.0 * 6.566 * (_vref / 1000.0);
  if (voltage > 1 ) 
  { // Only display if there is a valid reading
    Serial.println("\nVoltage = " + String(voltage));
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.20) percentage = 0;  // orig 3.5
    drawRect(x + 25, y - 14, 40, 15, Black);
    fillRect(x + 65, y - 10, 4, 7, Black);
    fillRect(x + 27, y - 12, 36 * percentage / 100.0, 11, Black);
    drawString(x + 85, y - 14, String(percentage) + "%  " + String(voltage, 1) + "v", LEFT);
  }
}

// #########################################################################################
// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
// #########################################################################################
void addcloud(int x, int y, int scale, int linesize) 
{
  fillCircle(x - scale * 3, y, scale, Black);                                                              // Left most circle
  fillCircle(x + scale * 3, y, scale, Black);                                                              // Right most circle
  fillCircle(x - scale, y - scale, scale * 1.4, Black);                                                    // left middle upper circle
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, Black);                                       // Right middle upper circle
  fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, Black);                                 // Upper and lower lines
  fillCircle(x - scale * 3, y, scale - linesize, White);                                                   // Clear left most circle
  fillCircle(x + scale * 3, y, scale - linesize, White);                                                   // Clear right most circle
  fillCircle(x - scale, y - scale, scale * 1.4 - linesize, White);                                         // left middle upper circle
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, White);                            // Right middle upper circle
  fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, White); // Upper and lower lines
}

// #########################################################################################
// Draw the rain symbol at a given position, size and scale
// #########################################################################################
void addrain(int x, int y, int scale, bool IconSize) 
{
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 12, "///////", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 25, "///////", LEFT);
  }
}

// #########################################################################################
// Draw the snow symbol at a given position, size and scale
// #########################################################################################
void addsnow(int x, int y, int scale, bool IconSize) {
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 15, "* * * *", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 30, "* * * *", LEFT);
  }
}

// #########################################################################################
// Draw the thunderstorm symbol at a given position, size and scale 
// #########################################################################################
void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 5; i++) {
    drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, Black);
    if (scale != Small) {
      drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, Black);
      drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, Black);
    }
    drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, Black);
    if (scale != Small) {
      drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, Black);
      drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, Black);
    }
    drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, Black);
    if (scale != Small) {
      drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, Black);
      drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, Black);
    }
  }
}

// #########################################################################################
// Draw the sun symbol at a given position, size and scale
// #########################################################################################
void addsun(int x, int y, int scale, bool IconSize) {
  int linesize = 5;
  fillRect(x - scale * 2, y, scale * 4, linesize, Black);
  fillRect(x, y - scale * 2, linesize, scale * 4, Black);
  drawLine(x - scale * 1.3, y - scale * 1.3, x + scale * 1.3, y + scale * 1.3, Black);
  drawLine(x - scale * 1.3, y + scale * 1.3, x + scale * 1.3, y - scale * 1.3, Black);
  if (IconSize == LargeIcon) {
    drawLine(1 + x - scale * 1.3, y - scale * 1.3, 1 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(2 + x - scale * 1.3, y - scale * 1.3, 2 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(3 + x - scale * 1.3, y - scale * 1.3, 3 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(1 + x - scale * 1.3, y + scale * 1.3, 1 + x + scale * 1.3, y - scale * 1.3, Black);
    drawLine(2 + x - scale * 1.3, y + scale * 1.3, 2 + x + scale * 1.3, y - scale * 1.3, Black);
    drawLine(3 + x - scale * 1.3, y + scale * 1.3, 3 + x + scale * 1.3, y - scale * 1.3, Black);
  }
  fillCircle(x, y, scale * 1.3, White);
  fillCircle(x, y, scale, Black);
  fillCircle(x, y, scale - linesize, White);
}

// #########################################################################################
// Draw the moon symbol at a given position, size and scale
// #########################################################################################
void addfog(int x, int y, int scale, int linesize, bool IconSize) {
  if (IconSize == SmallIcon) {
    y -= 10;
    linesize = 1;
  }
  for (int i = 0; i < 6; i++) {
    fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, Black);
  }
}

void Sunny(int x, int y, bool IconSize, String IconName) {
  int scale = Small, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  else y = y - 3; // Shift up small sun icon
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  scale = scale * 1.6;
  addsun(x, y, scale, IconSize);
}

void MostlySunny(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
}

void MostlyCloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
}

void Cloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x + 15, y - 22, scale / 2, linesize); // Cloud top right
  addcloud(x - 10, y - 18, scale / 2, linesize); // Cloud top left
  addcloud(x, y, scale, linesize);             // Main cloud
}

void Rain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

void ExpectRain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

void ChanceRain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

void Tstorms(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addtstorm(x, y, scale);
}

void Snow(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addsnow(x, y, scale, IconSize);
}

void Fog(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y - 5, scale, linesize);
  addfog(x, y - 5, scale, linesize, IconSize);
}

void Haze(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x, y - 5, scale * 1.4, IconSize);
  addfog(x, y - 5, scale * 1.4, linesize, IconSize);
}

void CloudCover(int x, int y, int CCover) {
  addcloud(x - 9, y + 2, Small * 0.3, 2); // Cloud top left
  addcloud(x + 3, y - 2, Small * 0.3, 2); // Cloud top right
  addcloud(x, y + 10, Small * 0.6, 2); // Main cloud
  drawString(x + 20, y, String(CCover) + "%", LEFT);
}

// #########################################################################################
// Draw the visibility symbol at a given position, size and scale
// #########################################################################################
void Visibility(int x, int y, String Visi) {
  float start_angle = 0.52, end_angle = 2.61, Offset = 8;
  int r = 14;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y - r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y - r / 2 + r * sin(i) + Offset, Black);
  }
  start_angle = 3.61; end_angle = 5.78;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y + r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y + r / 2 + r * sin(i) + Offset, Black);
  }
  fillCircle(x, y + Offset, r / 4, Black);
  drawString(x + 20, y, Visi, LEFT);
}

void addmoon(int x, int y, int scale, bool IconSize) {
  if (IconSize == LargeIcon) {
    fillCircle(x - 85, y - 100, uint16_t(scale * 0.8), Black);
    fillCircle(x - 57, y - 100, uint16_t(scale * 1.6), White);
  }
  else
  {
    fillCircle(x - 28, y - 37, uint16_t(scale * 1.0), Black);
    fillCircle(x - 20, y - 37, uint16_t(scale * 1.6), White);
  }
}

// #########################################################################################
// Draw a question mark symbol at a given position, size and scale
// #########################################################################################
void Nodata(int x, int y, bool IconSize, String IconName) {
  if (IconSize == LargeIcon) 
    setFont(OpenSans24B); 
  else 
    setFont(OpenSans12B);

  drawString(x - 3, y - 10, "?", CENTER);
}

/* (C) D L BIRD
    This function will draw a graph on a ePaper/TFT/LCD display using data from an array containing data to be graphed.
    The variable 'max_readings' determines the maximum number of data elements for each array. Call it with the following parametric data:
    x_pos-the x axis top-left position of the graph
    y_pos-the y-axis top-left position of the graph, e.g. 100, 200 would draw the graph 100 pixels along and 200 pixels down from the top-left of the screen
    width-the width of the graph in pixels
    height-height of the graph in pixels
    Y1_Max-sets the scale of plotted data, for example 5000 would scale all data to a Y-axis of 5000 maximum
    data_array1 is parsed by value, externally they can be called anything else, e.g. within the routine it is called data_array1, but externally could be temperature_readings
    auto_scale-a logical value (TRUE or FALSE) that switches the Y-axis autoscale On or Off
    barchart_on-a logical value (TRUE or FALSE) that switches the drawing mode between barhcart and line graph
    barchart_colour-a sets the title and graph plotting colour
    If called with Y!_Max value of 500 and the data never goes above 500, then autoscale will retain a 0-500 Y scale, if on, the scale increases/decreases to match the data.
    auto_scale_margin, e.g. if set to 1000 then autoscale increments the scale by 1000 steps.
*/
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode) {
#define auto_scale_margin 0 // Sets the autoscale increment, so axis steps up fter a change of e.g. 3
#define y_minor_axis 5      // 5 y-axis division markers
  setFont(OpenSans10B);
  int maxYscale = -10000;
  int minYscale =  10000;
  int last_x, last_y;
  float x2, y2;
  if (auto_scale == true) {
    for (int i = 1; i < readings; i++ ) {
      if (DataArray[i] >= maxYscale) maxYscale = DataArray[i];
      if (DataArray[i] <= minYscale) minYscale = DataArray[i];
    }
    maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
    Y1Max = round(maxYscale + 0.5);
    if (minYscale != 0) minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
    Y1Min = round(minYscale);
  }
  // Draw the graph
  last_x = x_pos + 1;
  last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
  drawRect(x_pos, y_pos, gwidth + 3, gheight + 2, Grey);
  drawString(x_pos - 20 + gwidth / 2, y_pos - 28, title, CENTER);
  for (int gx = 0; gx < readings; gx++) {
    x2 = x_pos + gx * gwidth / (readings - 1) - 1 ; // max_readings is the global variable that sets the maximum data that can be plotted
    y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
    if (barchart_mode) {
      fillRect(last_x + 2, y2, (gwidth / readings) - 1, y_pos + gheight - y2 + 2, Black);
    } else {
      drawLine(last_x, last_y - 1, x2, y2 - 1, Black); // Two lines for hi-res display
      drawLine(last_x, last_y, x2, y2, Black);
    }
    last_x = x2;
    last_y = y2;
  }
  //Draw the Y-axis scale
#define number_of_dashes 20
  for (int spacing = 0; spacing <= y_minor_axis; spacing++) {
    for (int j = 0; j < number_of_dashes; j++) { // Draw dashed graph grid lines
      if (spacing < y_minor_axis) drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes), y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes), Grey);
    }
    if ((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing) < 5 || title == TXT_PRESSURE_IN) {
      drawString(x_pos - 10, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
    }
    else
    {
      if (Y1Min < 1 && Y1Max < 10) {
        drawString(x_pos - 3, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
      }
      else {
        drawString(x_pos - 7, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0), RIGHT);
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    drawString(20 + x_pos + gwidth / 3 * i, y_pos + gheight + 10, String(i) + "d", LEFT);
    if (i < 2) drawFastVLine(x_pos + gwidth / 3 * i + gwidth / 3, y_pos, gheight, LightGrey);
  }
}

// #########################################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################
// #### DRAWING PRIMITIVES  ################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################

// #########################################################################################
// Draw a string of text on the display at a given position, with optional alignment (LEFT, CENTER, RIGHT)
// #########################################################################################
void drawString(int x, int y, String text, alignment align) {
  char * data  = const_cast<char*>(text.c_str());
  int  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  int w, h;
  int xx = x, yy = y;
  get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  int cursor_y = y + h;
  write_string(&currentFont, data, &x, &cursor_y, framebuffer);
}

Rect_t GetTextBounds(int x, int y, String text) 
{
  char * data  = const_cast<char*>(text.c_str());
  int  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  int w, h;
  int xx = x, yy = y;
  get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
  Rect_t bounds = {x1, y1, w, h};
  return bounds;
}


// #########################################################################################
// Draw a filled circle on the display at a given position, with a given radius and color
// #########################################################################################
void fillCircle(int x, int y, int r, uint8_t color) {
  epd_fill_circle(x, y, r, color, framebuffer);
}

// #########################################################################################
// Draw a filled rectangle on the display at a given position, with a given width, height, and color
// #########################################################################################  
void drawFastHLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_hline(x0, y0, length, color, framebuffer);
}

// #########################################################################################
// Draw a filled vertical line on the display at a given position, with a given length and
// color
// #########################################################################################  
void drawFastVLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_vline(x0, y0, length, color, framebuffer);
}

// #########################################################################################
// Draw a line on the display from (x0, y0) to (x1, y1) with a given color
// #########################################################################################  
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  epd_write_line(x0, y0, x1, y1, color, framebuffer);
}

// #########################################################################################
// Draw a circle on the display at a given position, with a given radius and color
// #########################################################################################
void drawCircle(int x0, int y0, int r, uint8_t color) {
  epd_draw_circle(x0, y0, r, color, framebuffer);
}

// #########################################################################################
// Draw a thick arc whose outer diameter is diameter. Angle is the counter-clockwise sweep from 0 degrees.
// The arc is drawn with the specified thickness and color.
// #########################################################################################
void drawArc(int centerX, int centerY, int radius, int thickness, uint8_t color, float angle) 
{
  if (radius <= 0 || thickness <= 0 || angle < 0) return;
  if (angle > 360) angle = 360;

  const float outerRadius = radius;
  const float innerRadius = fmaxf(0.0f, outerRadius - thickness + 1.0f);
  const float sweepRadians = angle * PI / 180.0f;
  const int steps = (int)(sweepRadians * outerRadius) + 1;

  for (int step = 0; step <= steps; ++step) 
  {
    const float radians = sweepRadians * step / steps;
    const int x0 = centerX + (int)roundf(innerRadius * cosf(radians));
    const int y0 = centerY - (int)roundf(innerRadius * sinf(radians));
    const int x1 = centerX + (int)roundf(outerRadius * cosf(radians));
    const int y1 = centerY - (int)roundf(outerRadius * sinf(radians));
    drawLine(x0, y0, x1, y1, color);
  }
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_draw_rect(x, y, w, h, color, framebuffer);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_fill_rect(x, y, w, h, color, framebuffer);
}

void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color) {
  epd_fill_triangle(x0, y0, x1, y1, x2, y2, color, framebuffer);
}

void drawPixel(int x, int y, uint8_t color) {
  epd_draw_pixel(x, y, color, framebuffer);
}

void setFont(GFXfont const &font) {
  currentFont = font;
}

// #########################################################################################
// Update the ePaper display with the contents of the framebuffer
// #########################################################################################
void edp_update() {
  epd_draw_grayscale_image(epd_full_screen(), framebuffer); // Update the screen
}

// #########################################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################
// #### BMS DISPLAY SECTION ################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################
// #########################################################################################

// ---------- WebSocket helpers ----------

WebsocketsClient _client;

// ---------- Globals ----------
unsigned long _lastScreenUpdate = 0;  

// Last drawn text for change detection
String _lastClockText;
String _lastCalLine1Text;
String _lastCalLine2Text;

// REC BMS  (host/port loaded from secrets.h)
const char* _websockets_server_host  = RECBMS_HOST_ADDR;  
const uint16_t _websockets_server_port = RECBMS_WEBSOCKET_PORT;
const char* _websockets_server_path  = RECBMS_WEBSOCKET_PART;  

// Update intervals (ms)
const unsigned long _UPDATE_INTERVAL_MS = 30UL * 1000UL;         // 30 seconds


float _BMS_Voltage = 0.0;          // Display the battery voltage
float _BMS_Current = 0.0;          // Display the battery current
float _BMS_MaxCellVoltage = 0.0;  // Display the minimum and maximum cell voltages
float _BMS_MinCellVoltage = 0.0;
int _BMS_SOC = 0;                   // State of the battery 
int _BMS_MinCellNum = 0;                // Number of the cell with the minimum voltage
int _BMS_MaxCellNum = 0;                // Number of the cell with the maximum voltage
int _BMS_BattTemperature = 0;
bool _optoChgRelay = false;          // State of the charge relay
bool _optoDsgRelay = false;          // State of the discharge relay

void recbms_demo_data() 
{
  _BMS_Voltage = random(2590, 2741) / 100.0f; // Random demo voltage from 25.90 V to 27.40 V
  _BMS_Current = -5.0;          // Display the battery current
  _BMS_MaxCellVoltage = 3.21;  // Display the minimum and maximum cell voltages
  _BMS_MinCellVoltage = 3.25;
  _BMS_SOC = 57;                  // State of the battery 
  _BMS_MinCellNum = 1;                // Number of the cell with the minimum voltage
  _BMS_MaxCellNum = 8;                // Number of the cell with the maximum voltage
  _BMS_BattTemperature = 25;          // Battery temperature
  _optoChgRelay = true;            // State of the charge relay
  _optoDsgRelay = false;          // State of the discharge relay
}


// #########################################################################################
// Display the BMS information on the e-paper display, including Wi-Fi signal strength, 
// battery voltage, general info, wind conditions, astronomy info, main weather section, weather icon, and forecast section
// #########################################################################################
void DisplayBMS(bool partialUpdate) 
{                          // 4.7" e-paper display is 960x540 resolution
  RECDisplayStatusSection(600, 15, _wifi_signal, partialUpdate);    // Wi-Fi signal strength and Battery voltage
  RECDisplayGeneralInfoSection(partialUpdate);                   // Top line of the display
  RECDisplaySOCSection(670, 300, 180, _BMS_SOC, partialUpdate);                 // Display the battery state of charge

  RECDisplayDataSection(partialUpdate);          // Display the BMS data section, including voltage, current, temperature, and SOC
  
  // RECDisplayMinMaxCellSection(20, 450, _BMS_MinCellVoltage, _BMS_MinCellNum, _BMS_MaxCellVoltage, _BMS_MaxCellNum);  // Display the minimum and maximum cell voltages


}

// #########################################################################################
// Display the Wi-Fi signal strength and battery voltage on the display
// #########################################################################################
void RECDisplayStatusSection(int x, int y, int rssi, bool partialUpdate) 
{
  setFont(OpenSans8B);
  DrawRSSI(x + 305, y + 20, rssi, partialUpdate);
  // DrawBattery(x + 150, y);
}

// #########################################################################################
// Display the top line of the display, including the city name, date and time
// #########################################################################################
void RECDisplayGeneralInfoSection(bool partialUpdate) 
{

  setFont(OpenSans12B);
  String  strDisp = Date_str + "  @   " + Time_str;
  Rect_t area = GetTextBounds(500, 20, strDisp);

  // Clear only the bound area (partial refresh)
  if ( partialUpdate )
    epd_clear_area(area);

  //drawString(15, 20, City, LEFT);
  drawString(500, 20, strDisp, LEFT);
}


void RECDisplayDataSection(bool partialUpdate) 
{
  static float lastVoltage = -1; // Store the last voltage value to detect changes
  static float lastCurrent = -1; // Store the last current value to detect changes
  static float lastTemperature = -1; // Store the last temperature value to detect changes
  static bool  lastChgRelay = false; // Store the last charge relay status to detect changes

  if (partialUpdate && _BMS_Voltage == lastVoltage && _BMS_Current == lastCurrent && _BMS_BattTemperature == lastTemperature && _optoChgRelay == lastChgRelay) 
  {
    return; // No change in voltage, skip drawing
  }
  lastVoltage = _BMS_Voltage;
  lastCurrent = _BMS_Current;
  lastTemperature = _BMS_BattTemperature;
  lastChgRelay = _optoChgRelay;

  // Display the BMS data section, including voltage, current, temperature, and SOC
  int X_COLUMN1 = 20; // X position for the first column
  int X_COLUMN2 = 420; // X position for the second column
  int LINESPACING = 75; // Spacing between lines


  int x = 20;
  int y = 90;

  Rect_t area;
  area.x = x; 
  area.y = y;
  area.width = 400+20; // Width of the area to clear (including some padding)
  area.height = LINESPACING*4 + 10; // Height of the area to clear (including some padding)

// Clear only this area (partial refresh)
  if ( partialUpdate )
    epd_clear_area(area);

  // Draw the voltage section
  int yline = y;
  setFont(OpenSans18B);
  drawString(X_COLUMN1, yline, "Voltage:", LEFT);
  setFont(OpenSans24B);
  drawString(X_COLUMN2, yline, String(_BMS_Voltage, 1) + " V", RIGHT);

  // Draw the current section
  /*
  yline += LINESPACING;
  setFont(OpenSans18B);
  drawString(X_COLUMN1, yline, "Current:", LEFT);
  setFont(OpenSans24B);
  drawString(X_COLUMN2, yline, String(_BMS_Current, 1) + " A", RIGHT);
*/

  // Draw the temperature section
  yline += LINESPACING;
  setFont(OpenSans18B);
  drawString(X_COLUMN1, yline, "Bat.Temp.:", LEFT);
  setFont(OpenSans24B);
  drawString(X_COLUMN2, yline, String(_BMS_BattTemperature) + " °C", RIGHT);

  // Draw the charge relay status section
  yline += LINESPACING;
  setFont(OpenSans18B);
  drawString(X_COLUMN1, yline, "Alt.Enabled:", LEFT);
  setFont(OpenSans24B);
  if ( _optoChgRelay )
  {
    drawString(X_COLUMN2, yline, "ON", RIGHT);
  }
  else
  {
    drawString(X_COLUMN2, yline, "OFF", RIGHT);
  }
  
}


// #########################################################################################
// Display the battery state of charge (SOC) as a circular arc with percentage text
// #########################################################################################
void RECDisplaySOCSection(int x, int y, int cRadius, int soc, bool partialUpdate) 
{
  static int lastSOC = -1; // Store the last SOC value to detect changes
  if (soc == lastSOC && partialUpdate) {
    return; // No change in SOC, skip drawing
  }
  lastSOC = soc;

  Rect_t area;
  area.x = x; 
  area.y = y;
  area.width = cRadius * 2 + 10; // Width of the area to clear (including some padding)
  area.height = cRadius * 2 + 10; // Height of the area to clear (including some padding)

  // Clear only this area (partial refresh)
  if ( partialUpdate )
    epd_clear_area(area);

  int thickness = 35; // Thickness of the SOC arc
  int outerCircleRadius = cRadius;
  int innerCircleRadius = cRadius - thickness;

  // Calculate the angle for the SOC arc based on the percentage
  float angle = map(soc, 0, 100, 0, 360);

  fillCircle(x, y, outerCircleRadius, Black);
  fillCircle(x, y, innerCircleRadius, Black);
  fillCircle(x, y, outerCircleRadius - 1, LightGrey);
  fillCircle(x, y, innerCircleRadius-1, White);
  drawArc(x, y, cRadius, thickness, Black, angle);

  // setFont(OpenSans24B);
  setFont(OpenSans36B);
  drawString(x, y-cRadius/5, String(soc) + "%", CENTER);
} 

void RECDisplayMinMaxCellSection(int x, int y, float minCellVoltage, int minCellNum, float maxCellVoltage, int maxCellNum) {
  setFont(OpenSans10B);
  drawString(x, y, "Min Cell: [#" + String(minCellNum) + "] " + String(minCellVoltage, 3) + " V", LEFT);
  drawString(x, y + 30, "Max Cell: [#" + String(maxCellNum) + "] " + String(maxCellVoltage, 3) + " V", LEFT);
}


// ---------- Arduino lifecycle ----------
void recbms_setup() {

  InitialiseSystem();

  // Connect to REC BMS WebSocket
  // String wsUrl = String("ws://") + _websockets_server_host + ":" + _websockets_server_port + _websockets_server_path;
  /*
  _client.onMessage(onMessageCallback);
  _client.onEvent(onEventsCallback);
  bool connected = _client.connect(_websockets_server_host, _websockets_server_port, _websockets_server_path);
  if(!connected) 
  {
      Serial.println("Connessione al WebSocket fallita.");
      recbms_demo_data(); // Load demo data if connection fails
  }

  */
  recbms_demo_data(); // Load demo data if connection fails

  // Connect to WiFi at sleep cycle time, then get weather and forecast data,  then disconnect to save power
  if (StartWiFi() == WL_CONNECTED && SetupTime() == true) 
  {
      { // Only if received both Weather or Forecast proceed
        StopWiFi();         // Reduces power consumption
        epd_poweron();      // Switch on EPD display
        epd_clear();        // Clear the screen
        DisplayBMS(false);       // Display the BMS data
        edp_update();       // Update the display to show the information
        epd_poweroff_all(); // Switch off all power to EPD
      }
  }

  xTaskCreate(RefreshTask, "RefreshTask", 4096, NULL, 2, NULL);  

  // Sleep until next update, to save power, only if the current time is between WakeupHour and SleepHour
  // BeginSleep();
}

// #########################################################################################
// Callback function to handle incoming messages from the REC BMS
// #########################################################################################
void onMessageCallback(WebsocketsMessage message) 
{
    if (message.isText()) 
    {
        String jsonPayload = message.data();
        
        // Allocazione della memoria per il JSON esteso del REC BMS
        DynamicJsonDocument doc(3072); 
        // JsonDocument doc(); 
        DeserializationError error = deserializeJson(doc, jsonPayload);

        if (error) 
        {
            Serial.print("Errore nel parsing RECBMS JSON: ");
            Serial.println(error.f_str());
            return;
        }

        // 1. Lettura dei dati generali dal blocco "master"
        JsonObject master = doc["bms_array"]["master"];
        
        if (!master.isNull()) 
        {
            // Reset dei valori prima di aggiornare
            _BMS_Voltage = 0.0;          // Display the battery voltage
            _BMS_Current = 0.0;          // Display the battery current
            _BMS_MaxCellVoltage = 0.0;  // Display the minimum and maximum cell voltages
            _BMS_MinCellVoltage = 0.0;
            _BMS_MinCellNum = 0;                // Number of the cell with the minimum voltage
            _BMS_MaxCellNum = 0;                // Number of the cell with the maximum voltage
            _optoChgRelay = false;          // State of the charge relay
            _optoDsgRelay = false;          // State of the discharge relay

// Dati base e SoC
            _BMS_Voltage  = master["vbat"];     // Tensione del banco (V)
            _BMS_Current  = master["ibat"];     // Corrente del banco (A)
            _BMS_SOC = master["soc"];      // State of Charge (%)
            
            // Tensioni Cella Min/Max e i loro rispettivi numeri di cella
            _BMS_MinCellVoltage = master["mincell"];  // Tensione cella minima (V)
            _BMS_MinCellNum = master["mincell_n"];// Numero della cella minima
            _BMS_MaxCellVoltage = master["maxcell"];  // Tensione cella massima (V)
            _BMS_MaxCellNum = master["maxcell_n"];// Numero della cella massima

            // Stati degli Opto-Relay (solitamente mappati come output booleani o interi 0/1)
            // Nota: a seconda del firmware REC BMS, i relè possono trovarsi sotto chiavi come "chg_r" (Charge) e "dsg_r" (Discharge) 
            // oppure generici "relay1", "relay2", "opto1", "opto2". Di seguito la mappatura standard:
            _optoChgRelay = master["chg_r"]; // Stato relè di carica (True/False o 1/0)
            _optoDsgRelay = master["dsg_r"]; // Stato relè di scarica (True/False o 1/0)

            // Stampa i dati aggiornati sul monitor seriale
            Serial.println("\n========== DATI REC BMS AGGIORNATI ==========");
            Serial.printf("Tensione Banco       : %.2f V\n", _BMS_Voltage);
            Serial.printf("Corrente Banco       : %.2f A\n", _BMS_Current);
            Serial.printf("Stato di Carica (SoC): %d %%\n", _BMS_SOC);
            Serial.println("---------------------------------------------");
            Serial.printf("Cella Minima         : [#%d] %.3f V\n", _BMS_MinCellNum, _BMS_MinCellVoltage);
            Serial.printf("Cella Massima        : [#%d] %.3f V\n", _BMS_MaxCellNum, _BMS_MaxCellVoltage);
            Serial.printf("Delta Sbilanciamento : %.3f V\n", (_BMS_MaxCellVoltage - _BMS_MinCellVoltage));
            Serial.println("---------------------------------------------");
            Serial.printf("Opto Relay Carica    : %s\n", _optoChgRelay ? "ATTIVO (ON)" : "DISATTIVO (OFF)");
            Serial.printf("Opto Relay Scarica   : %s\n", _optoDsgRelay ? "ATTIVO (ON)" : "DISATTIVO (OFF)");
            Serial.println("=============================================");
        }
    }
}

// #########################################################################################
// Callback function to handle WebSocket connection events (opened, closed, etc.)
// #########################################################################################
void onEventsCallback(WebsocketsEvent event, String data) 
{
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connessione WebSocket con REC BMS Stabilita!");
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connessione WebSocket Chiusa.");
    }
}

void recbms_loop() 
{
  /*
  // Refresh completely the display onnly after a longer interval
  epd_clear();        // Clear the screen
  DisplayBMS(false);       // Display the BMS data
  edp_update();       // Update the display to show the information

  */


  // Nothing to do: loop is never called because the ESP32 is in deep sleep mode, 
  // but if you want to keep the WebSocket connection alive, you can call _client.poll() here.

  // Wait a bit before the next loop iteration to avoid busy looping
  // _client.poll();
  // delay(5*60*1000);
  // delay(1000);
}

long _numberOfUpdates = 0; // Counter for the number of updates

// A task to refresh the display at regular intervals, with a full refresh every 10 updates
void RefreshTask(void * arg)
{
  while(1)
  {
    recbms_demo_data(); // Load demo data for testing (random values for voltage, current, etc.)

    // Connect to WiFi at sleep cycle time, then get weather and forecast data,  then disconnect to save power
    if (StartWiFi() == WL_CONNECTED && SetupTime() == true) 
    {
      // Receive data 
      // **** @TODO
      // if data receive
    
      StopWiFi();         // Reduces power consumption
      epd_poweron();      // Switch on EPD display

      if ( true )
      // if ( _numberOfUpdates % 10 == 0 )   @TODO: does not seem to work, so always do a full refresh for now
      {
        Serial.println("Full display refresh...");

        // Refresh completely the display onnly after a longer interval
        epd_clear();        // Clear the entire screen
        DisplayBMS(false);  // Display the entire BMS data
        edp_update();       // Update the display to show the information
      } 
      else 
      {
        Serial.println("Partial display refresh...");

        // Refresh only what is changed
        DisplayBMS(true);       // Display the BMS data
        edp_update();       // Update the display to show the information
      }

      epd_poweroff_all(); // Switch off all power to EPD
    }


    _numberOfUpdates++; // Increment the update counter
    Serial.printf("Display refreshed %ld times.\n", _numberOfUpdates);

    delay(120*1000);    // In milliseconds
  }
}


// #########################################################################################
// Setup and loop functions for Arduino, which call the REC BMS setup and loop functions
// #########################################################################################

void setup() {
  recbms_setup();
  // weather_setup();
}

void loop() 
{
  recbms_loop();
  // weather_loop();
}


