/**
 * Simple REC BMS data display
 * Based on LilyGo EPD47 examples, but stripped down:
 * - No web server
 * - No file upload
 * - Just fetches data from the wifi module of REC BMS via WebSocket and displays it.
 */

#ifndef BOARD_HAS_PSRAM
#error "Please enable PSRAM, Arduino IDE -> Tools -> PSRAM -> OPI PSRAM"
#endif

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "epd_driver.h"
#include "firasans.h"
#include "secrets.h"


using namespace websockets;

// ---------- CONFIG ----------
// WiFi (loaded from secrets.h, which is not committed to git)
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// REC BMS  (host/port loaded from secrets.h)
const char* websockets_server_host  = RECBMS_HOST_ADDR;  
const uint16_t websockets_server_port = RECBMS_WEBSOCKET_PORT;
const char* websockets_server_path  = RECBMS_WEBSOCKET_PART;  

// REC BMS entities we created
const char* ENTITY_LINE1 = "sensor.desk_calendar_line_1";
const char* ENTITY_LINE2 = "sensor.desk_calendar_line_2";
const char* ENTITY_CLOCK = "sensor.desk_clock_line";

// Update intervals (ms)
const unsigned long UPDATE_INTERVAL_MS = 30UL * 1000UL;         // 30 seconds

// ---------- Layout (reuse from example) ----------
const Rect_t line1Area = {
    .x = 0,
    .y = 387,
    .width = 960,
    .height = 51,
};
const Rect_t line2Area = {
    .x = 0,
    .y = 438,
    .width = 960,
    .height = 51,
};

const Rect_t clockArea = {
    .x = 0,
    .y = 330,      // adjust as needed
    .width = 960,
    .height = 51,
};

// ---------- Globals ----------
unsigned long lastScreenUpdate = 0;

// Last drawn text for change detection
String lastClockText;
String lastCalLine1Text;
String lastCalLine2Text;

// ---------- WiFi helpers ----------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed");
  }
}

// ---------- WebSocket helpers ----------

WebsocketsClient _client;

// Funzione che gestisce i messaggi in arrivo dal REC BMS
void onMessageCallback(WebsocketsMessage message) {
    if (message.isText()) {
        String jsonPayload = message.data();
        
        // Allocazione della memoria per il JSON esteso del REC BMS
        DynamicJsonDocument doc(3072); 
        DeserializationError error = deserializeJson(doc, jsonPayload);

        if (error) {
            Serial.print("Errore nel parsing JSON: ");
            Serial.println(error.f_str());
            return;
        }

        // 1. Lettura dei dati generali dal blocco "master"
        JsonObject master = doc["bms_array"]["master"];
        
        if (!master.isNull()) {
            // Dati base e SoC
            float voltage  = master["vbat"];     // Tensione del banco (V)
            float current  = master["ibat"];     // Corrente del banco (A)
            int soc        = master["soc"];      // State of Charge (%)
            
            // Tensioni Cella Min/Max e i loro rispettivi numeri di cella
            float minCellV = master["mincell"];  // Tensione cella minima (V)
            int minCellNum = master["mincell_n"];// Numero della cella minima
            float maxCellV = master["maxcell"];  // Tensione cella massima (V)
            int maxCellNum = master["maxcell_n"];// Numero della cella massima

            // Stati degli Opto-Relay (solitamente mappati come output booleani o interi 0/1)
            // Nota: a seconda del firmware REC BMS, i relè possono trovarsi sotto chiavi come "chg_r" (Charge) e "dsg_r" (Discharge) 
            // oppure generici "relay1", "relay2", "opto1", "opto2". Di seguito la mappatura standard:
            bool optoChgRelay = master["chg_r"]; // Stato relè di carica (True/False o 1/0)
            bool optoDsgRelay = master["dsg_r"]; // Stato relè di scarica (True/False o 1/0)

            // Stampa i dati aggiornati sul monitor seriale
            Serial.println("\n========== DATI REC BMS AGGIORNATI ==========");
            Serial.printf("Tensione Banco       : %.2f V\n", voltage);
            Serial.printf("Corrente Banco       : %.2f A\n", current);
            Serial.printf("Stato di Carica (SoC): %d %%\n", soc);
            Serial.println("---------------------------------------------");
            Serial.printf("Cella Minima         : [#%d] %.3f V\n", minCellNum, minCellV);
            Serial.printf("Cella Massima        : [#%d] %.3f V\n", maxCellNum, maxCellV);
            Serial.printf("Delta Sbilanciamento : %.3f V\n", (maxCellV - minCellV));
            Serial.println("---------------------------------------------");
            Serial.printf("Opto Relay Carica    : %s\n", optoChgRelay ? "ATTIVO (ON)" : "DISATTIVO (OFF)");
            Serial.printf("Opto Relay Scarica   : %s\n", optoDsgRelay ? "ATTIVO (ON)" : "DISATTIVO (OFF)");
            Serial.println("=============================================");
        }
    }
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connessione WebSocket con REC BMS Stabilita!");
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connessione WebSocket Chiusa.");
    }
}

// ---------- Drawing helpers ----------
void drawTextInArea(const Rect_t& area, const char* text) {
  int32_t cursor_x = area.x;
  int32_t cursor_y = area.y + FiraSans.advance_y + FiraSans.descender;

  // Clear only this area (partial refresh)
  epd_clear_area(area);

  // Draw using the same font + function the example uses
  writeln((GFXfont *)&FiraSans, text, &cursor_x, &cursor_y, NULL);
}

void drawInitialScreen() {
  // One-time full clear on boot so the panel starts in a known state
  epd_poweron();
  epd_clear();
  epd_poweroff();
}

void drawCalendarScreen(const String& clock, const String& line1, const String& line2) {
  // If nothing changed, don’t bother redrawing this set
  if (clock == lastClockText && line1 == lastCalLine1Text && line2 == lastCalLine2Text) {
    return;
  }

  lastClockText    = clock;
  lastCalLine1Text = line1;
  lastCalLine2Text = line2;

  epd_poweron();

  // Clock line (partial refresh)
  drawTextInArea(clockArea, clock.c_str());

  // Time line (partial refresh)
  drawTextInArea(line1Area, line1.c_str());

  // Title line (partial refresh)
  drawTextInArea(line2Area, line2.c_str());

  epd_poweroff();
}

// Clock-only partial refresh
void drawClockOnly(const String& clock) {
  // Skip if the clock text hasn’t changed
  if (clock == lastClockText) {
    return;
  }

  lastClockText = clock;

  epd_poweron();
  drawTextInArea(clockArea, clock.c_str());
  epd_poweroff();
}

// ---------- Arduino lifecycle ----------
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("\nLilyGo T5 REC BMS Display Starting...");

  // Init display
  epd_init();

  // One-time clear
  drawInitialScreen();

  // Initial WiFi connect
  connectWiFi();

  // Connect to REC BMS WebSocket
  String wsUrl = String("ws://") + websockets_server_host + ":" + websockets_server_port + websockets_server_path;
  _client.onMessage(onMessageCallback);
  _client.onEvent(onEventsCallback);

  bool connected = _client.connect(websockets_server_host, websockets_server_port, websockets_server_path);
  if(!connected) 
  {
      Serial.println("Connessione al WebSocket fallita.");
  }

}

void loop() 
{
  if(_client.available()) 
  {
    _client.poll();
  } 
  else {
    Serial.println("Tentativo di riconnessione...");
    _client.connect(websockets_server_host, websockets_server_port, websockets_server_path);
    delay(5000); 
  }
 

  // Clock-only update every CLOCK_UPDATE_INTERVAL_MS
  unsigned long now = millis();
  if (lastScreenUpdate == 0 || (now - lastScreenUpdate) > UPDATE_INTERVAL_MS) 
  {
    lastScreenUpdate = now;

    // String clock = fetchSensorState(ENTITY_CLOCK);
    // Serial.println("[HA] CLOCK-ONLY: " + clock);

    // drawClockOnly(clock);
  }

  // Wait a bit before the next loop iteration to avoid busy looping
  delay(1000);
}