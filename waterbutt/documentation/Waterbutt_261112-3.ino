#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h> 
#include <Servo.h>

// Your permanent network credentials
const char* ssid     = "####";
const char* password = "####";

ESP8266WebServer server(80);
Servo pinchValve;

const int OPEN_PULSE   = 700;   
const int CLOSED_PULSE = 2300;  

// Fail-safe variables
unsigned long valveOpenedAt = 0;
bool valveIsOpen = false;
const unsigned long FAILSAFE_TIMEOUT = 5000; // 5-second automatic timeout

// Wi-Fi Connection Watchdog
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000; // Check network health every 10 seconds

// Variables for the non-blocking LED status heartbeat
unsigned long lastLEDUpdate = 0;
bool ledState = false; 

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<style>html { font-family: sans-serif; text-align: center;}";
  html += ".btn { display: inline-block; padding: 15px 30px; font-size: 24px; margin: 20px; cursor: pointer; text-decoration: none; color: white; border-radius: 5px; }";
  html += ".open { background-color: #2ecc71; } .close { background-color: #e74c3c; }</style></head>";
  html += "<body><h1>Water Butt Controller</h1>";
  html += "<h2 style=\"color: " + String(valveIsOpen ? "#2ecc71" : "#e74c3c") + ";\">";
  html += "VALVE STATUS: " + String(valveIsOpen ? "OPEN" : "CLOSED") + "</h2>";
  html += "<a href=\"/open\" class=\"btn open\">OPEN VALVE</a>";
  html += "<a href=\"/close\" class=\"btn close\">CLOSE VALVE</a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleOpen() {
  // 1. Wake up the control line
  pinchValve.attach(D2, 500, 2500); 
  delay(10); // Tiny stability pause
  
  // 2. Drive the cam to crush open
  pinchValve.writeMicroseconds(OPEN_PULSE);
  delay(500); // Give the physical brass gears half a second to finish moving
  
  // 3. Put the motor to sleep
  pinchValve.detach(); 
  
  valveOpenedAt = millis(); 
  valveIsOpen = true;
  server.sendHeader("Location", "/"); 
  server.send(303);
}

void handleClose() {
  pinchValve.attach(D2, 500, 2500); 
  delay(10);
  
  pinchValve.writeMicroseconds(CLOSED_PULSE);
  delay(500); 
  
  pinchValve.detach(); // Total silence, zero holding current!
  
  valveIsOpen = false; 
  server.sendHeader("Location", "/");
  server.send(303);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn off LED (Inverted logic)

  WiFi.hostname("waterbutt"); 
  WiFi.setAutoReconnect(true); // Let the native network stack handle dropouts

  // \n handles the bootloader noise cleanly
  Serial.print("\nScanning and connecting to Wi-Fi");
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected successfully!");

  if (MDNS.begin("waterbutt")) {
    Serial.println("mDNS responder started! Access at http://waterbutt.local");
  }

  server.on("/", handleRoot);
  server.on("/open", handleOpen);
  server.on("/close", handleClose);
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- WI-FI LIVENESS WATCHDOG ---
  // Every 10 seconds, make sure we haven't entered a "ghost connection" state
  if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = currentMillis;
    
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP().toString() == "0.0.0.0") {
      Serial.println("Network link dead. Native stack attempting recovery...");
      WiFi.begin(ssid, password); // Force native stack re-association to the AP
    }
  }

  // Define true network readiness based on status AND valid IP address allocation
  bool networkReady = (WiFi.status() == WL_CONNECTED && WiFi.localIP().toString() != "0.0.0.0");
  
  if (networkReady) {
    MDNS.update(); 
    server.handleClient(); 
  }

  // --- AUTOMATIC WATCHDOG/FAIL-SAFE ---
  if (valveIsOpen && (millis() - valveOpenedAt >= FAILSAFE_TIMEOUT)) {
    // Wake up, shut the valve, then go back to sleep
    pinchValve.attach(D2, 500, 2500);
    delay(10);
    
    pinchValve.writeMicroseconds(CLOSED_PULSE);
    delay(500);
    
    pinchValve.detach();
    
    valveIsOpen = false;
    Serial.println("Failsafe triggered: Valve closed and detached.");
  }

  // --- NON-BLOCKING HEARTBEAT LED (Two-State Mode) ---
  unsigned long interval = 0;
  if (networkReady) {
    interval = ledState ? 100 : 1900; // Snappy flash (Connected and waiting)
  } else {
    interval = ledState ? 1000 : 1000; // Even 1s on/off flash (Disconnected/Reconnecting)
  }

  if (currentMillis - lastLEDUpdate >= interval) {
    lastLEDUpdate = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);  
  }
}
