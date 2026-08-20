/*
 * A-B Line Precision Planter Controller
 * 
 * ESP32-based GPS planter row control system using RTK GPS.
 * Calculates perpendicular distance from an A-B baseline to trigger
 * planter row solenoid pulses at configured intervals.
 * 
 * Hardware: ESP32 + RTK GPS Module (on Serial2: RX2=GPIO16, TX2=GPIO17)
 * Output: GPIO 23 to optocoupled relay/MOSFET for solenoid control
 * 
 * Dependencies: TinyGPS++, ArduinoJson v7+ (install via Arduino Library Manager)
 *
 * Configuration and A-B line coordinates are persisted as a JSON file
 * (/config.json) on the ESP32 internal LittleFS flash partition, so all
 * settings survive power cycles.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <math.h>
#include <TinyGPS++.h>

// --- WiFi AP Configuration ---
const char* ssid = "Tractor_Planter_GPS";
const char* password = "agri-precision";

// --- Server & Storage ---
WebServer server(80);
const char* CONFIG_PATH = "/config.json";

// --- GPS Parser ---
TinyGPSPlus gps;
TinyGPSCustom ggaFixQuality(gps, "GPGGA", 6);

// --- Configuration Variables ---
double distanceInterval;
double distanceOffset;
unsigned long pulseTime;
double timeLookahead;

// --- A-B Line Coordinates ---
double latA, lonA, latB, lonB;
bool lineReady = false;

// --- Hardware and Tracking ---
const int OUTPUT_PIN = 23;
long lastIntervalIndex = -1;
unsigned long pulseEndTime = 0;
bool pulseActive = false;

HardwareSerial GPSSerial(2);

// --- Conversion Constants ---
const double LAT_TO_METERS = 111132.95;
double lonToMeters = 0.0;
int liveFixType = 0;

// --- Live Telemetry (updated every GPS fix) ---
double lastLat = 0.0;
double lastLon = 0.0;
double lastSpeedMPS = 0.0;
int lastSatellites = 0;
unsigned long lastFixMillis = 0;
unsigned long pulseCount = 0;

// ---------------------------------------------------------------------------
// Config Persistence (JSON file on internal LittleFS flash)
// ---------------------------------------------------------------------------
bool saveConfig() {
  JsonDocument doc;
  doc["interval_m"]  = distanceInterval;
  doc["offset_m"]    = distanceOffset;
  doc["pulse_ms"]    = pulseTime;
  doc["lookahead_s"] = timeLookahead;
  doc["line_a_lat"]  = latA;
  doc["line_a_lon"]  = lonA;
  doc["line_b_lat"]  = latB;
  doc["line_b_lon"]  = lonB;

  File file = LittleFS.open(CONFIG_PATH, "w");
  if (!file) {
    Serial.println("Config save failed: cannot open " + String(CONFIG_PATH));
    return false;
  }

  bool ok = serializeJson(doc, file) > 0;
  file.close();
  if (!ok) Serial.println("Config save failed: write error");
  return ok;
}

void loadConfig() {
  if (!LittleFS.exists(CONFIG_PATH)) return;

  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) return;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.print("Config restore failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Missing keys fall back to current (default) values
  distanceInterval = doc["interval_m"]  | distanceInterval;
  distanceOffset   = doc["offset_m"]    | distanceOffset;
  pulseTime        = doc["pulse_ms"]    | pulseTime;
  timeLookahead    = doc["lookahead_s"] | timeLookahead;
  latA             = doc["line_a_lat"]  | latA;
  lonA             = doc["line_a_lon"]  | lonA;
  latB             = doc["line_b_lat"]  | latB;
  lonB             = doc["line_b_lon"]  | lonB;

  Serial.println("Configuration restored from " + String(CONFIG_PATH));
}

// ---------------------------------------------------------------------------
// Web Interface
// ---------------------------------------------------------------------------
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>"
          "body{font-family:Arial;margin:20px;background:#f4f4f4;}"
          ".card{background:white;padding:20px;border-radius:8px;max-width:400px;margin:auto;}"
          "input[type='number']{width:100%;padding:8px;margin:5px 0 15px 0;box-sizing:border-box;}"
          "input[type='submit'],.btn{width:100%;padding:12px;background:#28a745;color:white;border:none;"
          "border-radius:4px;font-size:16px;cursor:pointer;margin-bottom:10px;text-align:center;display:block;text-decoration:none;}"
          ".btn-blue{background:#007bff;width:94%;}"
          ".btn-red{background:#dc3545;}"
          ".status{background:#eee;padding:10px;border-radius:4px;font-size:13px;margin-bottom:15px;}"
          "</style></head><body>";

  html += "<div class='card'><h2>A-B Line Planter Control</h2>";

  // Dashboard Status Box
  String fixStr = "NO FIX";
  if (liveFixType == 1) fixStr = "Standard GPS (No RTK)";
  if (liveFixType == 2) fixStr = "DGPS / WAAS";
  if (liveFixType == 4) fixStr = "RTK FIXED (Precision Mode)";
  if (liveFixType == 5) fixStr = "RTK FLOAT";

  html += "<div class='status'><strong>GPS Link:</strong> " + fixStr + " (" + String(gps.satellites.value()) + " Sats)<br><hr>";
  html += "<a class='btn btn-blue' href='/status'>Status</a>";
  html += "<strong>Line A-B Status:</strong> " + String(lineReady ? "Configured &amp; Live" : "NOT CONFIGURED") + "</div>";

  // Action Buttons
  if (!lineReady) {
    html += "<a class='btn btn-blue' href='/setA'>Mark Point A (Current Fix)</a>";
    html += "<a class='btn btn-blue' href='/setB'>Mark Point B (Current Fix)</a>";
  } else {
    html += "<a class='btn btn-red' href='/resetLines' onclick=\"return confirm('Clear current A-B field lines?');\">Reset Field Lines</a>";
  }

  // Config Form
  html += "<form action='/save' method='POST'>";
  html += "<label>Interval Between Rows (m):</label><input type='number' name='interval' step='0.01' value='" + String(distanceInterval, 2) + "'>";
  html += "<label>Antenna-to-Planter Offset (m):</label><input type='number' name='dist_offset' step='0.01' value='" + String(distanceOffset, 2) + "'>";
  html += "<label>Solenoid Pulse (ms):</label><input type='number' name='pulse' value='" + String(pulseTime) + "'>";
  html += "<label>Look-Ahead Delay (s):</label><input type='number' name='lookahead' step='0.001' value='" + String(timeLookahead, 3) + "'>";
  html += "<input type='submit' value='Apply Changes'></form></div></body></html>";

  server.send(200, "text/html", html);
}

void handleSetA() {
  if (liveFixType == 4 && gps.location.isValid()) {
    latA = gps.location.lat();
    lonA = gps.location.lng();
    saveConfig();
    server.send(200, "text/html", "<html><body><script>alert('Point A baseline anchored!');window.location.href='/';</script></body></html>");
  } else {
    server.send(200, "text/html", "<html><body><script>alert('Error: Requires RTK Fixed lock!');window.location.href='/';</script></body></html>");
  }
}

void handleSetB() {
  if (liveFixType == 4 && gps.location.isValid()) {
    latB = gps.location.lat();
    lonB = gps.location.lng();
    saveConfig();
    lineReady = (latA != 0.0 && latB != 0.0);
    lastIntervalIndex = -1;
    server.send(200, "text/html", "<html><body><script>alert('Point B baseline anchored!');window.location.href='/';</script></body></html>");
  } else {
    server.send(200, "text/html", "<html><body><script>alert('Error: Requires RTK Fixed lock!');window.location.href='/';</script></body></html>");
  }
}

void handleResetLines() {
  latA = 0.0; lonA = 0.0;
  latB = 0.0; lonB = 0.0;
  saveConfig();

  lineReady = false;
  lastIntervalIndex = -1;

  server.send(200, "text/html", "<html><body><script>alert('Field data wiped.');window.location.href='/';</script></body></html>");
}

void handleStatus() {
  String fixStr = "NO FIX";
  if (liveFixType == 1) fixStr = "Standard GPS";
  if (liveFixType == 2) fixStr = "DGPS / WAAS";
  if (liveFixType == 4) fixStr = "RTK FIXED";
  if (liveFixType == 5) fixStr = "RTK FLOAT";

  String json = "{";
  json += "\"fix_type\":\"" + fixStr + "\",";
  json += "\"fix_quality\":" + String(liveFixType) + ",";
  json += "\"satellites\":" + String(lastSatellites) + ",";
  json += "\"latitude\":" + String(lastLat, 7) + ",";
  json += "\"longitude\":" + String(lastLon, 7) + ",";
  json += "\"speed_mps\":" + String(lastSpeedMPS, 2) + ",";
  json += "\"line_ready\":" + String(lineReady ? "true" : "false") + ",";
  json += "\"line_a_lat\":" + String(latA, 7) + ",";
  json += "\"line_a_lon\":" + String(lonA, 7) + ",";
  json += "\"line_b_lat\":" + String(latB, 7) + ",";
  json += "\"line_b_lon\":" + String(lonB, 7) + ",";
  json += "\"interval_m\":" + String(distanceInterval, 2) + ",";
  json += "\"offset_m\":" + String(distanceOffset, 2) + ",";
  json += "\"pulse_ms\":" + String(pulseTime) + ",";
  json += "\"lookahead_s\":" + String(timeLookahead, 3) + ",";
  json += "\"pulse_count\":" + String(pulseCount) + ",";
  json += "\"pulse_active\":" + String(pulseActive ? "true" : "false") + ",";
  json += "\"last_fix_age_ms\":" + String(millis() - lastFixMillis) + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";

  server.send(200, "application/json", json);
}

void handleSave() {
  if (server.hasArg("interval"))   distanceInterval = server.arg("interval").toDouble();
  if (server.hasArg("dist_offset")) distanceOffset = server.arg("dist_offset").toDouble();
  if (server.hasArg("pulse"))      pulseTime = server.arg("pulse").toInt();
  if (server.hasArg("lookahead"))  timeLookahead = server.arg("lookahead").toDouble();

  saveConfig();

  server.send(200, "text/html", "<html><body><script>alert('Settings Saved Successfully');window.location.href='/';</script></body></html>");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  GPSSerial.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // Mount internal flash filesystem (format on first boot / corruption)
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed - running with defaults");
  }

  // Defaults, overridden by /config.json when present
  distanceInterval = 0.75;
  distanceOffset   = 1.25;
  pulseTime        = 100;
  timeLookahead    = 0.05;
  latA = 0.0; lonA = 0.0;
  latB = 0.0; lonB = 0.0;

  loadConfig();

  lineReady = (latA != 0.0 && latB != 0.0);

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/setA", handleSetA);
  server.on("/setB", handleSetB);
  server.on("/resetLines", handleResetLines);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  Serial.println("A-B Line Planter Controller started");
  Serial.print("AP SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  server.handleClient();

  // Turn off solenoid pulse after duration expires
  if (pulseActive && millis() >= pulseEndTime) {
    digitalWrite(OUTPUT_PIN, LOW);
    pulseActive = false;
  }

  // Feed GPS data into TinyGPS++ parser
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Update fix quality from GGA sentence
  if (ggaFixQuality.isUpdated()) {
    liveFixType = atoi(ggaFixQuality.value());
  }

  // Process new location fix
  if (gps.location.isUpdated() && gps.location.isValid()) {
    // Always store last known telemetry
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    lastSpeedMPS = gps.speed.mps();
    lastSatellites = gps.satellites.value();
    lastFixMillis = millis();

    // Only run the pulse logic when RTK is fixed and line is configured
    if (liveFixType != 4 || !lineReady) return;

    double currentLat = lastLat;
    double currentLon = lastLon;
    double currentSpeedMPS = lastSpeedMPS;

    // Localize longitude scaling to current latitude
    lonToMeters = cos(currentLat * M_PI / 180.0) * 111319.9;

    // Project A, B, and tractor position into local meters (A is origin)
    double xB = (lonB - lonA) * lonToMeters;
    double yB = (latB - latA) * LAT_TO_METERS;
    double xP = (currentLon - lonA) * lonToMeters;
    double yP = (currentLat - latA) * LAT_TO_METERS;

    // Perpendicular distance via cross product / line magnitude
    double AB_mag = sqrt((xB * xB) + (yB * yB));
    if (AB_mag < 0.1) return;

    double perpDistance = ((xP * (-yB)) + (yP * xB)) / AB_mag;
    perpDistance = fabs(perpDistance);

    // Apply physical offset and look-ahead correction
    double correctedDistance = perpDistance + distanceOffset + (currentSpeedMPS * timeLookahead);
    long currentIntervalIndex = floor(correctedDistance / distanceInterval);

    // Fire pulse when crossing into a new interval lane
    if (lastIntervalIndex != -1 && currentIntervalIndex > lastIntervalIndex && !pulseActive) {
      Serial.print("Target crossed! Fired Pulse Index: ");
      Serial.println(currentIntervalIndex + 1);
      digitalWrite(OUTPUT_PIN, HIGH);
      pulseActive = true;
      pulseEndTime = millis() + pulseTime;
      pulseCount++;
    }
    lastIntervalIndex = currentIntervalIndex;
  }
}
