// ═══════════════════════════════════════════════════════════
//   SmartPark RX  v5.1  —  Gateway Node
//   ESP32 + LoRa 433MHz + WiFi + HiveMQ + Adafruit IO
//
//   Receives LoRa packets from 3 TX sensor boards
//   Publishes to HiveMQ (instant) + Adafruit IO (every 16s)
//   Subscribes to servo-angle feed for remote barrier control
// ═══════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── PIN DEFINITIONS ──────────────────────────────────────────
#define LORA_SS    32
#define LORA_RST    4
#define LORA_DIO0  26

#define SERVO_PIN    14
#define ANGLE_OPEN   90    // Toggle ON  → 90°  OPEN
#define ANGLE_CLOSED  0    // Toggle OFF → 0°   CLOSED

#define RGB_RED    25
#define RGB_GREEN  33      // GPIO33 — NOT 26 (LORA_DIO0)
#define RGB_BLUE   27

#define OLED_SDA   21
#define OLED_SCL   22
#define OLED_ADDR  0x3C

// ── WiFi ─────────────────────────────────────────────────────
const char* ssid     = "mukesh pc";
const char* password = "Muk@2#34";

// ── HiveMQ Cloud  (TLS port 8883) ────────────────────────────
const char* HIVEMQ_HOST = "3800c882c48b47949d040f181e9d6278.s1.eu.hivemq.cloud";
const int   HIVEMQ_PORT = 8883;
const char* HIVEMQ_USER = "esp32user";
const char* HIVEMQ_PASS = "Test1234";

const char* TOPIC_S1 = "parking/slot1/status";
const char* TOPIC_S2 = "parking/slot2/status";
const char* TOPIC_S3 = "parking/slot3/status";

// ── Adafruit IO  (plain TCP port 1883) ───────────────────────
//
//   FEEDS PUBLISHED every 16 seconds:
//     parking-status   →  "S1:FREE,S2:OCCUPIED,S3:FREE|RSSI:-65"
//     slot1-distance   →  "165.0"  cm
//     slot2-distance   →  "7.2"    cm
//     slot3-distance   →  "142.5"  cm
//     rssi             →  "-65"    dBm
//
//   FEED SUBSCRIBED (dashboard → ESP32):
//     servo-angle      →  "1" = OPEN 90°   "0" = CLOSED 0°
//     Toggle block:  ON = 1   OFF = 0
//
//   Rate limit: free tier = 30 pts/min
//   5 feeds × 1 per 16s = 18.75 pts/min  ✅ safe
// ─────────────────────────────────────────────────────────────
const char* AIO_HOST = "io.adafruit.com";
const int   AIO_PORT = 1883;
const char* AIO_USER = "jay6025";
const char* AIO_KEY  = "aio_vGCr59Oh99ad2gKlFWeLiLHJXpmP";

const uint32_t AIO_INTERVAL = 16000;  // publish every 16 seconds

// ── OBJECTS ──────────────────────────────────────────────────
Servo             barrier;
WiFiClientSecure  secClient;
PubSubClient      hivemq(secClient);
WiFiClient        aioClient;
PubSubClient      aio(aioClient);
Adafruit_SSD1306  oled(128, 64, &Wire, -1);

// ── AIO FEED PATHS  (built in setup BEFORE connectAIO) ───────
String AIO_PARKING;
String AIO_S1_DIST;
String AIO_S2_DIST;
String AIO_S3_DIST;
String AIO_RSSI;
String AIO_SERVO;

// ── SLOT DATA ────────────────────────────────────────────────
struct Slot {
  String status = "----";
  float  dist   = 0.0;
  bool   fresh  = false;
};
Slot slots[3];   // [0]=S1  [1]=S2  [2]=S3

uint32_t lastAIO    = 0;
uint32_t pktCount   = 0;
int      lastRSSI   = 0;
int      servoAngle = ANGLE_OPEN;
bool     oledOK     = false;

// ── RGB ──────────────────────────────────────────────────────
void rgb(bool r, bool g, bool b) {
  digitalWrite(RGB_RED,   r);
  digitalWrite(RGB_GREEN, g);
  digitalWrite(RGB_BLUE,  b);
}
void rgbRed()    { rgb(1,0,0); }
void rgbGreen()  { rgb(0,1,0); }
void rgbBlue()   { rgb(0,0,1); }
void rgbYellow() { rgb(1,1,0); }
void rgbOff()    { rgb(0,0,0); }

// ── OLED — BOOT ──────────────────────────────────────────────
void oledBoot() {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(20, 3);
  oled.print("SmartPark  v5.1");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(16, 20);
  oled.print("GATEWAY");
  oled.setTextSize(1);
  oled.setCursor(10, 42);
  oled.print("HiveMQ + AIO");
  oled.drawRoundRect(0, 54, 128, 10, 2, SSD1306_WHITE);
  oled.setCursor(8, 55);
  oled.print("ESP32 + LoRa 433MHz");
  oled.display();
  delay(2000);
}

// ── OLED — CONNECTING ────────────────────────────────────────
void oledConnecting(String msg) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 4);
  oled.print("Connecting...");
  oled.setCursor(0, 22);
  oled.print(msg);
  oled.display();
}

// ── OLED — DASHBOARD ─────────────────────────────────────────
void oledDash() {
  oled.clearDisplay();

  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print("SmartPark  Pkt#");
  oled.print(pktCount);
  oled.setTextColor(SSD1306_WHITE);

  const char* labels[3] = {"S1", "S2", "S3"};
  for (int i = 0; i < 3; i++) {
    int y = 15 + i * 14;
    oled.setCursor(0, y);
    oled.print(labels[i]);
    oled.print(":");
    if (slots[i].status == "occupied") {
      oled.fillRoundRect(18, y-1, 56, 10, 2, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
      oled.setCursor(20, y);
      oled.print("OCCUPIED");
      oled.setTextColor(SSD1306_WHITE);
    } else if (slots[i].status == "free") {
      oled.drawRoundRect(18, y-1, 32, 10, 2, SSD1306_WHITE);
      oled.setCursor(22, y);
      oled.print("FREE");
    } else {
      oled.setCursor(22, y);
      oled.print("----");
    }
    oled.setCursor(80, y);
    if (slots[i].dist > 0) { oled.print(slots[i].dist, 1); oled.print("cm"); }
    else                      oled.print("--cm");
  }

  oled.setCursor(0, 57);
  oled.print("Servo:");
  oled.print(servoAngle);
  oled.print((char)247);
  oled.print(" ");
  oled.print(servoAngle == ANGLE_OPEN ? "OPEN" : "CLOSED");
  oled.setCursor(88, 57);
  oled.print("R:");
  oled.print(lastRSSI);
  oled.display();
}

// ── WiFi CONNECT ─────────────────────────────────────────────
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  if (oledOK) oledConnecting("WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 30000) { Serial.println("\n[WiFi] Timeout!"); ESP.restart(); }
  }
  Serial.println();
  Serial.print("[WiFi] Connected! IP: ");
  Serial.println(WiFi.localIP());
}

// ── HiveMQ CONNECT ───────────────────────────────────────────
void connectHiveMQ() {
  if (oledOK) oledConnecting("HiveMQ TLS...");
  while (!hivemq.connected()) {
    Serial.print("[HiveMQ] Connecting...");
    if (hivemq.connect("SmartPark-GW-v51", HIVEMQ_USER, HIVEMQ_PASS)) {
      Serial.println(" Connected!");
    } else {
      Serial.printf(" Failed rc=%d, retry 3s\n", hivemq.state());
      delay(3000);
    }
  }
}

// ── AIO CALLBACK — fires when Toggle pressed on dashboard ────
void aioCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.println("\n[AIO RX] ─────────────────────────────");
  Serial.println("[AIO RX] Topic  : " + String(topic));
  Serial.println("[AIO RX] Message: " + msg);

  String t = String(topic);
  if (t != AIO_SERVO && t != "/" + AIO_SERVO) return;

  String up = msg;
  up.toUpperCase();

  // "1" or "ON" or "90" → OPEN 90°
  // "0" or "OFF"        → CLOSED 0°
  int angle;
  if (up == "1" || up == "ON" || msg == "90") {
    angle = ANGLE_OPEN;
  } else {
    angle = ANGLE_CLOSED;
  }

  barrier.write(angle);
  servoAngle = angle;

  if (angle == ANGLE_OPEN) {
    rgbGreen();
    Serial.println("[Servo]  → 90° OPEN ✓");
  } else {
    rgbRed();
    Serial.println("[Servo]  → 0° CLOSED ✓");
  }
  if (oledOK) oledDash();
}

// ── AIO CONNECT ──────────────────────────────────────────────
void connectAIO() {
  if (oledOK) oledConnecting("Adafruit IO...");
  int retries = 0;
  while (!aio.connected()) {
    Serial.print("[AIO] Connecting...");
    String cid = "SmartPark-" + String(random(0xffff), HEX);
    if (aio.connect(cid.c_str(), AIO_USER, AIO_KEY)) {
      Serial.println(" Connected!");
      bool ok = aio.subscribe(AIO_SERVO.c_str(), 1);
      Serial.println("[AIO] Subscribe servo-angle: " + String(ok ? "OK" : "FAILED"));
      // Publish current servo state so dashboard toggle shows correct position
      aio.publish(AIO_SERVO.c_str(), servoAngle == ANGLE_OPEN ? "1" : "0");
      Serial.println("[AIO] Initial servo state → " + String(servoAngle == ANGLE_OPEN ? "1 (OPEN)" : "0 (CLOSED)"));
    } else {
      Serial.printf(" Failed rc=%d\n", aio.state());
      if (++retries > 10) { Serial.println("[AIO] Check AIO_USER / AIO_KEY / feed names"); retries = 0; }
      delay(3000);
    }
  }
}

// ── AIO PUBLISH HELPER ───────────────────────────────────────
//   Non-blocking 350ms gap — keeps MQTT alive so servo toggle
//   is NEVER missed during sendAIO()
// ─────────────────────────────────────────────────────────────
bool aioPublish(const String& feed, const String& value) {
  if (!aio.connected()) connectAIO();

  aio.loop();
  hivemq.loop();

  bool ok = aio.publish(feed.c_str(), value.c_str());
  Serial.printf("[AIO] %-38s : %-10s %s\n",
                feed.c_str(), value.c_str(), ok ? "✓" : "✗ FAILED");

  // Non-blocking 350ms — processes toggle during wait
  uint32_t t = millis();
  while (millis() - t < 350) {
    aio.loop();
    hivemq.loop();
    yield();
  }
  return ok;
}

// ── SEND ALL FEEDS ───────────────────────────────────────────
void sendAIO() {
  Serial.println("\n[AIO] ═══ Publishing all feeds ═══");

  // 1. parking-status
  String s1 = (slots[0].status == "occupied") ? "OCCUPIED" : "FREE";
  String s2 = (slots[1].status == "occupied") ? "OCCUPIED" : "FREE";
  String s3 = (slots[2].status == "occupied") ? "OCCUPIED" : "FREE";
  String combined = "S1:" + s1 + ",S2:" + s2 + ",S3:" + s3
                    + "|RSSI:" + String(lastRSSI);
  aioPublish(AIO_PARKING, combined);

  // 2. slot1-distance
  aioPublish(AIO_S1_DIST, slots[0].dist > 0 ? String(slots[0].dist, 1) : "0");

  // 3. slot2-distance
  aioPublish(AIO_S2_DIST, slots[1].dist > 0 ? String(slots[1].dist, 1) : "0");

  // 4. slot3-distance
  aioPublish(AIO_S3_DIST, slots[2].dist > 0 ? String(slots[2].dist, 1) : "0");

  // 5. rssi
  aioPublish(AIO_RSSI, String(lastRSSI));

  Serial.println("[AIO] ═══ All feeds done ═══\n");

  slots[0].fresh = false;
  slots[1].fresh = false;
  slots[2].fresh = false;
}

// ── PARSE LORA PACKET ────────────────────────────────────────
//   v5 format:  "S1,occupied,7.2,0"
//   v3/v4:      "S1,occupied,7.2"
//   Legacy:     "free" / "occupied"
// ─────────────────────────────────────────────────────────────
void parsePacket(String msg, int rssi) {
  msg.trim();
  lastRSSI = rssi;

  int c1 = msg.indexOf(',');
  if (c1 < 0) {
    slots[0].status = msg;
    slots[0].dist   = 0.0;
    slots[0].fresh  = true;
    if (hivemq.connected()) hivemq.publish(TOPIC_S1, msg.c_str());
    Serial.printf("[Legacy] %-8s RSSI:%d\n", msg.c_str(), rssi);
    return;
  }

  int c2 = msg.indexOf(',', c1 + 1);
  int c3 = msg.indexOf(',', c2 + 1);

  String nodeId = msg.substring(0, c1);
  String status = msg.substring(c1 + 1, c2);
  float  dist   = (c3 > 0) ? msg.substring(c2+1, c3).toFloat()
                            : msg.substring(c2+1).toFloat();

  int         idx   = -1;
  const char* topic = nullptr;
  if      (nodeId == "S1") { idx = 0; topic = TOPIC_S1; }
  else if (nodeId == "S2") { idx = 1; topic = TOPIC_S2; }
  else if (nodeId == "S3") { idx = 2; topic = TOPIC_S3; }
  else { Serial.println("[Parser] Unknown node: " + nodeId); return; }

  slots[idx].status = status;
  slots[idx].dist   = dist;
  slots[idx].fresh  = true;

  if (!hivemq.connected()) connectHiveMQ();
  hivemq.publish(topic, status.c_str());

  Serial.printf("[%s] %-8s | %5.1fcm | RSSI:%d\n",
                nodeId.c_str(), status.c_str(), dist, rssi);
  Serial.printf("  All → S1:%-8s %.1fcm  S2:%-8s %.1fcm  S3:%-8s %.1fcm\n",
                slots[0].status.c_str(), slots[0].dist,
                slots[1].status.c_str(), slots[1].dist,
                slots[2].status.c_str(), slots[2].dist);
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n==========================================");
  Serial.println("  SmartPark RX GATEWAY  v5.1  |  3-Slot");
  Serial.println("  HiveMQ : TLS 8883  (instant publish)");
  Serial.println("  AIO    : TCP 1883  (every 16 seconds)");
  Serial.println("==========================================\n");

  // STEP 1 — Build AIO feed paths FIRST (before connectAIO)
  AIO_PARKING = String(AIO_USER) + "/feeds/parking-status";
  AIO_S1_DIST = String(AIO_USER) + "/feeds/slot1-distance";
  AIO_S2_DIST = String(AIO_USER) + "/feeds/slot2-distance";
  AIO_S3_DIST = String(AIO_USER) + "/feeds/slot3-distance";
  AIO_RSSI    = String(AIO_USER) + "/feeds/rssi";
  AIO_SERVO   = String(AIO_USER) + "/feeds/servo-angle";

  Serial.println("[AIO] Feed paths built:");
  Serial.println("  TX → " + AIO_PARKING);
  Serial.println("  TX → " + AIO_S1_DIST);
  Serial.println("  TX → " + AIO_S2_DIST);
  Serial.println("  TX → " + AIO_S3_DIST);
  Serial.println("  TX → " + AIO_RSSI);
  Serial.println("  RX ← " + AIO_SERVO + "\n");

  // STEP 2 — GPIO
  pinMode(RGB_RED,   OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE,  OUTPUT);
  rgbBlue();

  // STEP 3 — OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOK = true;
    Serial.println("[OLED]  OK");
    oledBoot();
  } else {
    Serial.println("[OLED]  Not found — continuing");
  }

  // STEP 4 — Servo start OPEN
  barrier.attach(SERVO_PIN);
  barrier.write(ANGLE_OPEN);
  servoAngle = ANGLE_OPEN;
  Serial.println("[Servo] OK → " + String(ANGLE_OPEN) + "° OPEN");

  // STEP 5 — WiFi
  rgbYellow();
  connectWiFi();

  // STEP 6 — HiveMQ TLS
  secClient.setInsecure();
  hivemq.setServer(HIVEMQ_HOST, HIVEMQ_PORT);
  hivemq.setKeepAlive(60);
  connectHiveMQ();

  // STEP 7 — Adafruit IO
  aio.setServer(AIO_HOST, AIO_PORT);
  aio.setBufferSize(512);
  aio.setKeepAlive(60);
  aio.setCallback(aioCallback);
  connectAIO();

  // STEP 8 — LoRa
  SPI.begin(5, 19, 23, 32);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("[LoRa]  INIT FAILED!");
    rgbRed();
    if (oledOK) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(10, 20);
      oled.print("LoRa FAIL");
      oled.display();
    }
    while (1) { rgbRed(); delay(300); rgbOff(); delay(300); }
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xA5);
  LoRa.enableCrc();
  Serial.println("[LoRa]  OK → SF7|BW125|CR4/5|Sync:0xA5");

  rgbGreen();
  Serial.println("\n>>> Gateway ready — listening for LoRa packets...\n");
  if (oledOK) oledDash();
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {

  // HiveMQ keepalive
  if (!hivemq.connected()) {
    rgbYellow();
    Serial.println("[HiveMQ] Reconnecting...");
    connectHiveMQ();
    rgbGreen();
  }
  hivemq.loop();

  // Adafruit IO keepalive
  if (!aio.connected()) {
    Serial.println("[AIO] Reconnecting...");
    connectAIO();
  }
  aio.loop();  // ← this fires aioCallback when toggle is pressed

  // LoRa receive
  int pktSize = LoRa.parsePacket();
  if (pktSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    int rssi = LoRa.packetRssi();
    pktCount++;

    rgbBlue();
    Serial.println("\n──────────────────────────────────────");
    Serial.printf("[LoRa RX] Pkt#%lu  \"%s\"  RSSI:%ddBm\n",
                  pktCount, msg.c_str(), rssi);
    parsePacket(msg, rssi);
    rgbGreen();
    if (oledOK) oledDash();
    aio.loop();  // pump AIO right after packet — catches any toggle waiting
  }

  // Publish to Adafruit IO every 16s (only when fresh data exists)
  bool anyFresh = slots[0].fresh || slots[1].fresh || slots[2].fresh;
  if (anyFresh && (millis() - lastAIO >= AIO_INTERVAL)) {
    sendAIO();
    lastAIO = millis();
  }
}
