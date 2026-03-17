// ═══════════════════════════════════════════════════════════
//   SmartPark TX  v5.0  —  Sensor Node
//   ESP32 + HC-SR04 + Servo + LoRa 433MHz + OLED
//
//   ── CHANGE THESE 2 LINES PER BOARD ──
//   Board 1:  NODE_ID = "S1"   STARTUP_DELAY = 0
//   Board 2:  NODE_ID = "S2"   STARTUP_DELAY = 1000
//   Board 3:  NODE_ID = "S3"   STARTUP_DELAY = 2000
// ═══════════════════════════════════════════════════════════

#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── CHANGE THESE 2 LINES PER BOARD ──────────────────────────
#define NODE_ID        "S1"
#define STARTUP_DELAY  0
// ────────────────────────────────────────────────────────────

// ── PIN DEFINITIONS ──────────────────────────────────────────
#define LORA_SS    32
#define LORA_RST    4
#define LORA_DIO0  26   // GPIO26 — used by LoRa interrupt

#define TRIG_PIN   16
#define ECHO_PIN   17
#define SERVO_PIN  14

#define RGB_RED    25
#define RGB_GREEN  33   // GPIO33 — NOT 26 (that is LORA_DIO0)
#define RGB_BLUE   27

#define OLED_SDA   21
#define OLED_SCL   22
#define OLED_ADDR  0x3C

// ── DETECTION CONFIG ─────────────────────────────────────────
#define DETECT_CM    10     // distance < 10 cm  → OCCUPIED
#define CYCLE_MS   3000     // send packet every 3 seconds

#define ANGLE_OPEN    90    // servo when slot FREE
#define ANGLE_CLOSED   0    // servo when slot OCCUPIED

// ── OBJECTS ──────────────────────────────────────────────────
Servo            barrier;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

uint32_t packetCount = 0;
String   lastStatus  = "";
int      servoAngle  = ANGLE_OPEN;
bool     oledOK      = false;

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

// ── HC-SR04 ──────────────────────────────────────────────────
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1.0;
  return dur * 0.034f / 2.0f;
}

// ── OLED — BOOT ──────────────────────────────────────────────
void oledBoot() {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(20, 3);
  oled.print("SmartPark  v5.0");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(22, 20);
  oled.print("TX NODE");
  oled.setTextSize(1);
  oled.setCursor(46, 42);
  oled.print("ID: ");
  oled.print(NODE_ID);
  oled.drawRoundRect(0, 54, 128, 10, 2, SSD1306_WHITE);
  oled.setCursor(8, 55);
  oled.print("ESP32 + LoRa 433MHz");
  oled.display();
  delay(2000);
}

// ── OLED — MAIN ──────────────────────────────────────────────
void oledUpdate(float dist, String status, bool txOK) {
  oled.clearDisplay();

  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print("Node:");
  oled.print(NODE_ID);
  oled.setCursor(68, 2);
  oled.print("Pkt#");
  oled.print(packetCount);
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 16);
  oled.print("Dist:");
  oled.setTextSize(2);
  oled.setCursor(0, 25);
  if (dist < 0) {
    oled.print("NoEcho");
  } else {
    oled.print(dist, 1);
    oled.print("cm");
  }

  oled.setTextSize(1);
  oled.setCursor(90, 16);
  oled.print("Servo");
  oled.setCursor(90, 26);
  oled.print(servoAngle);
  oled.print((char)247);
  oled.print(servoAngle == ANGLE_OPEN ? "O" : "C");

  if (status == "occupied") {
    oled.fillRoundRect(0, 46, 128, 12, 3, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(28, 49);
    oled.print(">>> OCCUPIED <<<");
    oled.setTextColor(SSD1306_WHITE);
  } else if (status == "free") {
    oled.drawRoundRect(0, 46, 128, 12, 3, SSD1306_WHITE);
    oled.setCursor(38, 49);
    oled.print("-- FREE --");
  }

  oled.setCursor(0, 59);
  oled.print(txOK ? "LoRa:OK" : "LoRa:ERR");
  oled.display();
}

// ── OLED — ERROR ─────────────────────────────────────────────
void oledError(String msg) {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 64, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(2);
  oled.setCursor(26, 4);
  oled.print("ERROR!");
  oled.drawLine(0, 22, 128, 22, SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(4, 28);
  oled.print(msg);
  oled.setCursor(4, 44);
  oled.print("Check wiring");
  oled.display();
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===========================");
  Serial.println(" SmartPark TX  v5.0  ID:" NODE_ID);
  Serial.println("===========================\n");

  pinMode(RGB_RED,   OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE,  OUTPUT);
  rgbBlue();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOK = true;
    Serial.println("[OLED]    OK");
    oledBoot();
  } else {
    Serial.println("[OLED]    Not found — continuing");
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("[HC-SR04] OK");

  barrier.attach(SERVO_PIN);
  barrier.write(ANGLE_OPEN);
  servoAngle = ANGLE_OPEN;
  Serial.println("[Servo]   OK → " + String(ANGLE_OPEN) + "° OPEN");

  SPI.begin(5, 19, 23, 32);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("[LoRa]    INIT FAILED!");
    rgbRed();
    if (oledOK) oledError("LoRa FAIL");
    while (1) { rgbRed(); delay(300); rgbOff(); delay(300); }
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  LoRa.setSyncWord(0xA5);
  LoRa.enableCrc();
  Serial.println("[LoRa]    OK → SF7|BW125|CR4/5|17dBm|Sync:0xA5");

  if (STARTUP_DELAY > 0) {
    Serial.printf("[Offset]  Waiting %dms...\n", STARTUP_DELAY);
    delay(STARTUP_DELAY);
  }

  rgbGreen();
  Serial.println("\n>>> TX Ready — sending every 3s <<<\n");
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  unsigned long t0 = millis();

  // 1. Measure distance
  float dist = getDistance();
  if (dist < 0) Serial.println("[Sensor]  No echo");
  else          Serial.printf("[Sensor]  %.1f cm\n", dist);

  // 2. Decide status + move servo
  String status;
  if (dist > 0 && dist < DETECT_CM) {
    status     = "occupied";
    servoAngle = ANGLE_CLOSED;
    barrier.write(ANGLE_CLOSED);
    rgbRed();
  } else {
    status     = "free";
    servoAngle = ANGLE_OPEN;
    barrier.write(ANGLE_OPEN);
    rgbGreen();
  }

  if (status != lastStatus) {
    Serial.printf("[Status]  Changed → %s  Servo → %d°\n",
                  status.c_str(), servoAngle);
    lastStatus = status;
  }

  // 3. Build and transmit LoRa packet: "S1,free,165.0,90"
  String pkt = String(NODE_ID) + "," +
               status          + "," +
               String(dist, 1) + "," +
               String(servoAngle);

  rgbBlue();
  LoRa.beginPacket();
  LoRa.print(pkt);
  bool sent = LoRa.endPacket();
  packetCount++;

  Serial.printf("[LoRa TX] \"%s\"  Pkt#%lu  %s\n",
                pkt.c_str(), packetCount, sent ? "OK" : "FAIL");

  if (status == "occupied") rgbRed();
  else                       rgbGreen();

  if (oledOK) oledUpdate(dist, status, sent);

  long wait = CYCLE_MS - (long)(millis() - t0);
  if (wait > 0) delay(wait);
}
