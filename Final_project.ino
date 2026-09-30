// =============================================================
//  SOLTRACK — Dual-Axis Solar Tracker
//  with Storm-Safe Wind Shutdown, Panel Power Monitoring,
//  and ST7789 1.14" TFT Display (135x240)
// =============================================================

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ESP32Servo.h>
#include <DHTesp.h>
#include "model_azimuth.h"
#include "model_zenith.h"

// ---------- TFT Display Pins ----------
#define TFT_CS    14
#define TFT_RST   26
#define TFT_DC    25
#define TFT_DIN   13    // = MOSI
#define TFT_CLK   17    // = SCLK  (was 18, moved because occupied)

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_DIN, TFT_CLK, TFT_RST);

// ---------- Other Pins ----------
#define DHTPIN        4
#define PIN_LDR1      34
#define PIN_LDR2      35
#define PIN_LDR3      32
#define PIN_LDR4      33
#define PIN_SERVO_AZ  23
#define PIN_SERVO_ZN  19
#define LED_RED       15
#define LED_GREEN     16
#define PIN_ACS712    27
#define PIN_PANEL_V   39

// ---------- Objects ----------
DHTesp dht;
Servo servoAz;
Servo servoZn;

// ---------- Panel Measurement ----------
const float V_REF         = 3.3f;
const int   ADC_MAX       = 4095;
const float ACS_DIV_RATIO = 1.0f / 3.0f;   // 1k + 1k top, 1k bottom
const float V_DIV_RATIO   = 2.0f;          // 10k + 10k
const float ACS_SENS_RAW  = 0.185f;        // ACS712-05B = 185mV/A
const float ACS_SENS_EFF  = ACS_SENS_RAW * ACS_DIV_RATIO;
const int   CAL_SAMPLES   = 200;
float zero_offset = 0.0f;

// ---------- Servo Tunables ----------
const int AZ_CENTER = 90, ZN_CENTER = 90;
const int AZ_MIN = 10,  AZ_MAX = 170;
const int ZN_MIN = 20,  ZN_MAX = 160;
const float AZ_SWING = 100.0f;
const float ZN_SWING =  80.0f;

// ---------- Wind Detection Tunables ----------
const int   WINDOW          = 5;
const float WIND_HUM_DELTA  = 3.0f;
const float WIND_TEMP_DELTA = 0.8f;
const unsigned long HOLD_MS = 4000;

// ---------- Wind State ----------
float temp_history[WINDOW];
float hum_history[WINDOW];
int   history_idx    = 0;
int   history_count  = 0;
bool  wind_detected  = false;
unsigned long wind_until_ms = 0;

// ---------- Rate Tracking ----------
float prev_ldr1 = 0.0f, prev_ldr2 = 0.0f, prev_ldr3 = 0.0f, prev_ldr4 = 0.0f;

// ---------- Fallback Values ----------
const float FALLBACK_TEMP = 27.0f;
const float FALLBACK_HUM  = 52.0f;

// ---------- Loop State ----------
unsigned long loop_count = 0;
unsigned long boot_ms = 0;
unsigned long page_ms = 0;
int page = 0;

// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("#############################################################");
  Serial.println("#                   SOLTRACK BOOT                           #");
  Serial.println("#  Dual-Axis Solar Tracker + Wind Shutdown + Power Monitor  #");
  Serial.println("#  Display: ST7789 1.14\" TFT (135x240)                      #");
  Serial.println("#############################################################");

  // ---------- TFT ----------
  tft.init(135, 240);
  tft.setRotation(3);           // landscape 240x135
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("SOLTRACK");
  tft.setTextSize(1);
  tft.setCursor(0, 40);
  tft.println("Booting...");

  analogSetAttenuation(ADC_11db);

  // ---------- LEDs ----------
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);

  // ---------- DHT11 ----------
  dht.setup(DHTPIN, DHTesp::DHT11);
  delay(2000);
  Serial.println("Warming up DHT11...");
  for (int i = 0; i < 3; i++) {
    dht.getTempAndHumidity();
    delay(1200);
  }
  Serial.println("DHT11 ready");

  // ---------- LDRs ----------
  pinMode(PIN_LDR1, INPUT);
  pinMode(PIN_LDR2, INPUT);
  pinMode(PIN_LDR3, INPUT);
  pinMode(PIN_LDR4, INPUT);

  // ---------- ACS712 Calibration ----------
  Serial.println();
  Serial.println("Calibrating ACS712 zero-offset...");
  Serial.println(">> Cover the panel or disconnect panel (+) <<");
  for (int i = 3; i > 0; i--) {
    Serial.print("Starting in ");
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  float sum = 0.0f;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum += (analogRead(PIN_ACS712) / (float)ADC_MAX) * V_REF;
    delay(2);
  }
  zero_offset = sum / CAL_SAMPLES;

  Serial.print("Zero-offset calibrated: ");
  Serial.print(zero_offset, 4);
  Serial.println(" V");

  if (zero_offset < 0.5f || zero_offset > 1.2f) {
    Serial.println("!! WARNING: unexpected offset. Check ACS712 wiring.");
  }

  // ---------- Servos ----------
  servoAz.attach(PIN_SERVO_AZ, 1100, 1900);
  servoZn.attach(PIN_SERVO_ZN, 1100, 1900);
  servoAz.write(AZ_CENTER);
  servoZn.write(ZN_CENTER);
  delay(800);

  Serial.println("Servos centered at 90°");
  boot_ms = millis();
  page_ms = millis();

  // ---------- Serial Header ----------
  Serial.println();
  Serial.println("  #  |  T     H   | Wind dT dH | LDR TB   LR   | Model AZ  ZN  | Servo AZ  ZN  | Pwr V    mA    mW  | PANEL POSITION");
  Serial.println("-----+------------+------------+---------------+---------------+---------------+--------------------+----------------");

  tft.fillScreen(ST77XX_BLACK);
}

// =============================================================
void loop() {
  loop_count++;

  // ---------- 1. DHT11 ----------
  TempAndHumidity dhtData = dht.getTempAndHumidity();
  float temp = dhtData.temperature;
  float hum  = dhtData.humidity;
  bool dht_ok = !isnan(temp) && !isnan(hum);
  if (!dht_ok) { temp = FALLBACK_TEMP; hum = FALLBACK_HUM; }

  // ---------- 2. Wind Detection ----------
  temp_history[history_idx] = temp;
  hum_history[history_idx]  = hum;
  history_idx = (history_idx + 1) % WINDOW;
  if (history_count < WINDOW) history_count++;

  float t_min = temp, t_max = temp, h_min = hum, h_max = hum;
  for (int i = 0; i < history_count; i++) {
    if (temp_history[i] < t_min) t_min = temp_history[i];
    if (temp_history[i] > t_max) t_max = temp_history[i];
    if (hum_history[i]  < h_min) h_min = hum_history[i];
    if (hum_history[i]  > h_max) h_max = hum_history[i];
  }
  float dT = t_max - t_min;
  float dH = h_max - h_min;

  bool triggered = false;
  if (dht_ok && history_count >= WINDOW) {
    if (dH >= WIND_HUM_DELTA || dT >= WIND_TEMP_DELTA) triggered = true;
  }
  if (triggered) { wind_until_ms = millis() + HOLD_MS; wind_detected = true; }
  if (wind_detected && millis() > wind_until_ms) wind_detected = false;

  // ---------- 3. Read LDRs ----------
  float ldr1 = (analogRead(PIN_LDR1) / 4095.0f) * 3.3f;
  float ldr2 = (analogRead(PIN_LDR2) / 4095.0f) * 3.3f;
  float ldr3 = (analogRead(PIN_LDR3) / 4095.0f) * 3.3f;
  float ldr4 = (analogRead(PIN_LDR4) / 4095.0f) * 3.3f;

  float ldr_top_bottom = ldr1 - ldr3;
  float ldr_right_left = ldr2 - ldr4;
  float ldr_total      = ldr1 + ldr2 + ldr3 + ldr4;
  float ldr_balance    = fabsf(ldr_top_bottom) + fabsf(ldr_right_left);

  float ldr1_rate = ldr1 - prev_ldr1;
  float ldr2_rate = ldr2 - prev_ldr2;
  float ldr3_rate = ldr3 - prev_ldr3;
  float ldr4_rate = ldr4 - prev_ldr4;
  prev_ldr1 = ldr1; prev_ldr2 = ldr2; prev_ldr3 = ldr3; prev_ldr4 = ldr4;

  // ---------- 4. Model Inference ----------
  float features[14] = {
    ldr1, ldr2, ldr3, ldr4,
    temp, hum,
    ldr_top_bottom, ldr_right_left,
    ldr1_rate, ldr2_rate, ldr3_rate, ldr4_rate,
    ldr_total, ldr_balance
  };

  float model_azimuth = sol_track_azimuth_predict(features, 14);
  float model_zenith  = sol_track_zenith_predict(features, 14);

  // ---------- 5. Servo Angles ----------
  int azDeg = AZ_CENTER + (int)(ldr_right_left * AZ_SWING);
  int znDeg = ZN_CENTER + (int)(ldr_top_bottom * ZN_SWING);

  if (azDeg < AZ_MIN) azDeg = AZ_MIN;
  if (azDeg > AZ_MAX) azDeg = AZ_MAX;
  if (znDeg < ZN_MIN) znDeg = ZN_MIN;
  if (znDeg > ZN_MAX) znDeg = ZN_MAX;

  // ---------- 6. Panel Position State ----------
  const char* panel_position;
  const char* tft_state;
  bool force_stow = false;

  if (wind_detected) {
    panel_position = "STOW (WIND)";
    tft_state      = "STOW";
    force_stow     = true;
  } else if (!dht_ok) {
    panel_position = "STOW (DHT-FAIL)";
    tft_state      = "STOW";
    force_stow     = true;
  } else if (millis() - boot_ms < 2000) {
    panel_position = "BOOT";
    tft_state      = "BOOT";
    force_stow     = true;
  } else {
    panel_position = "TRACKING";
    tft_state      = "TRACKING";
  }

  if (force_stow) { azDeg = 90; znDeg = 90; }

  servoAz.write(azDeg);
  servoZn.write(znDeg);

  // ---------- 7. LEDs ----------
  if (wind_detected || !dht_ok) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }

  // ---------- 8. Panel Power ----------
  float v_acs = (analogRead(PIN_ACS712) / (float)ADC_MAX) * V_REF;
  float v_pin = (analogRead(PIN_PANEL_V) / (float)ADC_MAX) * V_REF;

  float panel_v   = v_pin * V_DIV_RATIO;
  float current_A = (v_acs - zero_offset) / ACS_SENS_EFF;
  if (current_A < 0.02f) current_A = 0.0f;
  float power_W   = panel_v * current_A;

  // ---------- 9. Serial Output ----------
  char buf[280];
  snprintf(buf, sizeof(buf),
           " %3lu | %4.1f %3.0f | %4.1f %4.1f | %+5.2f %+5.2f | %5.1f  %5.1f |  %3d°   %3d°  | %4.2fV %6.1f %6.0f | %s",
           loop_count, temp, hum, dT, dH,
           ldr_top_bottom, ldr_right_left,
           model_azimuth, model_zenith,
           azDeg, znDeg,
           panel_v, current_A * 1000.0, power_W * 1000.0,
           panel_position);
  Serial.println(buf);

  // ---------- 10. TFT Display — alternating pages ----------
  if (millis() - page_ms > 3000) {
    page = (page + 1) % 2;
    page_ms = millis();
  }

  if (page == 0) {
    // Page 1: state + angles
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(0, 0);
    tft.print("STATE:");

    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(0, 25);
    tft.print(tft_state);
    tft.print("   ");

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 65);
    tft.print("AZ:");
    tft.print(azDeg);
    tft.print("  ");

    tft.setCursor(0, 95);
    tft.print("ZE:");
    tft.print(znDeg);
    tft.print("  ");
  } else {
    // Page 2: panel power
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(0, 0);
    tft.print("PANEL PWR");

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 35);
    tft.print(panel_v, 1);
    tft.print(" V  ");

    tft.setCursor(0, 65);
    tft.print(current_A * 1000.0, 0);
    tft.print(" mA  ");

    tft.setCursor(0, 95);
    tft.print(power_W * 1000.0, 0);
    tft.print(" mW  ");
  }

  delay(1500);
}