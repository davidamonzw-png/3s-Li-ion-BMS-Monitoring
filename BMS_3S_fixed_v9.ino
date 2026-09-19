
/*
Created on Sat 11 April 19:55:00 2026

@author: David Amon
*/

// 3S Li-Ion BMS MONITORING SYSTEM (MEDIAN + WINDOW + SLEW LIMIT)
// ESP32 + Blynk + 16x2 I2C LCD


#define BLYNK_TEMPLATE_ID "YOUR TEMP ID"
#define BLYNK_TEMPLATE_NAME "DAVID BMS SYSTEM"
#define BLYNK_AUTH_TOKEN "YOUR TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ==========================================================
// WIFI
// ==========================================================
char ssid[] = "YOUR WIFI NAME";
char pass[] = "YOUR WIFI PASSWORD";

// ==========================================================
// PIN CONFIGURATION
// ==========================================================
const int redLED    = 17;
const int whiteLED  = 16;
const int greenLED  = 18;

const int tempPin    = 34; 
const int voltagePin = 35;

// ==========================================================
// LCD
// ==========================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==========================================================
// VOLTAGE DIVIDER CALCULATIONS (R1+R2+R3+R4, tap after R3 -> /4)
// ==========================================================
const float DIVIDER_FACTOR = 4.0;
const float CALIBRATION_FACTOR = 1.22; 

// ==========================================================
// BATTERY LIMITS
// ==========================================================
const float BATTERY_MIN = 9.0;
const float BATTERY_MAX = 12.6;

// ==========================================================
// FILTER / STABILITY SETTINGS
// ==========================================================
const int MEDIAN_SAMPLES    = 21;    // must be odd — kills spike noise per reading
const int SAMPLE_SPACING_US = 1000;  // spreads the burst across ~21ms

const int WINDOW_SECONDS = 20; // rolling average length in seconds

// *** Caps how fast the displayed battery % is allowed to change ***
// This is independent of the averaging window above — even if the raw
// voltage genuinely sags under load, the readout won't jump faster than this.
const float MAX_PCT_SLEW_PER_MIN = 4.0; // % per minute

const int PCT_DEADBAND = 0; // slew limiter already smooths this; keep at 0

// ==========================================================
// DIAGNOSTIC ALERTS (Option 1: fixed thresholds + rate-of-change)
// ==========================================================
const float TEMP_ALERT_MAX = 45.0f;  // C -- above this = overheating
const float VOLT_ALERT_MIN = 9.3f;   // V -- just above BATTERY_MIN, early low-voltage warning
const float VOLT_ALERT_MAX = 12.9f;  // V -- above nominal full charge = overvoltage

const float MAX_VOLT_DROP_PER_MIN = 0.5f; // V/min -- faster than this = abnormal load/fault
const float MAX_TEMP_RISE_PER_MIN = 5.0f; // C/min -- faster than this = thermal runaway risk

float  prevWindowVoltage = 0.0f;
float  prevWindowTemp    = 0.0f;
bool   diagPrevInit      = false;
String activeAlert       = "";

// ==========================================================
// ROLLING WINDOW BUFFERS
// ==========================================================
float voltBuffer[WINDOW_SECONDS];
float tempBuffer[WINDOW_SECONDS];
int   bufferIndex = 0;
float voltSum = 0;
float tempSum = 0;

float windowVoltage = 12.0; // rolling-average voltage
float windowTemp    = 25.0;

float displayedPctF = 100.0; // continuous internal %, slew-limited
int   displayedPct   = 100;  // rounded, what's actually shown/sent
String batteryStatus = "Unknown";

unsigned long previousMillis = 0;
const unsigned long measurementInterval = 1000;

// ==========================================================
// HELPER: SORT FOR MEDIAN FILTER
// ==========================================================
void sortArray(int a[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (a[i] > a[j]) {
        int t = a[i];
        a[i] = a[j];
        a[j] = t;
      }
    }
  }
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(redLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  digitalWrite(redLED, LOW);
  digitalWrite(whiteLED, LOW);
  digitalWrite(greenLED, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(voltagePin, ADC_11db);
  analogSetPinAttenuation(tempPin, ADC_11db);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BMS Initializing");

  // Clean WiFi state before connecting -- on ESP32, a stale/previous
  // connection state can make the handshake take much longer or fail
  // intermittently. Forcing STA mode + a fresh disconnect first makes
  // the connect time far more consistent.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connecting");

  WiFi.begin(ssid, pass);
  unsigned long wifiStart = millis();
  int dotCol = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(dotCol % 16, 1);
    lcd.print(".");
    dotCol++;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("WiFi: Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
  } else {
    lcd.print("WiFi: FAILED");
    lcd.setCursor(0, 1);
    lcd.print("Running offline");
  }
  delay(1800); // hold the status on screen briefly before normal readout

  // Always configure Blynk, even if WiFi hasn't connected yet -- this lets
  // Blynk.run() in loop() pick up the connection later instead of being
  // permanently skipped for the whole session.
  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.connect(1500);
    Serial.println("\nWiFi connected, Blynk connecting...");
  } else {
    Serial.println("\nWiFi not connected at boot -- will retry in loop().");
  }

  // Pre-fill the window with a real first reading
  float firstV = readMedianVoltage();
  float firstT = readMedianTemperature();
  for (int i = 0; i < WINDOW_SECONDS; i++) {
    voltBuffer[i] = firstV;
    tempBuffer[i] = firstT;
  }
  voltSum = firstV * WINDOW_SECONDS;
  tempSum = firstT * WINDOW_SECONDS;

  windowVoltage = firstV;
  windowTemp    = firstT;
  displayedPctF = voltageToPercentF(windowVoltage);
  displayedPct  = constrain((int)round(displayedPctF), 0, 100);
  updateBatteryStatus(displayedPct);

  lcd.clear();
}

// ==========================================================
// MEDIAN-FILTERED RAW VOLTAGE (calibrated millivolt reads)
// ==========================================================
float voltEMA = 0.0f;
bool  voltEMAInit = false;
const float VOLT_EMA_ALPHA = 0.15f; // same smoothing strength as temp

float readMedianVoltage() {
  // Throwaway read: right after switching ADC pins, the ESP32's first
  // conversion can be off -- discard one sample before the real burst.
  analogReadMilliVolts(voltagePin);
  delayMicroseconds(SAMPLE_SPACING_US);

  int samples[MEDIAN_SAMPLES];
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    samples[i] = analogReadMilliVolts(voltagePin);
    delayMicroseconds(SAMPLE_SPACING_US);
  }
  sortArray(samples, MEDIAN_SAMPLES);
  int medianMV = samples[MEDIAN_SAMPLES / 2];
  Serial.printf("[diag] volt pin raw: %dmV  ", medianMV);

  float pinVoltage = medianMV / 1000.0f;
  float instantVoltage = pinVoltage * DIVIDER_FACTOR * CALIBRATION_FACTOR;

  // Sanity check: a 3S pack should never realistically read outside this
  // range even at the extremes of over/under-voltage. If it does, it's a
  // glitch/loose contact, not a real reading -- hold the last trusted value.
  const float VOLT_SANITY_MIN = 5.0f;
  const float VOLT_SANITY_MAX = 15.0f;
  if (instantVoltage < VOLT_SANITY_MIN || instantVoltage > VOLT_SANITY_MAX) {
    Serial.printf("[WARN] voltage reading %.2fV out of sane range -- ignoring, holding last value\n", instantVoltage);
    if (voltEMAInit) {
      return voltEMA;
    } else {
      instantVoltage = 11.1f; // no prior value yet -- assume nominal 3S resting voltage
    }
  }

  // Same EMA smoothing as temperature: kills the small flicker between
  // consecutive 1-second readings before it enters the 20s rolling window.
  if (!voltEMAInit) {
    voltEMA = instantVoltage;
    voltEMAInit = true;
  } else {
    voltEMA += VOLT_EMA_ALPHA * (instantVoltage - voltEMA);
  }
  return voltEMA;
}

// ==========================================================
// MEDIAN-FILTERED RAW TEMPERATURE
// Wiring: 3.3V -> R5(1k) -> node(GPIO34) -> NTC(10D-9, R0=10kohm) -> GND
// ==========================================================
float tempEMA = 0.0f;
bool  tempEMAInit = false;
const float TEMP_EMA_ALPHA = 0.15f; // lower = smoother but slower to react

float readMedianTemperature() {
  // Throwaway read: right after changing attenuation/pins, the ESP32 ADC's
  // first conversion can be off -- discard one sample before the real burst.
  analogReadMilliVolts(tempPin);
  delayMicroseconds(SAMPLE_SPACING_US);

  int samples[MEDIAN_SAMPLES];
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    samples[i] = analogReadMilliVolts(tempPin);
    delayMicroseconds(SAMPLE_SPACING_US);
  }
  sortArray(samples, MEDIAN_SAMPLES);
  int medianMV = samples[MEDIAN_SAMPLES / 2];

  float vOut = medianMV / 1000.0f;
  Serial.printf("  [diag] temp pin raw: %.3fV  ", vOut);

  if (vOut < 0.05f) vOut = 0.05f;
  if (vOut > 3.25f) vOut = 3.25f;

  const float R_SERIES = 1000.0f; // R5
  const float R0       = 10000.0f; // "10D-9" = 10kohm nominal @25C (the "10" prefix
                                    // in NTC part codes is in kOhm, "-9" is bead size in mm)
  const float BETA     = 3950.0f; // datasheet value if you have one is better

  float resistanceNTC = R_SERIES * (vOut / (3.3f - vOut));

  float steinhart = resistanceNTC / R0;
  steinhart = log(steinhart);
  steinhart /= BETA;
  steinhart += 1.0f / (25.0f + 273.15f);
  steinhart = 1.0f / steinhart;
  float instantTemp = steinhart - 273.15f;

  // Sanity check: a 3S Li-ion pack operating indoors should never
  // realistically read outside this range. If it does, something is
  // wrong (bad contact, ADC glitch, disconnected sensor) -- don't let
  // a nonsense value like the old "8C" bug silently reach the display
  // or Blynk. Instead, hold the last good EMA value and flag it.
  const float TEMP_SANITY_MIN = 0.0f;
  const float TEMP_SANITY_MAX = 80.0f;
  if (instantTemp < TEMP_SANITY_MIN || instantTemp > TEMP_SANITY_MAX) {
    Serial.printf("[WARN] temp reading %.1fC out of sane range -- ignoring, holding last value\n", instantTemp);
    if (tempEMAInit) {
      return tempEMA; // keep showing the last trusted value instead of garbage
    } else {
      instantTemp = 25.0f; // no prior value yet (very first boot) -- assume room temp
    }
  }

  // Extra smoothing layer: the median filter kills single-sample spikes,
  // but ADC noise can still make consecutive 1-second readings drift up
  // and down by a degree or two. EMA smooths that out before it even
  // enters the 20s rolling window, so windowTemp is much steadier.
  if (!tempEMAInit) {
    tempEMA = instantTemp;
    tempEMAInit = true;
  } else {
    tempEMA += TEMP_EMA_ALPHA * (instantTemp - tempEMA);
  }
  return tempEMA;
}

// ==========================================================
// VOLTAGE -> PERCENT (float version, for the slew limiter)
// ==========================================================
float voltageToPercentF(float v) {
  float rawPct = ((v - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100.0f;
  return constrain(rawPct, 0.0f, 100.0f);
}

// ==========================================================
// DIAGNOSTICS: fixed thresholds + rate-of-change
// Runs once per measurement cycle, after windowVoltage/windowTemp
// are updated. Sets activeAlert to "" if everything looks normal.
// ==========================================================
void runDiagnostics() {
  activeAlert = "";

  // --- Fixed threshold checks ---
  if (windowTemp > TEMP_ALERT_MAX) {
    activeAlert = "OVERTEMP";
  } else if (windowVoltage < VOLT_ALERT_MIN) {
    activeAlert = "UNDERVOLT";
  } else if (windowVoltage > VOLT_ALERT_MAX) {
    activeAlert = "OVERVOLT";
  }

  // --- Rate-of-change checks (need one prior reading to compare against) ---
  if (diagPrevInit) {
    float minutesElapsed  = measurementInterval / 60000.0f;
    float voltDropPerMin  = (prevWindowVoltage - windowVoltage) / minutesElapsed;
    float tempRisePerMin  = (windowTemp - prevWindowTemp) / minutesElapsed;

    if (voltDropPerMin > MAX_VOLT_DROP_PER_MIN) {
      activeAlert = "RAPID_VDROP";
    }
    if (tempRisePerMin > MAX_TEMP_RISE_PER_MIN) {
      activeAlert = "RAPID_TRISE";
    }
  }

  prevWindowVoltage = windowVoltage;
  prevWindowTemp    = windowTemp;
  diagPrevInit      = true;

  if (activeAlert != "") {
    Serial.printf("[ALERT] %s  (V=%.2f T=%.1f)\n", activeAlert.c_str(), windowVoltage, windowTemp);
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (activeAlert != "") {
      // Requires an Event named "bms_alert" created in the Blynk console
      // (Console -> your template -> Events) for this to trigger a push
      // notification. Without that setup, this line is harmless -- it just
      // won't produce a notification, V6 will still update either way.
      Blynk.logEvent("bms_alert", activeAlert);
      Blynk.virtualWrite(V6, activeAlert);
    } else {
      Blynk.virtualWrite(V6, "OK");
    }
  }
}

// ==========================================================
// REFRESH STATES & LED OUTPUT
// ==========================================================
void updateBatteryStatus(int percentage) {
  if (percentage <= 25) {
    batteryStatus = "LOW";
    digitalWrite(redLED, HIGH);
    digitalWrite(whiteLED, LOW);
    digitalWrite(greenLED, LOW);
  } else if (percentage <= 75) {
    batteryStatus = "MID";
    digitalWrite(redLED, LOW);
    digitalWrite(whiteLED, HIGH);
    digitalWrite(greenLED, LOW);
  } else {
    batteryStatus = "FULL";
    digitalWrite(redLED, LOW);
    digitalWrite(whiteLED, LOW);
    digitalWrite(greenLED, HIGH);
  }
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(windowVoltage, 2);
  lcd.print("V ");
  lcd.print(displayedPct);
  lcd.print("%        ");

  lcd.setCursor(0, 1);
  if (activeAlert != "") {
    lcd.print("ALERT:");
    lcd.print(activeAlert);
    lcd.print("            "); // pad to clear leftover chars from longer text
  } else {
    lcd.print("S:");
    lcd.print(batteryStatus);
    lcd.print(" T:");
    lcd.print(windowTemp, 1);
    lcd.print("C        ");
  }
}

void updateBlynk() {
  Blynk.virtualWrite(V0, windowVoltage);   // VOLTAGE
  Blynk.virtualWrite(V1, windowTemp);      // temperature
  Blynk.virtualWrite(V4, displayedPct);    // state of discharge (percentage)
}

// ==========================================================
// MAIN LOOP
// ==========================================================
unsigned long lastWifiRetry = 0;
const unsigned long WIFI_RETRY_INTERVAL = 10000; // try every 10s if disconnected

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiRetry > WIFI_RETRY_INTERVAL) {
      lastWifiRetry = now;
      Serial.println("WiFi disconnected -- retrying...");
      WiFi.begin(ssid, pass);
    }
  } else {
    Blynk.run();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= measurementInterval) {
    previousMillis = currentMillis;

    // Step 1: median-filtered instantaneous reading
    float rawVoltage = readMedianVoltage();
    float rawTemp     = readMedianTemperature();
    Serial.printf("raw V:%.3f raw T:%.1f\n", rawVoltage, rawTemp);

    // Step 2: rolling window average
    voltSum -= voltBuffer[bufferIndex];
    tempSum -= tempBuffer[bufferIndex];
    voltBuffer[bufferIndex] = rawVoltage;
    tempBuffer[bufferIndex] = rawTemp;
    voltSum += rawVoltage;
    tempSum += rawTemp;
    bufferIndex = (bufferIndex + 1) % WINDOW_SECONDS;

    windowVoltage = voltSum / WINDOW_SECONDS;
    windowTemp    = tempSum / WINDOW_SECONDS;

    // Step 3: slew-rate limit the percentage — caps how fast it's
    // allowed to move regardless of how fast the raw voltage moves
    float targetPctF = voltageToPercentF(windowVoltage);
    float maxStep = MAX_PCT_SLEW_PER_MIN * (measurementInterval / 60000.0f);
    float delta = constrain(targetPctF - displayedPctF, -maxStep, maxStep);
    displayedPctF += delta;

    int newPct = constrain((int)round(displayedPctF), 0, 100);
    if (abs(newPct - displayedPct) > PCT_DEADBAND) {
      displayedPct = newPct;
    }

    updateBatteryStatus(displayedPct);
    runDiagnostics();
    updateLCD();

    if (WiFi.status() == WL_CONNECTED) {
      updateBlynk();
    }

    Serial.printf("AVG(%ds) -> Volt: %.2fV | Temp: %.1fC | Bat: %d%% (target %.1f%%) | Status: %s\n",
                  WINDOW_SECONDS, windowVoltage, windowTemp, displayedPct, targetPctF, batteryStatus.c_str());
  }
}
