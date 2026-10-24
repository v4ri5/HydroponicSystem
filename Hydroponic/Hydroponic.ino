#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>


// Add precision reaDINGS FOR TDS AND PH 

// --- Pins ---
const int TEMP_PIN      = 4;
const int WATER_LEVEL   = 6;
const int BACKLIGHT_PIN = 7;
const int PH_PIN        = A1;
const int TDS_PIN       = A2;
const int TDS_POWER_PIN = 5;// This has to have the VCC wire into the D pin in the 5 digital pin group

// --- Display tuning ---
const uint8_t CONTRAST = 30;    // tune 0-255 if the screen is blank or solid black

const float ADC_VREF = 5.0;
const float ADC_MAX  = 1023.0;

const bool  PH_CALIBRATED = false;
const float PH_V7 = 1.50;   // volts in pH 7 buffer
const float PH_V4 = 2.03;   // volts in pH 4 buffer

const unsigned long SAMPLE_INTERVAL = 40;
unsigned long lastSample = 0;

const int SAMPLE_COUNT = 25;
int phSamples[SAMPLE_COUNT];
int tdsSamples[SAMPLE_COUNT];
int tdsCount = 0; 
int phCount = 0;

bool dataReady = false;

const unsigned long READ_INTERVAL = 1000;  // ms between reads (must stay > 750)
unsigned long lastRead = 0;

OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
U8G2_ST7565_NHD_C12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs */ 10, /* dc */ 9, /* reset */ 8);


int medianOf(const int *src, int n) {
  int tmp[SAMPLE_COUNT];
  for (int i = 0; i < n; i++) tmp[i] = src[i];
  for (int i = 1; i < n; i++) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

float toVolts(int raw) {
  return raw * ADC_VREF / ADC_MAX;
}

float voltsToTds(float volts, float tempC) {
  float coeff = 1.0 + 0.02 * (tempC - 25.0);
  float v = volts / coeff;
  return (133.42 * v * v * v - 255.86 * v * v + 857.39 * v) * 0.5;
}

float voltsToPh(float volts) {
  float slope = (7.0 - 4.0) / (PH_V7 - PH_V4);
  return 7.0 + slope * (volts - PH_V7);
}

enum TDS_States {
  TDS_Stablize,
  TDS_Collection,
  pH_Recover,
  pH_Collection,
};

unsigned long stateStart = 0;
TDS_States current = TDS_Stablize;

void setup() {

  Serial.begin(9600);

  pinMode(WATER_LEVEL, INPUT_PULLUP);
  pinMode(BACKLIGHT_PIN, OUTPUT);
  pinMode(TDS_POWER_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);   // if the backlight stays off, try LOW
  digitalWrite(TDS_POWER_PIN, HIGH);
  sensors.begin();
  sensors.setWaitForConversion(false); // don't block ~750 ms on each request
  sensors.requestTemperatures();       // start the first conversion

  Serial.print("Devices found: ");
  Serial.println(sensors.getDeviceCount());

  u8g2.begin();
  u8g2.setContrast(CONTRAST);
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Splash screen, shown until the first reading
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "STARTING....");
  u8g2.sendBuffer();


  stateStart = millis();
}

void loop() {
    switch (current) {
    case TDS_Stablize:
      if (millis() - stateStart >= 1000){
        current = TDS_Collection;
        stateStart = millis();
        Serial.println("Stablize -> Collection");
      }
      break;

    case TDS_Collection:
      if (millis() - lastSample >= SAMPLE_INTERVAL) {
        lastSample = millis();
        tdsSamples[tdsCount] = analogRead(TDS_PIN);
        tdsCount++;
      }
      if (tdsCount >= SAMPLE_COUNT) {
        tdsCount = 0;
        digitalWrite(TDS_POWER_PIN, LOW);
        stateStart = millis();
        current = pH_Recover;
        Serial.println("Collection -> Recover");
      }
      break;

    case pH_Recover:
      if (millis() - stateStart >= 5000) {
        stateStart = millis();
        current = pH_Collection;
        Serial.println("Recover -> Collection");
      }
      break;

    case pH_Collection:
      if (millis() - lastSample >= SAMPLE_INTERVAL) {
        lastSample = millis();
        phSamples[phCount] = analogRead(PH_PIN);
        phCount++;
      }
      if (phCount >= SAMPLE_COUNT) {
        phCount = 0;
        dataReady = true;
        digitalWrite(TDS_POWER_PIN, HIGH);
        stateStart = millis();
        current = TDS_Stablize;
        Serial.println("Collection -> Stablize");
      }
      break;
  }

  if (millis() - lastRead >= READ_INTERVAL && dataReady > 0) {
    lastRead = millis();

    // Read the conversion started on the previous tick, then start the next one
    float tempC = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    bool probeOk = (tempC != DEVICE_DISCONNECTED_C);
    bool level   = digitalRead(WATER_LEVEL);
    float phVolts  = toVolts(medianOf(phSamples, SAMPLE_COUNT));
    float tdsVolts = toVolts(medianOf(tdsSamples, SAMPLE_COUNT));
    float compTemp = probeOk ? tempC : 25.0;
    float tds = voltsToTds(tdsVolts, compTemp);
    float ph  = voltsToPh(phVolts);
    
    // --- Serial output ---
    Serial.println(level ? "WATER LEVEL HIGH" : "WATER LEVEL LOW");

    if (probeOk) {
      Serial.print("Temp: ");
      Serial.print(tempC);
      Serial.println(" C");
    } else {
      Serial.println("Probe not found - check wiring and 4.7k resistor");
    }

    // --- LCD output ---
    u8g2.clearBuffer();

    u8g2.drawStr(0, 10, "Temp:");
    u8g2.setCursor(50, 10);
    if (probeOk) {
      u8g2.print(tempC, 1);
      u8g2.print(" C");
    } else {
      u8g2.print("--");
    }

    u8g2.drawStr(0, 25, "Water:");
    u8g2.setCursor(50, 25);
    u8g2.print(level ? "HIGH" : "LOW");
    u8g2.drawStr(0, 40, "TDS:");
    u8g2.setCursor(50, 40);
    u8g2.print(tds, 0);
    u8g2.print(" ppm");

    u8g2.drawStr(0, 55, "pH:");
    u8g2.setCursor(50, 55);
    if (PH_CALIBRATED) {
      u8g2.print(ph, 2);
    } else {
      u8g2.print(phVolts, 2);
      u8g2.print(" V");
    }

    u8g2.sendBuffer();
  }
}
