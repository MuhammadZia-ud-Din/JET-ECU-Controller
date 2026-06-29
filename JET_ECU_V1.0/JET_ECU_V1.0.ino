/*
  JET ECU V1.0
  Platform : STM32 Blue Pill (STM32F103C8T6)
*/
#include <HardwareSerial.h>
#include <Adafruit_MAX31855.h>

HardwareSerial Serial2(PA3, PA2);

// ─── Thermocouple 1 (Exhaust / EGT) ───────────────────────────────────────
#define TC1_SCK PA5
#define TC1_SO  PA6
#define TC1_CS  PB0

// ─── Thermocouple 2 (Intake / Ambient) ─────────────────────────────────────
#define TC2_SCK PB13
#define TC2_SO  PB14
#define TC2_CS  PB12

// ─── Battery Voltage ────────────────────────────────────────────────────────
#define BATT_PIN    PA1
#define BATT_R1     39000.0f
#define BATT_R2     5600.0f
#define ADC_REF     3.3f
#define ADC_RES     4095.0f
#define NUM_SAMPLES 64
#define CAL_FACTOR  0.9905f

// ─── RC PWM Input (R12DS CH3) ────────────────────────────────────────────────
#define RC_PWM_PIN  PA0
#define RPM_SENSOR_PIN PC14
#define RPM_MAX        50000L
#define RC_MAX_STEP 400

// ─── RPM Sensor ─────────────────────────────────────────────────────────────
#define RPM_PIN PB11

// ─── FET Output Pins ────────────────────────────────────────────────────────
#define FET_1 PB6
#define FET_2 PB5
#define FET_3 PB9
#define FET_4 PB8
#define FET_5 PB4
#define FET_6 PB7

#define FET_1_HIGH digitalWrite(FET_1, HIGH)
#define FET_1_LOW  digitalWrite(FET_1, LOW)
#define FET_2_HIGH digitalWrite(FET_2, HIGH)
#define FET_2_LOW  digitalWrite(FET_2, LOW)
#define FET_3_HIGH digitalWrite(FET_3, HIGH)
#define FET_3_LOW  digitalWrite(FET_3, LOW)
#define FET_4_HIGH digitalWrite(FET_4, HIGH)
#define FET_4_LOW  digitalWrite(FET_4, LOW)
#define FET_5_HIGH digitalWrite(FET_5, HIGH)
#define FET_5_LOW  digitalWrite(FET_5, LOW)
#define FET_6_HIGH digitalWrite(FET_6, HIGH)
#define FET_6_LOW  digitalWrite(FET_6, LOW)

const uint8_t FET_PINS[6] = {FET_1, FET_2, FET_3, FET_4, FET_5, FET_6};

Adafruit_MAX31855 tc1(TC1_SCK, TC1_CS, TC1_SO);
Adafruit_MAX31855 tc2(TC2_SCK, TC2_CS, TC2_SO);

float         temp1       = 0;
float         temp2       = 0;
float         battVoltage = 0;

unsigned long lastTempRead    = 0;
unsigned long lastVoltRead    = 0;
unsigned long lastFETReport   = 0;
unsigned long lastRPMRead     = 0;
unsigned long lastMCUTempRead = 0;
String        serialBuf       = "";

// RC PWM calibration
static unsigned long pwMin       = 0;
static unsigned long pwMax       = 0;
static unsigned long lastPW      = 1000;
static unsigned long lastPWValid = 0;

unsigned long samplePW() {
    unsigned long sum = 0;
    int cnt = 0;
    for (int i = 0; i < 20; i++) {
        unsigned long p = pulseIn(RC_PWM_PIN, HIGH, 25000);
        if (p >= 900 && p <= 2100) { sum += p; cnt++; }
        delay(50);
    }
    return cnt > 0 ? sum / cnt : 0;
}

void handleSerial() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            serialBuf.trim();
            if (serialBuf.startsWith("FET") && serialBuf.length() >= 6 && serialBuf[4] == ':') {
                int n   = serialBuf[3] - '1';
                int val = serialBuf.substring(5).toInt();
                if (n >= 0 && n <= 5)
                    digitalWrite(FET_PINS[n], val ? HIGH : LOW);
            } else if (serialBuf == "CAL_CENTER") {
                pwMin = samplePW();
                pwMax = 0;
                Serial2.print("CAL_CENTER:"); Serial2.println(pwMin);
            } else if (serialBuf == "CAL_FULL") {
                pwMax = samplePW();
                Serial2.print("CAL_FULL:"); Serial2.println(pwMax);
            } else if (serialBuf == "CAL_RESET") {
                pwMin = 0; pwMax = 0; lastPW = 1000;
                Serial2.println("CAL_RESET:OK");
            }
            serialBuf = "";
        } else if (c != '\r') {
            serialBuf += c;
        }
    }
}

void reportFETStates() {
    Serial2.print("FET:");
    for (int i = 0; i < 6; i++) {
        Serial2.print(digitalRead(FET_PINS[i]));
        if (i < 5) Serial2.print(",");
    }
    Serial2.println();
}

uint16_t readADC_avg(uint8_t pin) {
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(pin);
        delayMicroseconds(200);
    }
    return sum / NUM_SAMPLES;
}

void readMCUTemp() {
    uint16_t raw  = analogRead(ATEMP);
    float    v    = (raw / 4095.0f) * 3.3f;
    float    temp = ((1.58f - v) / 0.00430f) + 25.0f;
    Serial2.print("MCU: ");
    Serial2.print(temp, 1);
    Serial2.println(" C");
}

void readThrottle() {
    unsigned long pw = pulseIn(RC_PWM_PIN, HIGH, 25000);
    if (pw >= 900 && pw <= 2100) {
        unsigned long diff = (pw > lastPW) ? (pw - lastPW) : (lastPW - pw);
        if (diff <= RC_MAX_STEP) {
            lastPW      = pw;
            lastPWValid = millis();
        }
    }
    bool noSignal = (millis() - lastPWValid > 500);
    long thr = 0;
    if (!noSignal && pwMax > pwMin) {
        thr = map(constrain(lastPW, pwMin, pwMax), pwMin, pwMax, 0L, 100L);
        thr = constrain(thr, 0L, 100L);
    }
    Serial2.print("THR: ");
    Serial2.println(thr);
}

void setup() {
    Serial2.begin(115200);
    analogReadResolution(12);

    pinMode(FET_1, OUTPUT); digitalWrite(FET_1, LOW);
    pinMode(FET_2, OUTPUT); digitalWrite(FET_2, LOW);
    pinMode(FET_3, OUTPUT); digitalWrite(FET_3, LOW);
    pinMode(FET_4, OUTPUT); digitalWrite(FET_4, LOW);
    pinMode(FET_5, OUTPUT); digitalWrite(FET_5, LOW);
    pinMode(FET_6, OUTPUT); digitalWrite(FET_6, LOW);

    pinMode(BATT_PIN,   INPUT_ANALOG);
    pinMode(RPM_PIN,    INPUT);
    pinMode(RC_PWM_PIN, INPUT);

    if (!tc1.begin()) Serial2.println("TC1 error - check wiring");
    if (!tc2.begin()) Serial2.println("TC2 error - check wiring");

    Serial2.println("JET ECU V1.0 Ready");
}

void readTemperatures() {
    temp1 = tc1.readCelsius();
    temp2 = tc2.readCelsius();
    if (isnan(temp1)) temp1 = 0;
    if (isnan(temp2)) temp2 = 0;
    Serial2.print("TC1: "); Serial2.print(temp1, 1);
    Serial2.print(" C  |  ");
    Serial2.print("TC2: "); Serial2.print(temp2, 1);
    Serial2.println(" C");
}

void readBattVoltage() {
    uint16_t raw   = readADC_avg(BATT_PIN);
    float    v_adc = (raw / ADC_RES) * ADC_REF;
    battVoltage    = v_adc * ((BATT_R1 + BATT_R2) / BATT_R2) * CAL_FACTOR;
    Serial2.print("Raw: ");     Serial2.print(raw);
    Serial2.print("\tV_ADC: "); Serial2.print(v_adc, 3); Serial2.print(" V");
    Serial2.print("\tV_BATT: ");Serial2.print(battVoltage, 2); Serial2.println(" V");
}

void loop() {
    handleSerial();

    if (millis() - lastTempRead >= 250) {
        lastTempRead = millis();
        readTemperatures();
    }
    if (millis() - lastVoltRead >= 500) {
        lastVoltRead = millis();
        readBattVoltage();
    }
    if (millis() - lastFETReport >= 200) {
        lastFETReport = millis();
        reportFETStates();
    }
    if (millis() - lastRPMRead >= 200) {
        lastRPMRead = millis();
        readThrottle();
    }
    if (millis() - lastMCUTempRead >= 1000) {
        lastMCUTempRead = millis();
        readMCUTemp();
    }
}