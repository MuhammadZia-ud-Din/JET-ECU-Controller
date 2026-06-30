#include <HardwareSerial.h>

HardwareSerial Serial2(PA3, PA2);

#define PWM_PIN      PA0
#define RPM_MAX      120000L
#define REPORT_MS    200
#define SIG_TIMEOUT  500
#define MAX_STEP     400

static unsigned long lastReport  = 0;
static unsigned long lastValidMs = 0;
static unsigned long lastPW      = 1000;
static unsigned long pwMin       = 1000;
static unsigned long pwMax       = 2000;

unsigned long sampleAvg() {
    unsigned long sum = 0;
    int count = 0;
    for (int i = 0; i < 20; i++) {
        unsigned long pw = pulseIn(PWM_PIN, HIGH, 25000);
        if (pw >= 900 && pw <= 2100) { sum += pw; count++; }
        delay(50);
    }
    return (count > 0) ? (sum / count) : 0;
}

void countdown(int sec) {
    for (int i = sec; i > 0; i--) {
        Serial2.print(i); Serial2.println("...");
        delay(1000);
    }
}

void calibrate() {
    Serial2.println("=== CALIBRATION ===");
    Serial2.println("Step 1: Hold throttle at CENTER (0 RPM position)");
    countdown(5);
    pwMin = sampleAvg();
    Serial2.print("Center captured: "); Serial2.println(pwMin);

    Serial2.println("Step 2: Push throttle to FULL");
    countdown(5);
    pwMax = sampleAvg();
    Serial2.print("Full captured:   "); Serial2.println(pwMax);

    if (pwMax <= pwMin) {
        Serial2.println("ERROR: full must be > center. Using defaults.");
        pwMin = 1000; pwMax = 2000;
    }
    lastPW = pwMin;
    Serial2.println("=== DONE — Running ===");
}

void setup() {
    pinMode(PWM_PIN, INPUT);
    Serial2.begin(115200);
    calibrate();
}

void loop() {
    unsigned long pw = pulseIn(PWM_PIN, HIGH, 25000);
    if (pw >= 900 && pw <= 2100) {
        unsigned long diff = (pw > lastPW) ? (pw - lastPW) : (lastPW - pw);
        if (diff <= MAX_STEP) { lastPW = pw; lastValidMs = millis(); }
    }

    if (millis() - lastReport >= REPORT_MS) {
        lastReport = millis();
        bool noSignal = (millis() - lastValidMs > SIG_TIMEOUT);
        long rpm = 0;
        if (!noSignal && pwMax > pwMin) {
            rpm = map(constrain(lastPW, pwMin, pwMax), pwMin, pwMax, 0, RPM_MAX);
        }
        Serial2.print("RPM: "); Serial2.println(rpm);
    }
}