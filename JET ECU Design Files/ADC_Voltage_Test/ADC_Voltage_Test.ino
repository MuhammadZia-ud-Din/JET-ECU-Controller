#include <HardwareSerial.h>

HardwareSerial Serial2(PA3, PA2);

#define BATT_PIN    PA1
#define BATT_R1     39000.0f
#define BATT_R2      5600.0f
#define ADC_REF       3.3f
#define ADC_RES      4095.0f
#define NUM_SAMPLES    64
#define CAL_FACTOR    0.9905f   // corrects ~1% gain error from VDDA/resistor tolerance

uint16_t readADC_avg(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return sum / NUM_SAMPLES;
}

void setup() {
  Serial2.begin(115200);
  analogReadResolution(12);
  pinMode(BATT_PIN, INPUT_ANALOG);
}

void loop() {
  uint16_t raw   = readADC_avg(BATT_PIN);
  float    v_adc = (raw / ADC_RES) * ADC_REF;
  float    v_in  = v_adc * ((BATT_R1 + BATT_R2) / BATT_R2) * CAL_FACTOR;

  Serial2.print("Raw: ");
  Serial2.print(raw);
  Serial2.print("\tV_ADC: ");
  Serial2.print(v_adc, 3);
  Serial2.print(" V\tV_BATT: ");
  Serial2.print(v_in, 2);
  Serial2.println(" V");

  delay(500);
}