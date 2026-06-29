# JET ECU V1.0 — Complete Technical Specification

> **Purpose of this document:** Full system specification for the JET Engine ECU project.
> Any developer reading this document should be able to understand, replicate,
> extend, or modify any part of the firmware or GUI from scratch without additional context.

---

## 1. Project Overview

A custom Engine Control Unit (ECU) for monitoring and controlling a small JET engine during
ground testing. The system consists of two parts:

| Part | Description |
|------|-------------|
| **Firmware** | Runs on the STM32 Blue Pill. Reads sensors, controls outputs, streams telemetry over UART |
| **GUI Dashboard** | Python desktop application. Displays live telemetry, allows FET control over the same serial link |

### Design Philosophy
- Firmware is **non-blocking** — all timing done with `millis()` comparisons, no `delay()`
- Serial link is **bidirectional** — firmware streams data out, GUI sends commands in
- All ADC reads use **multi-sample averaging** to reduce noise
- Safe state on power-up: all FET outputs LOW

---

## 2. Hardware

### 2.1 Microcontroller

| Property | Value |
|----------|-------|
| MCU | STM32F103C8T6 |
| Board | Blue Pill |
| Flash | 64 KB |
| RAM | 20 KB |
| Clock | 72 MHz |
| ADC resolution | 12-bit (0–4095) |
| Operating voltage | 3.3V I/O |

### 2.2 External Components

| Component | Part | Purpose |
|-----------|------|---------|
| Thermocouple amplifiers | MAX31855 × 2 | K-type thermocouple interface via SPI |
| Voltage divider | 39KΩ + 5.6KΩ | Scale battery voltage (up to 24V) to ADC range |
| Signal conditioner | Dual LM358 op-amp | Condition RPM pulses for digital input on PB11 |
| Op-amp buffer | Single LM358 | Buffer potentiometer output for RPM simulation on PA0 |
| TTL Serial adapter | USB-UART | Connect UART2 to PC for telemetry and commands |

---

## 3. Complete Pin Map

| Function | Pin | Direction | Mode | Notes |
|----------|-----|-----------|------|-------|
| UART2 TX | PA2 | Output | AF_PP | Serial transmit to PC |
| UART2 RX | PA3 | Input | Input | Serial receive from PC |
| Battery Voltage | PA1 | Input | INPUT_ANALOG | Voltage divider output |
| RPM Test (pot) | PA0 | Input | INPUT_ANALOG | Op-amp buffered pot — testing only |
| TC1 SCK | PA5 | Output | Software SPI | MAX31855 #1 clock — Exhaust/EGT |
| TC1 SO | PA6 | Input | Software SPI | MAX31855 #1 data out |
| TC1 CS | PB0 | Output | Software SPI | MAX31855 #1 chip select |
| TC2 SCK | PB13 | Output | Software SPI | MAX31855 #2 clock — Intake/Ambient |
| TC2 SO | PB14 | Input | Software SPI | MAX31855 #2 data out |
| TC2 CS | PB12 | Output | Software SPI | MAX31855 #2 chip select |
| RPM Sensor | PB11 | Input | INPUT | LM358-conditioned pulse train |
| FET 1 | PB6 | Output | OUTPUT | Engine actuator / valve |
| FET 2 | PB5 | Output | OUTPUT | Engine actuator / valve |
| FET 3 | PB9 | Output | OUTPUT | Engine actuator / valve |
| FET 4 | PB8 | Output | OUTPUT | Engine actuator / valve |
| FET 5 | PB4 | Output | OUTPUT | Engine actuator / valve |
| FET 6 | PB7 | Output | OUTPUT | Engine actuator / valve |
| MCU Temp | Internal | Input | ADC ch.16 | Die temperature sensor (no external pin) |

---

## 4. Firmware Architecture

### 4.1 Build Environment

- Language: C++ (Arduino framework)
- Core: STMicroelectronics STM32 core v2.12.0
- FQBN: `STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8`
- Libraries:
  - `Adafruit_MAX31855` v1.4.2 — thermocouple reading
  - `Adafruit BusIO` v1.17.4 — dependency of MAX31855
  - `HardwareSerial` — built-in, UART2 declaration

### 4.2 Key Global Defines

```cpp
#define BATT_PIN    PA1
#define BATT_R1     39000.0f    // Upper resistor (ohms)
#define BATT_R2      5600.0f    // Lower resistor (ohms)
#define ADC_REF         3.3f    // VDDA reference voltage
#define ADC_RES      4095.0f    // 12-bit ADC full scale
#define NUM_SAMPLES       64    // Samples per ADC reading
#define CAL_FACTOR    0.9905f   // Gain correction (VDDA/resistor tolerance)

#define SBUS_PIN    PA0         // Future SBUS input
#define RPM_TEST_PIN SBUS_PIN   // PA0 reused for pot testing
#define RPM_PIN     PB11        // Real RPM (interrupt, pending)

#define FET_1 PB6
#define FET_2 PB5
#define FET_3 PB9
#define FET_4 PB8
#define FET_5 PB4
#define FET_6 PB7

const uint8_t FET_PINS[6] = {FET_1, FET_2, FET_3, FET_4, FET_5, FET_6};
```

### 4.3 UART2 Declaration

STM32duino does not auto-declare Serial2. Must be declared globally:

```cpp
#include <HardwareSerial.h>
HardwareSerial Serial2(PA3, PA2);   // RX=PA3, TX=PA2
```

Then in `setup()`:
```cpp
Serial2.begin(115200);
analogReadResolution(12);           // Enable 12-bit ADC (default is 10-bit)
```

### 4.4 Loop Timing (Non-Blocking)

All periodic tasks use independent `millis()` timers:

| Task | Interval | Function |
|------|----------|----------|
| Serial command receive | Every loop iteration | `handleSerial()` |
| Thermocouple read | 250 ms | `readTemperatures()` |
| Battery voltage read | 500 ms | `readBattVoltage()` |
| FET state report | 200 ms | `reportFETStates()` |
| RPM test read | 100 ms | `readRPMTest()` |
| MCU temperature read | 1000 ms | `readMCUTemp()` |

---

## 5. Sensor Implementation Details

### 5.1 Battery Voltage (PA1)

**Circuit:** Battery (+) → 39KΩ → PA1 → 5.6KΩ → GND

**ADC averaging:**
```cpp
uint16_t readADC_avg(uint8_t pin) {
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(pin);
        delayMicroseconds(200);     // Allow ADC to settle between samples
    }
    return sum / NUM_SAMPLES;
}
```

**Voltage formula:**
```cpp
float v_adc = (raw / 4095.0f) * 3.3f;
float v_batt = v_adc * ((BATT_R1 + BATT_R2) / BATT_R2) * CAL_FACTOR;
// = v_adc * (44600 / 5600) * 0.9905
// = v_adc * 7.964 * 0.9905
```

**Accuracy:** ±0.05V across 16V–24V range.
**CAL_FACTOR rationale:** VDDA slightly above 3.3V + resistor tolerance causes ~1% overshoot. Empirically measured and corrected.

**Serial output:**
```
Raw: 3741    V_ADC: 3.014 V    V_BATT: 24.01 V
```

### 5.2 Thermocouples TC1 and TC2 (MAX31855)

**Library constructor:** `Adafruit_MAX31855(SCK_pin, CS_pin, SO_pin)`

```cpp
Adafruit_MAX31855 tc1(TC1_SCK, TC1_CS, TC1_SO);
Adafruit_MAX31855 tc2(TC2_SCK, TC2_CS, TC2_SO);
```

**Reading:**
```cpp
temp1 = tc1.readCelsius();
if (isnan(temp1)) temp1 = 0;     // Returns NaN on open circuit or fault
```

**Important:** MAX31855 needs minimum 250ms between reads. Do not call faster.

**Serial output:**
```
TC1: 245.5 C  |  TC2: 32.1 C
```

### 5.3 RPM — Test Mode via Potentiometer (PA0)

**Circuit:** Potentiometer wiper → Op-amp unity-gain buffer → PA0

**Reading:**
```cpp
uint16_t raw = readADC_avg(RPM_TEST_PIN);
long rpm = map((long)raw, 0L, 4095L, 0L, 120000L);
```

Maps 0V→0 RPM, 3.3V→120,000 RPM linearly.

**Serial output:**
```
RPM: 17054
```

**Note:** PA0 is temporarily used for this. When SBUS is implemented, a different analog pin should be used for RPM testing.

### 5.4 MCU Internal Temperature Sensor

**Channel:** ADC channel 16 — accessed via `ATEMP` alias in STM32 core.

> ⚠️ `analogReadTemp()` does NOT exist in STM32 core v2.12.0. Use `analogRead(ATEMP)`.

**Formula:**
```cpp
uint16_t raw = analogRead(ATEMP);
float v    = (raw / 4095.0f) * 3.3f;
float temp = ((1.58f - v) / 0.00430f) + 25.0f;
```

**Constants:**
- `V25 = 1.58f` — voltage at 25°C for this specific chip (datasheet typical = 1.43V, varies ±0.15V chip-to-chip)
- `Avg_Slope = 0.00430f` — 4.3 mV/°C (consistent across chips)

**Calibration note:** If readings are offset, adjust V25. Each 0.01V change shifts reading by ~2.3°C.

**Serial output:**
```
MCU: 24.4 C
```

---

## 6. FET Output Control

### 6.1 Initialization

All 6 FETs set as outputs and driven LOW at startup:
```cpp
for (int i = 0; i < 6; i++) {
    pinMode(FET_PINS[i], OUTPUT);
    digitalWrite(FET_PINS[i], LOW);
}
```

### 6.2 GUI Command Parsing

The firmware reads incoming serial bytes character by character into a buffer:

```cpp
String serialBuf = "";

void handleSerial() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n') {
            serialBuf.trim();
            // Format: FET1:1  or  FET6:0
            if (serialBuf.startsWith("FET") && serialBuf[4] == ':') {
                int n   = serialBuf[3] - '1';          // '1'–'6' → index 0–5
                int val = serialBuf.substring(5).toInt();
                if (n >= 0 && n <= 5)
                    digitalWrite(FET_PINS[n], val ? HIGH : LOW);
            }
            serialBuf = "";
        } else if (c != '\r') {
            serialBuf += c;
        }
    }
}
```

### 6.3 FET State Reporting

Every 200ms the firmware broadcasts the state of all 6 FETs:
```cpp
void reportFETStates() {
    Serial2.print("FET:");
    for (int i = 0; i < 6; i++) {
        Serial2.print(digitalRead(FET_PINS[i]));
        if (i < 5) Serial2.print(",");
    }
    Serial2.println();
}
```

**Output example:**
```
FET:1,0,0,1,0,0
```

---

## 7. Serial Protocol Reference

### 7.1 Physical Layer

| Property | Value |
|----------|-------|
| Port | UART2 (PA2=TX, PA3=RX) |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Adapter | USB-TTL (must share GND with board) |

### 7.2 Firmware → PC (Output Lines)

All lines are terminated with `\r\n`.

| Line format | Interval | Example |
|-------------|----------|---------|
| `TC1: {f} C  \|  TC2: {f} C` | 250 ms | `TC1: 245.5 C  \|  TC2: 32.1 C` |
| `Raw: {d}\tV_ADC: {f} V\tV_BATT: {f} V` | 500 ms | `Raw: 3741    V_ADC: 3.014 V    V_BATT: 24.01 V` |
| `RPM: {d}` | 100 ms | `RPM: 17054` |
| `MCU: {f} C` | 1000 ms | `MCU: 24.4 C` |
| `FET:{b},{b},{b},{b},{b},{b}` | 200 ms | `FET:1,0,0,0,0,0` |
| `JET ECU V1.0 Ready` | Once on boot | — |
| `TC1 error - check wiring` | Once on boot if fault | — |

### 7.3 PC → Firmware (Input Commands)

| Command | Effect |
|---------|--------|
| `FET1:1\n` | Set FET 1 HIGH |
| `FET1:0\n` | Set FET 1 LOW |
| `FET2:1\n` … `FET6:0\n` | Same for FETs 2–6 |

The `\n` terminator is required. The firmware ignores `\r`.

---

## 8. GUI Dashboard

### 8.1 Technology Stack

| Item | Detail |
|------|--------|
| Language | Python 3 |
| GUI framework | tkinter + ttk |
| Serial library | pyserial |
| Threading | Standard `threading` module |

### 8.2 Architecture

- Serial reading runs in a **background daemon thread** — never blocks the UI
- All UI updates posted via `self.after(0, callback)` — thread-safe tkinter pattern
- Window is fixed size, non-resizable, landscape orientation

### 8.3 Display Elements

| Widget | Data Source | Range | Colour Zones |
|--------|-------------|-------|--------------|
| Tachometer (circular gauge) | `RPM:` line | 0–120,000 RPM | Green 0–80k, Yellow 80–100k, Red 100–120k |
| Battery Voltage (circular gauge) | `V_BATT:` value | 0–26V | Red 0–14V, Yellow 14–18V, Green 18–26V |
| TC1 Thermometer (vertical) | `TC1:` value | 0–900°C | Gradient blue→green→yellow→orange→red |
| TC2 Thermometer (vertical) | `TC2:` value | 0–100°C | Same gradient |
| MCU Thermometer (vertical) | `MCU:` value | -20–100°C | Same gradient, warn 70°C, crit 85°C |
| FET Panel (2×3 grid) | `FET:` line | HIGH/LOW | Green LED = HIGH, Red LED = LOW |
| Serial Log | All other lines | — | Scrolling text, max 60 lines, CLEAR button |

### 8.4 FET Bidirectional Behaviour

- **GUI → Firmware:** Clicking TOGGLE sends `FETn:1` or `FETn:0` over serial
- **Firmware → GUI:** Every 200ms the `FET:` line updates all 6 LEDs regardless of what changed them
- If firmware changes a FET via internal logic or interrupt, the GUI corrects within 200ms
- The `FET:` state line is **never written to the serial log** (would flood at 5 Hz)

### 8.5 Disconnect Behaviour

On disconnect, the GUI resets all displays to zero:
- RPM needle → 0
- Voltage needle → 0
- TC1, TC2, MCU thermometers → 0
- All 6 FET LEDs → red / LOW

### 8.6 GUI Serial Parsing (Python regex)

```python
# FET state — silent update, not logged
re.fullmatch(r"FET:([\d,]+)", line)

# Temperatures
re.search(r"TC1:\s*([\d.]+).*TC2:\s*([\d.]+)", line)

# Battery voltage
re.search(r"V_BATT:\s*([\d.]+)", line)

# RPM
re.search(r"\bRPM:\s*(\d+)", line)

# MCU temperature
re.search(r"\bMCU:\s*([\d.]+)", line)
```

---

## 9. Pending Implementation

### 9.1 SBUS Decoding (PA0)

| Property | Value |
|----------|-------|
| Protocol | Futaba SBUS |
| Baud rate | 100,000 |
| Frame format | 8E2 (8 data bits, Even parity, 2 stop bits) |
| Signal | Inverted UART (logic inverted — idle LOW) |
| Pin | PA0 |
| Frame size | 25 bytes, 11 channels × 11 bits |

Implementation requires hardware UART with inverted signal or software UART.
PA0 is currently used as `RPM_TEST_PIN` — must be freed before SBUS is activated.

### 9.2 Real RPM Measurement (PB11)

| Property | Value |
|----------|-------|
| Pin | PB11 |
| Signal | Digital pulse train from LM358 signal conditioner |
| Method | External interrupt, pulse counting |
| Calculation | `RPM = (pulse_count / pulses_per_rev) * 60000 / interval_ms` |

Implementation approach:
```cpp
volatile uint32_t pulseCount = 0;
void rpmISR() { pulseCount++; }

// In setup():
attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

// In loop() every 500ms:
uint32_t count = pulseCount; pulseCount = 0;
long rpm = (count * 60000UL) / 500;    // Adjust divisor for pulses-per-revolution
```

Remove `readRPMTest()` and its timer when real RPM is active. PA0 can then be reassigned to SBUS.

### 9.3 Engine Control State Machine

Planned states and transitions:

```
WAITING → (start command) → STARTING
STARTING → (RPM > idle threshold) → IDLING
IDLING → (throttle input) → OPERATING
OPERATING → (shutdown command) → COOLDOWN
COOLDOWN → (EGT < safe threshold) → WAITING
```

Each state controls which FETs are active (fuel valve, igniter, starter motor, etc.).
State is to be reported over serial: `STATE: IDLING`

---

## 10. File Structure

```
D:\NASTP\JET ECU\My ECU Working\
│
├── JET_ECU_V1.0\
│   ├── JET_ECU_V1.0.ino        ← Main firmware — always updated in place
│   ├── README.md                ← Short operational readme
│   └── README_V2.0.md           ← This document — full technical specification
│
├── JET ECU GUI\
│   └── ECU GUI.py               ← Python GUI dashboard — edited directly
│
└── JET ECU Design Files\
    ├── Schematic.pdf            ← PCB schematic
    └── ADC_Voltage_Test\
        └── ADC_Voltage_Test.ino ← Standalone ADC calibration test sketch
```

**Versioning rule:**
- `JET_ECU_V1.0.ino` is always overwritten with the latest code — it is the single source of truth
- Secondary test sketches (e.g. `ADC_Voltage_Test.ino`) use versioned filenames so previous iterations are preserved

---

## 11. Known Facts and Gotchas

| Issue | Detail |
|-------|--------|
| Serial2 not auto-declared | Must add `#include <HardwareSerial.h>` and `HardwareSerial Serial2(PA3, PA2)` globally |
| `analogReadTemp()` absent | Does not exist in STM32 core v2.12.0. Use `analogRead(ATEMP)` with manual formula |
| MCU temp chip variation | V25 in formula must be empirically set per chip. This board calibrated at 1.58V |
| ADC wild readings | Caused by missing common GND between TTL adapter and board. Always share GND |
| 12-bit ADC not default | Must call `analogReadResolution(12)` in `setup()`, otherwise reads are 10-bit |
| MAX31855 minimum interval | 250ms minimum between reads — calling faster returns stale or corrupt data |
| PA0 conflict | PA0 is defined as both SBUS_PIN and RPM_TEST_PIN. Currently used for pot testing. Free it before implementing SBUS |
| FET safe state | All FETs must be LOW on boot. Never leave any actuator pin floating or uninitialized |
