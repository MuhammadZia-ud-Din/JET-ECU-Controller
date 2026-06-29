# JET ECU V1.0

Firmware for a custom Engine Control Unit (ECU) designed to monitor and control a JET engine during testing. Built for the STM32F103C8T6 (Blue Pill).

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F103C8T6 (Blue Pill) |
| Serial | UART2 via TTL adapter (PA2=TX, PA3=RX) at 115200 baud — bidirectional |

---

## Pin Map

| Function | Pin | Notes |
|----------|-----|-------|
| Thermocouple 1 SCK | PA5 | EGT / Exhaust — MAX31855 software SPI |
| Thermocouple 1 SO | PA6 | |
| Thermocouple 1 CS | PB0 | |
| Thermocouple 2 SCK | PB13 | Intake / Ambient — MAX31855 software SPI |
| Thermocouple 2 SO | PB14 | |
| Thermocouple 2 CS | PB12 | |
| Battery Voltage | PA1 | Voltage divider: 39K + 5.6K, max input ~24V |
| RC PWM Input | PA0 | R12DS CH3 signal wire — standard 1000–2000µs PWM |
| RPM Sensor | PC14 | Future pulse-counting input (not yet implemented) |
| FET 1 | PB6 | |
| FET 2 | PB5 | |
| FET 3 | PB9 | |
| FET 4 | PB8 | |
| FET 5 | PB4 | |
| FET 6 | PB7 | |
| MCU Temperature | Internal | ADC channel 16 — die temperature sensor |

---

## Libraries

- `Adafruit_MAX31855` — thermocouple temperature reading
- `HardwareSerial` — UART2 serial communication

---

## What is Implemented

### Battery Voltage (PA1)
- 12-bit ADC with 64-sample averaging for noise reduction
- Voltage divider scaling: 39K + 5.6K resistors
- Calibration factor (0.9905) applied to correct ~1% gain error from VDDA/resistor tolerance
- Accuracy: within ±0.05V across 16V–24V input range
- Reported every 500ms

### Thermocouple Reading (TC1 + TC2)
- TC1 reads exhaust gas temperature (EGT) via MAX31855
- TC2 reads intake / ambient temperature via MAX31855
- NaN fault handling — returns 0.0 on sensor fault or open circuit
- Reported every 250ms

### FET Outputs (PB4–PB9)
- All 6 FET pins configured as outputs, driven LOW at startup (safe state)
- Accepts real-time toggle commands from the GUI over serial: `FET1:1` / `FET1:0`
- Reports all 6 FET states every 200ms: `FET:1,0,0,0,0,0`

### RC Throttle Input (PA0)
- Reads standard RC PWM signal from R12DS receiver CH3 pin
- Pulse width range: 1000–2000µs mapped to 0–100% throttle
- Spike rejection: rejects any reading that jumps more than 400µs from last valid
- Hold-last-valid: maintains last known value; resets to 0 only after 500ms signal loss
- Calibration via GUI: SET RC wizard captures center and full-throttle positions
- Reported every 200ms as `THR: xxx`

### MCU Internal Temperature
- Reads the STM32F103 on-chip die temperature sensor (ADC channel 16)
- Calibrated V25 = 1.58V, slope = 4.3 mV/°C
- Accuracy: ±2–3°C (sensor varies chip-to-chip; value reflects die temperature, not ambient)
- Reported every 1000ms

---

## Serial Interface

Connect a USB-TTL adapter to PA2 (TX) and PA3 (RX) at **115200 baud**.

### Output (firmware → PC)

```
JET ECU V1.0 Ready
TC1: 245.5 C  |  TC2: 32.1 C
Raw: 3741    V_ADC: 3.014 V    V_BATT: 24.01 V
THR: 75
MCU: 24.4 C
FET:1,0,0,0,0,0
```

### Input (PC → firmware)

| Command | Action |
|---------|--------|
| `FET1:1` | Turn FET 1 HIGH |
| `FET1:0` | Turn FET 1 LOW |
| `FET2:1` … `FET6:0` | Same for FETs 2–6 |
| `CAL_CENTER` | Sample RC center position (hold stick at center first) |
| `CAL_FULL` | Sample RC full throttle position (push stick to full first) |
| `CAL_RESET` | Clear calibration — throttle returns 0 until recalibrated |

---

## GUI Dashboard

A Python-based ground station (`JET ECU GUI/ECU GUI.py`) connects over serial and displays all channels in real time:

- **THROTTLE gauge** — RC input 0–100%
- **TACHOMETER gauge** — RPM sensor 0–50,000 (PC14, pending implementation)
- **Battery voltage** circular gauge
- **Vertical thermometers** for TC1 (EGT), TC2 (Intake), and MCU temperature
- **FET panel** — live status LEDs and toggle buttons (layout: 1/3/5 top, 2/4/6 bottom)
- **SET RC / RESET RC** calibration buttons
- Serial log with CLEAR button

---

## What is Pending

- [ ] RPM sensor (PC14) — pulse counting via external interrupt, output `RPM: xxx`
