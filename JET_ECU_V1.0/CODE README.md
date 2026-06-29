# JET ECU V1.0

Firmware for a custom Engine Control Unit (ECU) designed to monitor and control a JET engine during testing. Built for the STM32 Blue Pill (STM32F103C8T6).

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
| RPM Test Input | PA0 | Potentiometer via op-amp — analog RPM simulation (testing only) |
| RPM Sensor | PB11 | Signal conditioned through dual LM358 — interrupt counting (pending) |
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

### RPM — Test Mode (PA0)
- Potentiometer output through an op-amp buffer connected to PA0
- 12-bit ADC value mapped linearly to 0–120,000 RPM
- 64-sample averaging for noise reduction
- Reported every 100ms
- To be replaced with real pulse-counting on PB11 when RPM interrupt is implemented

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
RPM: 17054
MCU: 24.4 C
FET:1,0,0,0,0,0
```

### Input (PC → firmware)

| Command | Action |
|---------|--------|
| `FET1:1` | Turn FET 1 HIGH |
| `FET1:0` | Turn FET 1 LOW |
| `FET2:1` … `FET6:0` | Same for FETs 2–6 |

---

## GUI Dashboard

A Python-based ground station (`JET ECU GUI/ECU GUI.py`) connects over serial and displays all channels in real time:

- Tachometer and battery voltage circular gauges
- Vertical thermometer gauges for TC1, TC2, and MCU temperature
- FET output panel — live status LEDs and toggle buttons (2 × 3 grid)
- Serial log with clear button

---

## What is Pending

- [ ] SBUS decoding (PA0) — read RC receiver channels (inverted UART, 100000 baud, 8E2)
- [ ] RPM measurement (PB11) — pulse counting via external interrupt
- [ ] Engine control state machine — Waiting → Starting → Idling → Operating → Cooldown
