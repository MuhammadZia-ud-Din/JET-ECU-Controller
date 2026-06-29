# JET ECU Controller

A custom Engine Control Unit (ECU) for monitoring and controlling a small JET engine during ground testing.

The system consists of STM32F103C8T6 firmware and a Python GUI dashboard connected over serial.

---

## Repository Structure

```
JET_ECU_V1.0/
├── JET_ECU_V1.0.ino        — Main firmware
└── CODE README.md           — Firmware documentation

JET ECU GUI/
└── ECU GUI.py               — Python dashboard GUI
```

---

## Features

- **Dual thermocouple monitoring** — EGT (exhaust) and intake temperature via MAX31855
- **Battery voltage** — 12-bit ADC with 64-sample averaging, up to 24V
- **RC throttle input** — Standard PWM from RC receiver, 0–100%, GUI-calibrated
- **6-channel FET control** — Toggle engine actuators/valves from the GUI over serial
- **MCU temperature** — On-chip die temperature sensor
- **Python GUI** — Live gauges, FET panel, serial log — runs offline on any PC with Python

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32F103C8T6 (Blue Pill) |
| Thermocouple amplifiers | MAX31855 × 2 |
| RC receiver | R12DS — CH3 PWM signal to PA0 |
| Serial link | UART2 (PA2/PA3) via USB-TTL adapter at 115200 baud |
| Battery input | Up to 24V via 39K + 5.6K voltage divider on PA1 |

---

## GUI Requirements

```
pip install pyserial
```

Python 3 with `tkinter` (included in standard Python install). No internet required at runtime.

---

## Quick Start

1. Flash `JET_ECU_V1.0.ino` to the Blue Pill
2. Connect USB-TTL adapter to PA2 (TX) and PA3 (RX), share GND
3. Run `ECU GUI.py`
4. Select COM port, set baud to 115200, click CONNECT
5. Use **SET RC** to calibrate the throttle input

---

## Serial Protocol

| Direction | Format | Example |
|-----------|--------|---------|
| Firmware → PC | `THR: xxx` | `THR: 75` |
| Firmware → PC | `TC1: x.x C  \|  TC2: x.x C` | `TC1: 245.5 C  \|  TC2: 32.1 C` |
| Firmware → PC | `V_BATT: xx.xx V` | `V_BATT: 23.85 V` |
| Firmware → PC | `FET:x,x,x,x,x,x` | `FET:1,0,0,0,0,0` |
| PC → Firmware | `FETn:1` / `FETn:0` | `FET3:1` |
| PC → Firmware | `CAL_CENTER` / `CAL_FULL` / `CAL_RESET` | RC calibration |
