# JET ECU Controller

A custom Engine Control Unit (ECU) for monitoring and controlling a small JET engine during ground testing.

The system consists of STM32F103C8T6 firmware and a Python GUI dashboard connected over serial.

---

## Repository Structure

```
JET ECU Firmware/                    — Active firmware (STM32CubeIDE + FreeRTOS)
├── Core/Src/main.c                  — All application code (FreeRTOS tasks, serial protocol)
├── Core/Inc/main.h                  — Pin definitions
└── JET_ECU_Project_Documentation.docx — Full project documentation

JET ECU GUI/
└── ECU GUI.py                       — Python dashboard GUI

JET_ECU_V1.0/
└── JET_ECU_V1.0.ino                 — Original Arduino firmware (reference only)
```

---

## Firmware

The active firmware runs on **FreeRTOS (CMSIS-RTOS v2)** with four concurrent tasks:

| Task | Priority | Rate | Role |
|------|----------|------|------|
| ThrottleTask | AboveNormal | 200 ms | RC PWM reading, spike filter, 0–100% output |
| VoltageTask | Normal | 500 ms | Battery voltage (64-sample avg), MCU temperature |
| FETReportTask | Normal | 200 ms | Reports all 6 FET states over serial |
| SerialRxTask | High | Event | Parses incoming commands from GUI |

Built with **STM32CubeIDE**. All user code is inside `USER CODE BEGIN / END` sections and survives CubeMX regeneration.

---

## Features

- **Dual thermocouple monitoring** — EGT (exhaust) and intake temperature via MAX31855
- **Battery voltage** — 12-bit ADC, 64-sample averaging, two-point raw-count calibration, up to 26V
- **RC throttle input** — GPIO pulseIn via DWT counter, default calibration 1540–2000 µs → 0–100%
- **6-channel FET control** — Toggle individual FETs or use ALL HIGH / ALL LOW group control
- **MCU temperature** — On-chip die temperature sensor
- **Python GUI** — Live gauges, 2×4 FET panel, RC calibration wizard, serial log

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32F103C8T6 (Blue Pill), 64 MHz |
| Thermocouple amplifiers | MAX31855 × 2 (SPI1 + SPI2) |
| RC receiver | R12DS — CH3 PWM signal to PA0 (GPIO Input) |
| Serial link | UART2 (PA2/PA3) via USB-TTL adapter at 115200 baud |
| Battery input | Up to ~26V via 39K + 5.6K voltage divider on PA1 |

---

## GUI Requirements

```
pip install pyserial
```

Python 3 with `tkinter` (included in standard Python install). No internet required at runtime.

---

## Quick Start

1. Open `JET ECU Firmware/` in STM32CubeIDE and flash to the Blue Pill
2. Connect USB-TTL adapter to PA2 (TX) and PA3 (RX), share GND
3. Run `ECU GUI.py`
4. Select COM port, set baud to 115200, click CONNECT
5. Throttle works immediately — use **SET RC** only if you need to fine-tune to your transmitter

---

## Serial Protocol

| Direction | Format | Example |
|-----------|--------|---------|
| Firmware → PC | `THR: xxx` | `THR: 75` |
| Firmware → PC | `TC1: x.x C  \|  TC2: x.x C` | `TC1: 245.5 C  \|  TC2: 32.1 C` |
| Firmware → PC | `Raw: xxxx   V_ADC: x.xxx V   V_BATT: xx.xx V` | `V_BATT: 23.85 V` |
| Firmware → PC | `MCU: xx.x C` | `MCU: 24.4 C` |
| Firmware → PC | `FET:x,x,x,x,x,x` | `FET:1,0,0,0,0,0` |
| PC → Firmware | `FETn:1` / `FETn:0` | `FET3:1` |
| PC → Firmware | `FET_ALL:1` / `FET_ALL:0` | All 6 FETs HIGH or LOW |
| PC → Firmware | `CAL_CENTER` / `CAL_FULL` / `CAL_RESET` | RC throttle calibration |
