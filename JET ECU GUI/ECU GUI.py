#!/usr/bin/env python3
"""
JET ECU Dashboard — Real-time engine monitor
STM32F103C8T6 ECU telemetry display
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import math
import re

# ── Palette ────────────────────────────────────────────────────────────────────
BG     = "#0a0e1a"
CARD   = "#111827"
CARD2  = "#1a2234"
BORDER = "#1e2d47"
ACCENT = "#00d4ff"
ORANGE = "#ff6b00"
GREEN  = "#00ff88"
RED    = "#ff3355"
YELLOW = "#ffcc00"
TEXT   = "#e0e6f0"
DIM    = "#4a5568"
WHITE  = "#ffffff"


class GaugeCanvas(tk.Canvas):
    """Circular needle gauge — tachometer / voltmeter style."""

    def __init__(self, parent, size, min_val, max_val,
                 unit, color_zones=None, **kw):
        super().__init__(parent, width=size, height=size,
                         bg=CARD, highlightthickness=0, **kw)
        self.cx = size // 2
        self.cy = size // 2
        self.R  = size // 2 - 30
        self.mn, self.mx = min_val, max_val
        self.unit = unit
        self.zones = color_zones or []
        self._ORIG  = 225
        self._SWEEP = 270
        self._draw_face()
        self._update(min_val)

    def _ang(self, v):
        f = max(0.0, min(1.0, (v - self.mn) / (self.mx - self.mn)))
        return self._ORIG - f * self._SWEEP

    def _pt(self, deg, r):
        rad = math.radians(deg)
        return self.cx + r * math.cos(rad), self.cy - r * math.sin(rad)

    def _draw_face(self):
        s = self.cx * 2
        self.create_oval(5, 5, s-5, s-5, outline=BORDER, width=2)
        R = self.R
        a0, a1 = self._ang(self.mn), self._ang(self.mx)
        self.create_arc(self.cx-R, self.cy-R, self.cx+R, self.cy+R,
                        start=a0, extent=a1-a0,
                        outline=BORDER, width=8, style=tk.ARC)
        for z0, z1, col in self.zones:
            za0, za1 = self._ang(z0), self._ang(z1)
            self.create_arc(self.cx-R, self.cy-R, self.cx+R, self.cy+R,
                            start=za0, extent=za1-za0,
                            outline=col, width=8, style=tk.ARC)
        for i in range(51):
            ang   = self._ORIG - (i/50) * self._SWEEP
            major = (i % 5 == 0)
            ro, ri = R-6, R-6-(13 if major else 6)
            x1, y1 = self._pt(ang, ro)
            x2, y2 = self._pt(ang, ri)
            self.create_line(x1, y1, x2, y2,
                             fill=TEXT if major else DIM,
                             width=2 if major else 1)
        for i in range(11):
            ang = self._ORIG - (i/10) * self._SWEEP
            val = self.mn + (i/10) * (self.mx - self.mn)
            lx, ly = self._pt(ang, R-30)
            s = (f"{int(val/1000)}k" if val else "0") if self.mx >= 10000 else f"{val:.0f}"
            self.create_text(lx, ly, text=s, fill=DIM, font=("Consolas", 8))

    def _update(self, value):
        self.delete("needle"); self.delete("digit")
        ang    = self._ang(value)
        tx, ty = self._pt(ang,       self.R - 15)
        bx, by = self._pt(ang + 180, 18)
        self.create_line(bx, by, tx, ty, fill=ORANGE, width=3,
                         capstyle=tk.ROUND, tags="needle")
        pr = 9
        self.create_oval(self.cx-pr, self.cy-pr, self.cx+pr, self.cy+pr,
                         fill=ORANGE, outline="", tags="needle")

        # Digital readout text — sits in the blank 90° wedge at the bottom of
        # the dial (the gauge sweep + tick labels stop at ±45° from vertical,
        # leaving this gap genuinely empty; needle tail only reaches 18px).
        vs = (f"{int(value):,}" if self.mx >= 10000
              else f"{value:.1f}" if self.mx > 30
              else f"{value:.2f}")
        self.create_text(self.cx, self.cy + int(self.R * 0.90), text=f"{vs} {self.unit}",
                         fill=WHITE, font=("Consolas", 14, "bold"), tags="digit")

    def set_value(self, v): self._update(v)


class ThermometerGauge(tk.Canvas):
    """Vertical thermometer with smooth gradient fill — cold blue to hot red."""

    _W  = 110
    _TW = 16
    _BR = 20

    def __init__(self, parent, height, min_val, max_val,
                 label, unit, warn_val, crit_val, **kw):
        super().__init__(parent, width=self._W, height=height,
                         bg=CARD, highlightthickness=0, **kw)
        self.mn, self.mx = min_val, max_val
        self.label, self.unit = label, unit
        self.cx    = 48
        self.t_top = 40
        self.t_bot = height - self._BR - 20
        self.bcy   = self.t_bot
        self.t_h   = self.t_bot - self.t_top
        self._draw_static()
        self._update(min_val)

    @staticmethod
    def _lerp(c1, c2, t):
        r1,g1,b1 = int(c1[1:3],16),int(c1[3:5],16),int(c1[5:7],16)
        r2,g2,b2 = int(c2[1:3],16),int(c2[3:5],16),int(c2[5:7],16)
        return (f"#{int(r1+(r2-r1)*t):02x}"
                f"{int(g1+(g2-g1)*t):02x}"
                f"{int(b1+(b2-b1)*t):02x}")

    def _grad(self, f):
        stops = [(0.00,"#0066ff"),(0.30,"#00ff88"),
                 (0.60,"#ffcc00"),(0.80,"#ff6b00"),(1.00,"#ff3355")]
        for i in range(len(stops)-1):
            f0,c0 = stops[i]; f1,c1 = stops[i+1]
            if f <= f1:
                return self._lerp(c0, c1, (f-f0)/(f1-f0))
        return stops[-1][1]

    def _fy(self, v):
        f = max(0.0, min(1.0, (v-self.mn)/(self.mx-self.mn)))
        return self.t_bot - f * self.t_h

    def _draw_static(self):
        cx, tw = self.cx, self._TW
        self.create_text(cx, 11, text=self.label, fill=ACCENT,
                         font=("Consolas", 8, "bold"), anchor="center")
        self.create_text(cx, 25, text=self.unit, fill=DIM,
                         font=("Consolas", 7), anchor="center")
        n = 64
        for i in range(n):
            f_b = i / n;  f_t = (i+1) / n
            y_b = self.t_bot - f_b * self.t_h
            y_t = self.t_bot - f_t * self.t_h
            self.create_rectangle(cx-tw//2+1, y_t, cx+tw//2-1, y_b,
                                  fill=self._grad(f_t), outline="",
                                  tags="gfill")
        for i in range(11):
            f   = i / 10
            y   = self.t_bot - f * self.t_h
            val = self.mn + f * (self.mx - self.mn)
            maj = (i % 2 == 0)
            x1  = cx + tw // 2
            self.create_line(x1, y, x1+(7 if maj else 3), y,
                             fill=TEXT if maj else DIM, width=1)
            if maj:
                self.create_text(x1+10, y, anchor="w", fill=DIM,
                                text=f"{int(val)}", font=("Consolas", 6))

    def _update(self, value):
        self.delete("dyn")
        cx, tw, br, bcy = self.cx, self._TW, self._BR, self.bcy
        fy   = self._fy(value)
        frac = max(0.0, min(1.0, (value-self.mn)/(self.mx-self.mn)))
        if fy > self.t_top:
            self.create_rectangle(cx-tw//2+1, self.t_top,
                                  cx+tw//2-1, fy,
                                  fill=CARD, outline="", tags="dyn")
        self.create_rectangle(cx-tw//2, self.t_top, cx+tw//2, self.t_bot,
                              fill="", outline=DIM, width=1, tags="dyn")
        bc = self._grad(frac)
        self.create_oval(cx-br+1, bcy-br+1, cx+br-1, bcy+br-1,
                         fill=bc, outline="", tags="dyn")
        self.create_oval(cx-br, bcy-br, cx+br, bcy+br,
                         fill="", outline=DIM, width=1, tags="dyn")
        self.create_text(cx, bcy, text=f"{value:.0f}",
                         fill=WHITE, font=("Consolas", 9, "bold"), tags="dyn")

    def set_value(self, v): self._update(v)


class FETBtn(tk.Frame):
    """Horizontal FET row:  LED  FET N  STATUS  [TOGGLE]"""

    def __init__(self, parent, n, cb, **kw):
        super().__init__(parent, bg=CARD2, **kw)
        self.n, self.state, self.cb = n, False, cb
        tk.Button(self, text="TOGGLE", bg=BORDER, fg=TEXT,
                  font=("Consolas", 9, "bold"), relief="flat",
                  padx=14, pady=6, cursor="hand2",
                  activebackground=ORANGE, activeforeground=WHITE,
                  command=self._toggle).pack(side="right", padx=(0, 8), pady=4)
        self.led = tk.Canvas(self, width=14, height=14,
                             bg=CARD2, highlightthickness=0)
        self.led.pack(side="left", padx=(8, 4), pady=4)
        self._dot = self.led.create_oval(2, 2, 12, 12, fill=RED, outline="")
        tk.Label(self, text=f"FET {n}", bg=CARD2, fg=WHITE,
                 font=("Consolas", 9, "bold"),
                 anchor="w").pack(side="left", padx=(0, 3))
        self.slbl = tk.Label(self, text="LOW", bg=CARD2, fg=RED,
                              font=("Consolas", 8, "bold"), width=4,
                              anchor="w")
        self.slbl.pack(side="left")

    def _toggle(self):
        self.state = not self.state
        self._refresh()
        self.cb(self.n, self.state)

    def set_state(self, val):
        self.state = bool(val)
        self._refresh()

    def _refresh(self):
        self.led.itemconfig(self._dot, fill=GREEN if self.state else RED)
        self.slbl.config(text="HIGH" if self.state else "LOW",
                          fg=GREEN if self.state else RED)


class AllFETBtn(tk.Frame):
    """ALL HIGH / ALL LOW — same visual as FETBtn, fixed state."""

    def __init__(self, parent, high, cb, **kw):
        super().__init__(parent, bg=CARD2, **kw)
        color = GREEN if high else RED
        label = "HIGH" if high else "LOW "
        act   = "HIGH" if high else "LOW"
        tk.Button(self, text=act, bg=BORDER, fg=TEXT,
                  font=("Consolas", 9, "bold"), relief="flat",
                  padx=14, pady=6, cursor="hand2",
                  activebackground=color, activeforeground=WHITE,
                  command=cb).pack(side="right", padx=(0, 8), pady=4)
        led = tk.Canvas(self, width=14, height=14, bg=CARD2, highlightthickness=0)
        led.pack(side="left", padx=(8, 4), pady=4)
        led.create_oval(2, 2, 12, 12, fill=color, outline="")
        tk.Label(self, text="ALL", bg=CARD2, fg=WHITE,
                 font=("Consolas", 9, "bold"),
                 anchor="w").pack(side="left", padx=(0, 3))
        tk.Label(self, text=label, bg=CARD2, fg=color,
                 font=("Consolas", 8, "bold"), width=4,
                 anchor="w").pack(side="left")


class Dashboard(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("JET ECU — Engine Monitor")
        self.configure(bg=BG)
        self.resizable(False, False)
        self._port       = None
        self._running    = False
        self._cal_active = False
        self._build()
        self._refresh_ports()

    def _build(self):
        # ── Header ────────────────────────────────────────────────────────────
        h = tk.Frame(self, bg=BG)
        h.pack(fill="x", padx=20, pady=(12, 4))
        tk.Label(h, text="● JET ENGINE ECU  MONITOR", bg=BG, fg=ACCENT,
                 font=("Consolas", 13, "bold")).pack(side="left")
        self._status = tk.Label(h, text="DISCONNECTED", bg=BG, fg=RED,
                                 font=("Consolas", 10, "bold"))
        self._status.pack(side="right")

        # ── Connection bar ─────────────────────────────────────────────────────
        cb = tk.Frame(self, bg=CARD, pady=6)
        cb.pack(fill="x", padx=20, pady=4)
        tk.Label(cb, text=" PORT:", bg=CARD, fg=WHITE,
                 font=("Consolas", 9, "bold")).pack(side="left")
        self._pvar = tk.StringVar()
        self._pcb  = ttk.Combobox(cb, textvariable=self._pvar,
                                   width=9, font=("Consolas", 9),
                                   state="readonly")
        self._pcb.pack(side="left", padx=3)
        tk.Label(cb, text="BAUD:", bg=CARD, fg=WHITE,
                 font=("Consolas", 9, "bold")).pack(side="left", padx=(8, 0))
        self._baud = tk.StringVar(value="115200")
        ttk.Combobox(cb, textvariable=self._baud,
                     values=["9600","38400","57600","115200"],
                     width=8, font=("Consolas", 9),
                     state="readonly").pack(side="left", padx=3)
        self._cbtn = tk.Button(cb, text="CONNECT",
                                bg=GREEN, fg="#000",
                                font=("Consolas", 9, "bold"),
                                relief="flat", padx=14, pady=3,
                                cursor="hand2", command=self._toggle_conn)
        self._cbtn.pack(side="left", padx=10)
        tk.Button(cb, text="⟳", bg=CARD, fg=ACCENT,
                  font=("Consolas", 11), relief="flat",
                  cursor="hand2",
                  command=self._refresh_ports).pack(side="left")

        # ── Throttle calibration buttons ───────────────────────────────────────
        self._cal_btn = tk.Button(cb, text="SET RC",
                                   bg=ORANGE, fg=WHITE,
                                   font=("Consolas", 8, "bold"),
                                   relief="flat", padx=10, pady=3,
                                   cursor="hand2",
                                   command=self._start_cal)
        self._cal_btn.pack(side="left", padx=(16, 4))
        tk.Button(cb, text="RESET RC",
                  bg=BORDER, fg=TEXT,
                  font=("Consolas", 8, "bold"),
                  relief="flat", padx=10, pady=3,
                  cursor="hand2",
                  activebackground=RED, activeforeground=WHITE,
                  command=self._reset_rpm).pack(side="left", padx=4)

        # ── Emergency stop — always visible, forces all FETs off instantly ─────
        tk.Button(cb, text="⛔ EMERGENCY STOP", bg=RED, fg=WHITE,
                  font=("Consolas", 9, "bold"), relief="flat",
                  padx=14, pady=3, cursor="hand2",
                  activebackground="#cc0022", activeforeground=WHITE,
                  command=self._emergency_stop).pack(side="right", padx=10)

        # ── Instrument row ─────────────────────────────────────────────────────
        GAUGE_H = 290
        THERM_H = 290

        gr = tk.Frame(self, bg=BG)
        gr.pack(padx=20, pady=8)

        # Throttle — RC receiver input (0–100 %)
        thc = tk.Frame(gr, bg=CARD)
        thc.grid(row=0, column=0, padx=(0, 6))
        tk.Label(thc, text="THROTTLE", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._thr = GaugeCanvas(thc, size=GAUGE_H,
                                min_val=0, max_val=100,
                                unit="%",
                                color_zones=[
                                    (0,  70,  GREEN),
                                    (70, 90,  YELLOW),
                                    (90, 100, RED),
                                ])
        self._thr.pack(padx=10, pady=(4, 10))

        # Tachometer — RPM sensor on PC14
        rc = tk.Frame(gr, bg=CARD)
        rc.grid(row=0, column=1, padx=6)
        tk.Label(rc, text="TACHOMETER", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._rpm = GaugeCanvas(rc, size=GAUGE_H,
                                 min_val=0, max_val=50000,
                                 unit="RPM",
                                 color_zones=[
                                     (0,     35000,  GREEN),
                                     (35000, 45000,  YELLOW),
                                     (45000, 50000,  RED),
                                 ])
        self._rpm.pack(padx=10, pady=(4, 10))

        vc = tk.Frame(gr, bg=CARD)
        vc.grid(row=0, column=2, padx=6)
        tk.Label(vc, text="BATTERY VOLTAGE", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._volt = GaugeCanvas(vc, size=GAUGE_H,
                                  min_val=0, max_val=26,
                                  unit="V DC",
                                  color_zones=[
                                      (0,  14,   RED),
                                      (14, 18, YELLOW),
                                      (18, 26,  GREEN),
                                  ])
        self._volt.pack(padx=10, pady=(4, 10))

        t1c = tk.Frame(gr, bg=CARD)
        t1c.grid(row=0, column=3, padx=6)
        tk.Label(t1c, text="EXHAUST / EGT", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._tc1 = ThermometerGauge(t1c, height=THERM_H,
                                      min_val=0, max_val=900,
                                      label="TC1", unit="°C",
                                      warn_val=700, crit_val=850)
        self._tc1.pack(padx=14, pady=(4, 10))

        t2c = tk.Frame(gr, bg=CARD)
        t2c.grid(row=0, column=4, padx=6)
        tk.Label(t2c, text="INTAKE / AMBIENT", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._tc2 = ThermometerGauge(t2c, height=THERM_H,
                                      min_val=0, max_val=100,
                                      label="TC2", unit="°C",
                                      warn_val=60, crit_val=85)
        self._tc2.pack(padx=14, pady=(4, 10))

        mc = tk.Frame(gr, bg=CARD)
        mc.grid(row=0, column=5, padx=(6, 0))
        tk.Label(mc, text="MCU TEMP", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(pady=(8, 0))
        self._mcu = ThermometerGauge(mc, height=THERM_H,
                                      min_val=-20, max_val=100,
                                      label="MCU", unit="°C",
                                      warn_val=70, crit_val=85)
        self._mcu.pack(padx=14, pady=(4, 10))

        # ── FET panel ─────────────────────────────────────────────────────────
        fc = tk.Frame(self, bg=CARD)
        fc.pack(fill="x", padx=20, pady=(0, 8))
        tk.Label(fc, text=" FET OUTPUTS", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(anchor="w", pady=(7, 4))
        fr = tk.Frame(fc, bg=CARD)
        fr.pack(fill="x", padx=8, pady=(0, 6))
        self._fets = []
        for i in range(1, 7):
            b = FETBtn(fr, i, cb=self._send_fet)
            b.grid(row=(i-1)%2, column=(i-1)//2, padx=4, pady=1, sticky="ew")
            self._fets.append(b)
        AllFETBtn(fr, high=True,  cb=self._fet_all_high).grid(
            row=0, column=3, padx=4, pady=1, sticky="ew")
        AllFETBtn(fr, high=False, cb=self._fet_all_low).grid(
            row=1, column=3, padx=4, pady=1, sticky="ew")
        for col in range(4):
            fr.columnconfigure(col, weight=1)

        # ── Serial log ────────────────────────────────────────────────────────
        lc = tk.Frame(self, bg=CARD)
        lc.pack(fill="x", padx=20, pady=(0, 14))
        log_hdr = tk.Frame(lc, bg=CARD)
        log_hdr.pack(fill="x")
        tk.Label(log_hdr, text=" SERIAL LOG", bg=CARD, fg=ACCENT,
                 font=("Consolas", 9, "bold")).pack(side="left", pady=(6, 2))
        tk.Button(log_hdr, text="CLEAR LOG", bg=BORDER, fg=TEXT,
                  font=("Consolas", 7, "bold"), relief="flat",
                  padx=10, pady=2, cursor="hand2",
                  activebackground=RED, activeforeground=WHITE,
                  command=self._clear_log).pack(side="right", padx=8, pady=4)
        self._log = tk.Text(lc, height=8, bg="#060c16", fg=GREEN,
                            font=("Consolas", 9), relief="flat",
                            state="disabled", insertbackground=GREEN)
        self._log.pack(fill="x", padx=8, pady=(0, 8))

        # ── Combobox style ─────────────────────────────────────────────────────
        sty = ttk.Style()
        sty.theme_use("clam")
        sty.configure("TCombobox",
                       fieldbackground="#000000", background="#000000",
                       foreground="#ffffff", selectbackground=ACCENT,
                       selectforeground="#000000", arrowcolor=ACCENT,
                       insertcolor="#ffffff")
        sty.map("TCombobox",
                fieldbackground=[("readonly","#000000"),("disabled","#000000")],
                foreground=[("readonly","#ffffff"),("disabled","#ffffff")],
                background=[("readonly","#000000")])
        self.option_add("*TCombobox*Listbox.background",       "#000000")
        self.option_add("*TCombobox*Listbox.foreground",       "#ffffff")
        self.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        self.option_add("*TCombobox*Listbox.selectForeground", "#000000")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._pcb["values"] = ports
        if ports: self._pcb.current(0)

    def _toggle_conn(self):
        (self._disconnect if self._running else self._connect)()

    def _connect(self):
        p = self._pvar.get()
        if not p: return
        try:
            self._port    = serial.Serial(p, int(self._baud.get()), timeout=1)
            self._running = True
            threading.Thread(target=self._reader, daemon=True).start()
            self._status.config(text=f"CONNECTED  {p}", fg=GREEN)
            self._cbtn.config(text="DISCONNECT", bg=RED, fg=WHITE)
        except Exception as e:
            self._status.config(text=f"ERROR: {e}", fg=RED)

    def _disconnect(self):
        self._running = False
        if self._port:
            try:
                if self._port.is_open:
                    self._port.close()
            except Exception:
                pass
        self._status.config(text="DISCONNECTED", fg=RED)
        self._cbtn.config(text="CONNECT", bg=GREEN, fg="#000")
        self._thr.set_value(0)
        self._rpm.set_value(0)
        self._volt.set_value(0)
        self._tc1.set_value(0)
        self._tc2.set_value(0)
        self._mcu.set_value(0)
        for fet in self._fets:
            fet.set_state(0)

    def _connection_lost(self):
        if not self._running:
            return  # already torn down by an explicit DISCONNECT click
        self._disconnect()
        self._status.config(text="ERROR: USB-TTL disconnected", fg=RED)
        self._push("[ERROR] Serial connection lost — USB-TTL adapter disconnected")

    def _reader(self):
        while self._running:
            try:
                if self._port.in_waiting:
                    line = self._port.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.after(0, self._parse, line)
            except Exception:
                self.after(0, self._connection_lost)
                break

    def _parse(self, line):
        # FET state report — update silently
        m = re.fullmatch(r"FET:([\d,]+)", line)
        if m:
            for idx, v in enumerate(m.group(1).split(",")):
                if idx < 6 and v.isdigit():
                    self._fets[idx].set_state(int(v))
            return

        # Calibration responses — handle silently
        m = re.fullmatch(r"CAL_CENTER:(\d+)", line)
        if m:
            self._cal_step2(int(m.group(1)))
            return
        m = re.fullmatch(r"CAL_FULL:(\d+)", line)
        if m:
            self._cal_done(int(m.group(1)))
            return
        if line == "CAL_RESET:OK":
            return

        self._push(line)
        m = re.search(r"TC1:\s*([\d.]+).*TC2:\s*([\d.]+)", line)
        if m:
            self._tc1.set_value(float(m.group(1)))
            self._tc2.set_value(float(m.group(2)))
        m = re.search(r"V_BATT:\s*([\d.]+)", line)
        if m:
            self._volt.set_value(float(m.group(1)))
        m = re.search(r"\bTHR:\s*(\d+)", line)
        if m:
            self._thr.set_value(int(m.group(1)))
        m = re.search(r"\bRPM:\s*(\d+)", line)
        if m:
            self._rpm.set_value(int(m.group(1)))
        m = re.search(r"\bMCU:\s*([\d.]+)", line)
        if m:
            self._mcu.set_value(float(m.group(1)))

    def _send_cmd(self, cmd):
        if self._port and self._port.is_open:
            try: self._port.write(cmd.encode())
            except Exception: pass

    def _send_fet(self, n, state):
        cmd = f"FET{n}:{'1' if state else '0'}\n"
        if self._port and self._port.is_open:
            try: self._port.write(cmd.encode())
            except Exception: pass
        self._push(f"[CMD] {cmd.strip()}")

    def _fet_all_high(self):
        self._send_cmd("FET_ALL:1\n")
        for fet in self._fets:
            fet.set_state(1)
        self._push("[CMD] FET_ALL:1")

    def _fet_all_low(self):
        self._send_cmd("FET_ALL:0\n")
        for fet in self._fets:
            fet.set_state(0)
        self._push("[CMD] FET_ALL:0")

    def _emergency_stop(self):
        self._fet_all_low()
        self._push("[EMERGENCY STOP] All FET outputs forced LOW")

    # ── Throttle calibration wizard ───────────────────────────────────────────

    def _start_cal(self):
        if not (self._port and self._port.is_open):
            messagebox.showwarning("Not Connected", "Connect to ECU first.")
            return
        if self._cal_active:
            return
        if not messagebox.askyesno(
                "Throttle Calibration",
                "This will pause all telemetry for ~3 seconds.\n\nContinue?"):
            return
        if messagebox.askokcancel(
                "Step 1 of 2",
                "Set throttle to CENTER position (zero RPM).\n\nClick OK when ready."):
            self._cal_active = True
            self._cal_btn.config(state="disabled")
            self._send_cmd("CAL_CENTER\n")
            self._push("[CAL] Sampling center position...")

    def _cal_step2(self, center_val):
        self._push(f"[CAL] Center captured: {center_val} µs")
        if messagebox.askokcancel(
                "Step 2 of 2",
                f"Center position captured ({center_val} µs).\n\n"
                "Now push throttle to FULL.\n\nClick OK when ready."):
            self._send_cmd("CAL_FULL\n")
            self._push("[CAL] Sampling full throttle...")
        else:
            self._cal_active = False
            self._cal_btn.config(state="normal")

    def _cal_done(self, full_val):
        self._cal_active = False
        self._cal_btn.config(state="normal")
        self._push(f"[CAL] Full throttle captured: {full_val} µs — Done")
        messagebox.showinfo("RPM Setting Done",
                            "Throttle calibration complete!\nRPM output is now active.")

    def _reset_rpm(self):
        if not (self._port and self._port.is_open):
            messagebox.showwarning("Not Connected", "Connect to ECU first.")
            return
        if messagebox.askyesno(
                "Reset RPM Calibration",
                "Reset throttle calibration?\nRPM will show 0 until recalibrated."):
            self._send_cmd("CAL_RESET\n")
            self._push("[CAL] RPM calibration reset.")

    def _clear_log(self):
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        self._log.config(state="disabled")

    def _push(self, msg):
        self._log.config(state="normal")
        self._log.insert("end", msg + "\n")
        self._log.see("end")
        if int(self._log.index("end-1c").split(".")[0]) > 60:
            self._log.delete("1.0", "2.0")
        self._log.config(state="disabled")


if __name__ == "__main__":
    Dashboard().mainloop()
