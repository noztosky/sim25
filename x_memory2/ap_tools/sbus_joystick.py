#!/usr/bin/env python3
"""SBUS virtual joystick for ArduPilot SITL — sends real SBUS frames over TCP.

Default endpoint 127.0.0.1:5765 = SITL --serial5 tcp:5:nowait with
SERIAL5_PROTOCOL=23 (RCIN). AP's RC layer auto-detects the SBUS framing,
so this behaves exactly like a physical RC receiver (LOITER stick brake,
rudder arming etc. work like the real vehicle).

Channel map (matches the real K20 RCMAP/FLTMODE):
  ch1 roll   ch2 pitch   ch3 throttle   ch4 yaw   ch5 flight mode
Controls:
  Right pad / WASD(or arrows) : pitch/roll  (spring to center)
  Q / E                       : yaw left/right (spring)
  R / F                       : climb / descend (spring back to mid 1500)
  Space                       : center attitude sticks
  Mode buttons                : ch5 AltHold / Loiter (jogs the switch if the
                                channel is already at the target so AP always
                                sees a position CHANGE and actually switches)
  ARM / DISARM                : automated rudder gesture (throttle low + yaw 3.5 s)
  RESET SIM                   : disarm + AirSim reset + SITL bounce
                                (also fires AUTOMATICALLY on flip/tumble:
                                 |roll|>90 or |pitch|>90 or |gyro|>8 rad/s)
Usage: python sbus_joystick.py [host:port]
"""
import socket, subprocess, sys, threading, time
import tkinter as tk

HOST, PORT = "127.0.0.1", 5765
if len(sys.argv) > 1 and ":" in sys.argv[1]:
    HOST, PORT = sys.argv[1].rsplit(":", 1)[0], int(sys.argv[1].rsplit(":", 1)[1])

#SBUS->PWM in ArduPilot: pwm = 0.625*sbus + 880, so 192->1000us, 992->1500us,
#1792->2000us (stock 172/1811 would give 988/2012 - outside the 1000-2000 range)
SBUS_MIN, SBUS_MID, SBUS_MAX = 192, 992, 1792
RATE_HZ = 10
KEY_STEP = 1.0             # full deflection while held
THR_CLIMB = 0.90           # ch3 while R held (mid 0.5 = hold altitude)
THR_DESCEND = 0.10         # ch3 while F held
MAVLINK_PORT = 5762        # tumble monitor + disarm
AIRSIM_RPC = ("127.0.0.1", 41451)

def to_sbus(v):            # v in -1..1 -> sbus 11-bit
    return max(SBUS_MIN, min(SBUS_MAX, int(SBUS_MID + v * (SBUS_MAX - SBUS_MIN) / 2)))

def thr_sbus(v):           # v in 0..1
    return max(SBUS_MIN, min(SBUS_MAX, int(SBUS_MIN + v * (SBUS_MAX - SBUS_MIN))))

MODE_ALTHOLD = to_sbus(-0.8)
MODE_LOITER = to_sbus(0.8)

def build_frame(ch):       # ch: list of 16 sbus values
    bits = 0
    for i, v in enumerate(ch):
        bits |= (v & 0x7FF) << (11 * i)
    data = bits.to_bytes(22, "little")
    return b"\x0f" + data + b"\x00\x00"   # flags=0, footer=0

def airsim_reset():
    """Minimal msgpack-rpc call: reset() on the AirSim RPC server."""
    import msgpack
    s = socket.create_connection(AIRSIM_RPC, timeout=4)
    try:
        s.sendall(msgpack.packb([0, 1, "reset", []], use_bin_type=True))
        s.settimeout(4)
        try: s.recv(4096)
        except Exception: pass
    finally:
        s.close()

class App:
    def __init__(self, root):
        self.root = root
        root.title("SBUS Joystick — %s:%d" % (HOST, PORT))
        self.sock = None
        self.lock = threading.Lock()
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0; self.thr = 0.5
        self.mode_sbus = MODE_ALTHOLD     # start at AltHold so Loiter press = a CHANGE
        self.mode_jog = None              # (target_sbus, t_apply) two-step switch jog
        self.gesture = None               # (yaw_value, end_time) for arm/disarm
        self.pressed = set()              # true key-down state (autorepeat-safe)
        self.release_jobs = {}            # keysym -> pending after() id
        self.resetting = False
        self.last_reset = 0.0

        top = tk.Frame(root); top.pack(fill="x", padx=6, pady=4)
        self.addr = tk.Entry(top, width=18); self.addr.insert(0, "%s:%d" % (HOST, PORT))
        self.addr.pack(side="left")
        self.btn = tk.Button(top, text="Connect", command=self.toggle); self.btn.pack(side="left", padx=4)
        self.status = tk.Label(top, text="disconnected", fg="red"); self.status.pack(side="left", padx=6)

        mid = tk.Frame(root); mid.pack(padx=6, pady=4)
        self.pad = tk.Canvas(mid, width=220, height=220, bg="#222", highlightthickness=1)
        self.pad.pack(side="left", padx=4)
        self.pad.bind("<B1-Motion>", self.on_pad)
        self.pad.bind("<ButtonRelease-1>", lambda e: self.set_att(0, 0))
        right = tk.Frame(mid); right.pack(side="left", padx=8, fill="y")
        self.thr_bar = tk.Scale(right, from_=100, to=0, orient="vertical", length=180,
                                label="Alt", state="disabled")
        self.thr_bar.pack()

        modes = tk.Frame(root); modes.pack(pady=2)
        tk.Button(modes, text="AltHold", command=lambda: self.set_mode(MODE_ALTHOLD)).pack(side="left", padx=3)
        tk.Button(modes, text="Loiter",  command=lambda: self.set_mode(MODE_LOITER)).pack(side="left", padx=3)
        tk.Button(modes, text="ARM (rudder)",    command=lambda: self.rudder(1.0)).pack(side="left", padx=12)
        tk.Button(modes, text="DISARM (rudder)", command=lambda: self.rudder(-1.0)).pack(side="left", padx=3)
        tk.Button(modes, text="RESET SIM", fg="red", command=self.reset_sim).pack(side="left", padx=12)

        self.info = tk.Label(root, font=("Consolas", 10), justify="left",
                             text="WASD/arrows=pitch·roll  QE=yaw  R/F=climb/descend(→1500)  Space=center")
        self.info.pack(pady=3)
        self.guard_lbl = tk.Label(root, font=("Consolas", 9), fg="#888",
                                  text="tumble-guard: connecting...")
        self.guard_lbl.pack(pady=1)

        root.bind("<KeyPress>", self.on_key)
        root.bind("<KeyRelease>", self.on_key_up)
        self.draw()
        threading.Thread(target=self.sender, daemon=True).start()
        threading.Thread(target=self.tumble_guard, daemon=True).start()
        self.ui_tick()

    # ---------- connection ----------
    def toggle(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
            self.sock = None
            self.btn.config(text="Connect"); self.status.config(text="disconnected", fg="red")
            return
        host, port = self.addr.get().rsplit(":", 1)
        try:
            s = socket.create_connection((host, int(port)), timeout=3)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = s
            self.btn.config(text="Disconnect"); self.status.config(text="SBUS %dHz OK" % RATE_HZ, fg="green")
        except Exception as e:
            self.status.config(text="fail: %s" % e, fg="red")

    # ---------- inputs ----------
    def set_att(self, roll, pitch):
        with self.lock:
            self.roll, self.pitch = roll, pitch

    def set_mode(self, target):
        with self.lock:
            if self.mode_sbus == target:
                # jog: flip to the other position briefly so AP sees a change
                other = MODE_LOITER if target == MODE_ALTHOLD else MODE_ALTHOLD
                self.mode_sbus = other
                self.mode_jog = (target, time.time() + 0.4)
            else:
                self.mode_sbus = target
                self.mode_jog = None

    def rudder(self, direction):
        # rudder arm/disarm gesture: throttle min + full yaw for 3.5 s
        with self.lock:
            self.gesture = (direction, time.time() + 3.5)

    def on_pad(self, e):
        x = (e.x - 110) / 100.0; y = (e.y - 110) / 100.0
        self.set_att(max(-1, min(1, x)), max(-1, min(1, y)))

    def on_key(self, e):
        k = e.keysym.lower()
        job = self.release_jobs.pop(k, None)
        if job is not None:
            self.root.after_cancel(job)
        self.pressed.add(k)
        if e.keysym == "space":
            self.set_att(0, 0)
            with self.lock: self.yaw = 0.0

    def on_key_up(self, e):
        k = e.keysym.lower()
        job = self.release_jobs.pop(k, None)
        if job is not None:
            self.root.after_cancel(job)
        # delay the actual release slightly so autorepeat Release/Press pairs
        # don't cause a blink to center
        self.release_jobs[k] = self.root.after(60, lambda k=k: self._do_release(k))

    def _do_release(self, k):
        self.release_jobs.pop(k, None)
        self.pressed.discard(k)

    # ---------- periodic ----------
    def held(self, *names):
        return any(n in self.pressed for n in names)

    def ui_tick(self):
        r = (1.0 if self.held("d", "right") else 0) - (1.0 if self.held("a", "left") else 0)
        p = (1.0 if self.held("s", "down") else 0) - (1.0 if self.held("w", "up") else 0)
        y = (1.0 if self.held("e") else 0) - (1.0 if self.held("q") else 0)
        with self.lock:
            self.roll = r * KEY_STEP; self.pitch = p * KEY_STEP
            self.yaw = y * KEY_STEP
            # altitude key: spring back to mid 1500 (= hold altitude)
            if self.held("r"):   self.thr = THR_CLIMB
            elif self.held("f"): self.thr = THR_DESCEND
            else:                self.thr = 0.5
            # complete a pending mode-switch jog
            if self.mode_jog and time.time() >= self.mode_jog[1]:
                self.mode_sbus = self.mode_jog[0]
                self.mode_jog = None
        self.thr_bar.config(state="normal"); self.thr_bar.set(int(self.thr * 100)); self.thr_bar.config(state="disabled")
        self.draw()
        self.root.after(30, self.ui_tick)

    def draw(self):
        c = self.pad; c.delete("all")
        c.create_line(110, 0, 110, 220, fill="#444"); c.create_line(0, 110, 220, 110, fill="#444")
        x = 110 + self.roll * 100; ypx = 110 + self.pitch * 100
        c.create_oval(x - 8, ypx - 8, x + 8, ypx + 8, fill="#4af", outline="")
        c.create_text(110, 208, text="roll %+0.2f  pitch %+0.2f  yaw %+0.2f" %
                      (self.roll, self.pitch, self.yaw), fill="#aaa", font=("Consolas", 8))

    # ---------- sender ----------
    def sender(self):
        period = 1.0 / RATE_HZ
        while True:
            t0 = time.time()
            s = self.sock
            if s:
                with self.lock:
                    roll, pitch, yaw, thr = self.roll, self.pitch, self.yaw, self.thr
                    mode = self.mode_sbus
                    g = self.gesture
                    if g and time.time() > g[1]:
                        self.gesture = None; g = None
                if g:  # arm/disarm gesture overrides
                    yaw = g[0]; thr = 0.0
                ch = [SBUS_MID] * 16
                ch[0] = to_sbus(roll)
                ch[1] = to_sbus(pitch)
                ch[2] = thr_sbus(thr)
                ch[3] = to_sbus(yaw)
                ch[4] = mode
                try:
                    s.sendall(build_frame(ch))
                except Exception:
                    try: s.close()
                    except Exception: pass
                    self.sock = None
                    self.root.after(0, lambda: (self.btn.config(text="Connect"),
                                                self.status.config(text="link lost", fg="red")))
            time.sleep(max(0, period - (time.time() - t0)))

    # ---------- flip/tumble auto-reset ----------
    def tumble_guard(self):
        try:
            from pymavlink import mavutil
        except ImportError:
            self._guard_text("tumble-guard: pymavlink missing")
            return
        import math
        while True:
            try:
                m = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % MAVLINK_PORT)
                m.wait_heartbeat(timeout=30)
                m.mav.request_data_stream_send(m.target_system, m.target_component,
                                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                self.mav = m
                self._guard_text("tumble-guard: armed (>90deg or >8rad/s -> reset)")
                while True:
                    msg = m.recv_match(type="ATTITUDE", blocking=True, timeout=5)
                    if msg is None:
                        raise ConnectionError("silent")
                    r = abs(math.degrees(msg.roll)); p = abs(math.degrees(msg.pitch))
                    gy = max(abs(msg.rollspeed), abs(msg.pitchspeed), abs(msg.yawspeed))
                    if (r > 90 or p > 90 or gy > 8) and not self.resetting \
                       and time.time() - self.last_reset > 30:
                        self._guard_text("TUMBLE r=%.0f p=%.0f gyro=%.1f -> RESET" % (r, p, gy))
                        self.reset_sim()
            except Exception as e:
                self.mav = None
                self._guard_text("tumble-guard: reconnecting (%s)" % e)
                time.sleep(5)

    def _guard_text(self, s):
        try: self.root.after(0, lambda: self.guard_lbl.config(text=s))
        except Exception: pass

    def reset_sim(self):
        if self.resetting:
            return
        self.resetting = True
        self.last_reset = time.time()
        threading.Thread(target=self._do_reset, daemon=True).start()

    def _do_reset(self):
        self._guard_text("RESET: disarm + AirSim reset + SITL bounce...")
        # 1) force disarm (magic 21196)
        try:
            m = getattr(self, "mav", None)
            if m:
                from pymavlink import mavutil
                for _ in range(3):
                    m.mav.command_long_send(m.target_system, m.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                            0, 0, 21196, 0, 0, 0, 0, 0)
                    time.sleep(0.2)
        except Exception:
            pass
        # 2) AirSim vehicle reset (back to spawn, upright)
        try:
            airsim_reset()
        except Exception as e:
            self._guard_text("AirSim reset failed: %s" % e)
        # 3) SITL bounce (watchdog revives it; relink ~80-110 s)
        try:
            subprocess.run(["wsl", "-e", "bash", "-c", "pkill -9 -x arducopter"],
                           timeout=15, capture_output=True)
        except Exception:
            pass
        self._guard_text("RESET done - SITL rebooting, flyable again in ~90 s")
        self.resetting = False

root = tk.Tk()
App(root)
root.mainloop()
