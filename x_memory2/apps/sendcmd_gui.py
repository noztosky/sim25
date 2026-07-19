import os
import socket
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk
from datetime import datetime

class SendCmdGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SIL App - SendCmd Controller")
        self.root.geometry("550x820")
        self.root.configure(bg="#1e1e24")
        self.root.resizable(True, True)

        # Style Configuration
        self.style = ttk.Style()
        self.style.theme_use("clam")
        
        # Configure Colors
        self.bg_color = "#1e1e24"
        self.card_bg = "#2a2b36"
        self.fg_color = "#ffffff"
        self.accent_color = "#4a90e2" # Blue
        self.danger_color = "#e74c3c" # Red
        self.success_color = "#2ecc71" # Green
        self.warning_color = "#f1c40f" # Yellow
        self.entry_bg = "#3a3b45"
        self.log_bg = "#121214"

        # Notebook styles
        self.style.configure("TNotebook", background=self.bg_color, borderwidth=0)
        self.style.configure("TNotebook.Tab", background=self.card_bg, foreground=self.fg_color, borderwidth=0, padding=[12, 6])
        self.style.map("TNotebook.Tab", background=[("selected", self.accent_color)], foreground=[("selected", "white")])
        
        # Layout Frames
        self.create_widgets()

    def log(self, message, is_error=False):
        timestamp = datetime.now().strftime("%H:%M:%S")
        prefix = f"[{timestamp}] "
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, prefix)
        
        tag = "error" if is_error else "normal"
        self.log_text.insert(tk.END, f"{message}\n", tag)
        self.log_text.config(state=tk.DISABLED)
        self.log_text.see(tk.END)

    def send_command(self, cmd_str):
        ip = self.ip_entry.get().strip()
        port_str = self.port_entry.get().strip()
        
        if not ip or not port_str:
            self.log("Error: IP or Port is empty", is_error=True)
            return

        try:
            port = int(port_str)
        except ValueError:
            self.log(f"Error: Invalid Port '{port_str}'", is_error=True)
            return

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(cmd_str.encode('utf-8'), (ip, port))
            sock.close()
            self.log(f"Sent: \"{cmd_str}\" -> {ip}:{port}")
        except Exception as e:
            self.log(f"Failed to send: {str(e)}", is_error=True)

    def send_manual_command(self):
        axis = self.axis_var.get()
        value = self.value_entry.get().strip()
        duration = self.duration_entry.get().strip()
        
        if not value or not duration:
            self.log("Error: Value and Duration must be specified.", is_error=True)
            return
            
        try:
            float(value)
            float(duration)
        except ValueError:
            self.log("Error: Value and Duration must be numbers.", is_error=True)
            return

        cmd = f"{axis} {value} {duration}"
        self.send_command(cmd)

    def create_widgets(self):
        # Header / Title
        header_lbl = tk.Label(
            self.root, 
            text="EFT Z30 Drone Controller", 
            font=("Segoe UI", 16, "bold"), 
            bg=self.bg_color, 
            fg=self.fg_color
        )
        header_lbl.pack(pady=10)

        # 1. Connection Config Frame (Always visible)
        conn_frame = tk.LabelFrame(
            self.root, 
            text=" Connection Settings ", 
            font=("Segoe UI", 10, "bold"),
            bg=self.card_bg, 
            fg=self.fg_color, 
            padx=10, 
            pady=10,
            bd=0
        )
        conn_frame.pack(fill=tk.X, padx=15, pady=5)
        
        tk.Label(conn_frame, text="IP Address:", bg=self.card_bg, fg=self.fg_color).grid(row=0, column=0, sticky=tk.W, padx=5)
        self.ip_entry = tk.Entry(conn_frame, bg=self.entry_bg, fg=self.fg_color, insertbackground="white", bd=0, width=15)
        self.ip_entry.insert(0, "127.0.0.1")
        self.ip_entry.grid(row=0, column=1, padx=5, pady=2)

        tk.Label(conn_frame, text="Port:", bg=self.card_bg, fg=self.fg_color).grid(row=0, column=2, sticky=tk.W, padx=15)
        self.port_entry = tk.Entry(conn_frame, bg=self.entry_bg, fg=self.fg_color, insertbackground="white", bd=0, width=8)
        self.port_entry.insert(0, "5005")
        self.port_entry.grid(row=0, column=3, padx=5, pady=2)

        # Notebook for Tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)

        # Tab 1: Commands
        self.tab_control = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.tab_control, text=" Commands ")

        # Tab 2: PID Tuning
        self.tab_pid = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.tab_pid, text=" PID Tuning ")

        # Tab 3: Piloting (keyboard + mouse manual control)
        self.tab_pilot = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.tab_pilot, text=" Piloting ")

        # Tab 4: Auto-Tune (runs autotune.py, streams progress)
        self.tab_tune = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.tab_tune, text=" Auto-Tune ")

        # --- Tab 1: Commands Contents ---
        # 2. System Commands Frame (Quick Actions)
        sys_frame = tk.LabelFrame(
            self.tab_control, 
            text=" System Commands ", 
            font=("Segoe UI", 10, "bold"),
            bg=self.card_bg, 
            fg=self.fg_color, 
            padx=10, 
            pady=10,
            bd=0
        )
        sys_frame.pack(fill=tk.X, padx=10, pady=5)

        pid_btn = tk.Button(
            sys_frame, 
            text="🔄 Reload PID (pid)", 
            font=("Segoe UI", 10, "bold"),
            bg=self.success_color, 
            fg=self.bg_color, 
            activebackground="#27ae60",
            activeforeground=self.bg_color,
            bd=0, 
            height=2,
            command=lambda: self.send_command("pid")
        )
        pid_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        reset_btn = tk.Button(
            sys_frame, 
            text="🚨 Reset Drone (reset)", 
            font=("Segoe UI", 10, "bold"),
            bg=self.danger_color, 
            fg=self.fg_color, 
            activebackground="#c0392b",
            activeforeground=self.fg_color,
            bd=0, 
            height=2,
            command=lambda: self.send_command("reset")
        )
        reset_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # 3. Manual Command Builder Frame
        manual_frame = tk.LabelFrame(
            self.tab_control, 
            text=" Command Builder ", 
            font=("Segoe UI", 10, "bold"),
            bg=self.card_bg, 
            fg=self.fg_color, 
            padx=10, 
            pady=10,
            bd=0
        )
        manual_frame.pack(fill=tk.X, padx=10, pady=5)

        # Axis Selection
        tk.Label(manual_frame, text="Axis:", bg=self.card_bg, fg=self.fg_color, font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky=tk.W, pady=5)
        
        self.axis_var = tk.StringVar(value="a")
        axes = [("Altitude (a)", "a"), ("Roll (r)", "r"), ("Pitch (p)", "p"), ("Yaw (y)", "y")]
        
        axis_subframe = tk.Frame(manual_frame, bg=self.card_bg)
        axis_subframe.grid(row=0, column=1, columnspan=3, sticky=tk.W, padx=5)
        
        for text, mode in axes:
            r_btn = tk.Radiobutton(
                axis_subframe, 
                text=text, 
                variable=self.axis_var, 
                value=mode, 
                bg=self.card_bg, 
                fg=self.fg_color, 
                selectcolor=self.entry_bg,
                activebackground=self.card_bg,
                activeforeground=self.fg_color
            )
            r_btn.pack(side=tk.LEFT, padx=4)

        # Value & Duration
        tk.Label(manual_frame, text="Value (m/deg):", bg=self.card_bg, fg=self.fg_color).grid(row=1, column=0, sticky=tk.W, pady=5)
        self.value_entry = tk.Entry(manual_frame, bg=self.entry_bg, fg=self.fg_color, insertbackground="white", bd=0, width=12)
        self.value_entry.insert(0, "5.0")
        self.value_entry.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)

        tk.Label(manual_frame, text="Duration (sec):", bg=self.card_bg, fg=self.fg_color).grid(row=1, column=2, sticky=tk.W, padx=15, pady=5)
        self.duration_entry = tk.Entry(manual_frame, bg=self.entry_bg, fg=self.fg_color, insertbackground="white", bd=0, width=10)
        self.duration_entry.insert(0, "10.0")
        self.duration_entry.grid(row=1, column=3, sticky=tk.W, padx=5, pady=5)

        send_btn = tk.Button(
            manual_frame, 
            text="🚀 Send Manual Command", 
            font=("Segoe UI", 10, "bold"),
            bg=self.accent_color, 
            fg=self.fg_color, 
            activebackground="#357abd",
            activeforeground=self.fg_color,
            bd=0, 
            height=2,
            command=self.send_manual_command
        )
        send_btn.grid(row=2, column=0, columnspan=4, sticky="ew", pady=10)

        # 4. Preset Commands Frame
        preset_frame = tk.LabelFrame(
            self.tab_control, 
            text=" Preset Commands (Click to Send) ", 
            font=("Segoe UI", 10, "bold"),
            bg=self.card_bg, 
            fg=self.fg_color, 
            padx=10, 
            pady=10,
            bd=0
        )
        preset_frame.pack(fill=tk.X, padx=10, pady=5)

        presets = [
            ("🛫 Takeoff to 5m (10s)", "a 5 10"),
            ("🛫 Takeoff to 10m (15s)", "a 10 15"),
            ("🔄 Roll Right 30° (1s)", "r 30 1"),
            ("🔄 Roll Left -30° (1s)", "r -30 1"),
            ("📐 Pitch Forward 20° (1s)", "p 20 1"),
            ("📐 Pitch Backward -20° (1s)", "p -20 1"),
            ("🧭 Yaw CW 45° (2s)", "y 45 2"),
            ("🧭 Yaw CCW -45° (2s)", "y -45 2")
        ]

        # Use Grid layout for 2x4 Presets
        for index, (label, cmd) in enumerate(presets):
            row = index // 2
            col = index % 2
            btn = tk.Button(
                preset_frame, 
                text=label, 
                font=("Segoe UI", 9),
                bg="#34495e", 
                fg=self.fg_color, 
                activebackground="#2c3e50",
                activeforeground=self.fg_color,
                bd=0, 
                padx=5,
                pady=8,
                command=lambda c=cmd: self.send_command(c)
            )
            btn.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
            preset_frame.grid_columnconfigure(col, weight=1)

        # --- Tab 2: PID Tuning Setup ---
        self.setup_pid_tab()

        # --- Tab 3: Piloting Setup ---
        self.setup_pilot_tab()

        # --- Tab 4: Auto-Tune Setup ---
        self.setup_tune_tab()

        # 5. Log Console Frame (Always visible at bottom)
        log_frame = tk.LabelFrame(
            self.root, 
            text=" Transaction Log ", 
            font=("Segoe UI", 10, "bold"),
            bg=self.card_bg, 
            fg=self.fg_color, 
            padx=10, 
            pady=10,
            bd=0
        )
        log_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

        # Log Text Box
        self.log_text = tk.Text(
            log_frame, 
            bg=self.log_bg, 
            fg="#2ecc71", # Matrix Green
            font=("Consolas", 9), 
            bd=0, 
            state=tk.DISABLED
        )
        self.log_text.tag_config("normal", foreground="#2ecc71")
        self.log_text.tag_config("error", foreground="#e74c3c")
        self.log_text.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)

        # Scrollbar for Log
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)

        # Clear Log Button
        clear_btn = tk.Button(
            log_frame, 
            text="Clear", 
            font=("Segoe UI", 8),
            bg="#7f8c8d", 
            fg=self.fg_color, 
            activebackground="#95a5a6",
            bd=0,
            command=self.clear_logs
        )
        clear_btn.pack(side=tk.BOTTOM, fill=tk.X, pady=2)

        # Trigger initial load after all widgets (including log_text) are initialized
        self.load_pid_params()

    def setup_pid_tab(self):
        grid_frame = tk.Frame(self.tab_pid, bg=self.bg_color)
        grid_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        grid_frame.grid_columnconfigure(0, weight=1)
        grid_frame.grid_columnconfigure(1, weight=1)
        grid_frame.grid_rowconfigure(0, weight=1)
        grid_frame.grid_rowconfigure(1, weight=1)

        self.param_entries = {}
        def add_entry(parent, label_text, key, r):
            tk.Label(parent, text=label_text, bg=self.card_bg, fg=self.fg_color, font=("Segoe UI", 9), anchor="w").grid(row=r, column=0, sticky="w", padx=5, pady=1)
            entry = tk.Entry(parent, bg=self.entry_bg, fg=self.fg_color, insertbackground="white", bd=0, width=8, justify="right", font=("Consolas", 9))
            entry.grid(row=r, column=1, padx=5, pady=1, sticky="e")
            parent.grid_columnconfigure(0, weight=1)
            parent.grid_columnconfigure(1, weight=0)
            self.param_entries[key] = entry

        # 1. Altitude Frame
        alt_frame = tk.LabelFrame(grid_frame, text=" Altitude / Collective ", font=("Segoe UI", 9, "bold"), bg=self.card_bg, fg=self.fg_color, bd=0)
        alt_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        alt_keys = [
            ("Hover Throttle:", "hover_throttle"),
            ("Alt P:", "alt_p"),
            ("Alt I:", "alt_i"),
            ("Alt D:", "alt_d"),
            ("Alt I-Lim:", "alt_ilim"),
            ("Alt Min:", "alt_min"),
            ("Alt Max:", "alt_max"),
            ("Alt D-Filt:", "alt_dfilt")
        ]
        for idx, (lbl, key) in enumerate(alt_keys):
            add_entry(alt_frame, lbl, key, idx)

        # 2. Inner Rate Frame
        rate_frame = tk.LabelFrame(grid_frame, text=" Inner Rate PID ", font=("Segoe UI", 9, "bold"), bg=self.card_bg, fg=self.fg_color, bd=0)
        rate_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        rate_keys = [
            ("Rate P:", "rate_p"),
            ("Rate I:", "rate_i"),
            ("Rate D:", "rate_d"),
            ("Rate I-Lim:", "rate_ilim"),
            ("Rate Clamp:", "rate_clamp"),
            ("Rate D-Filt:", "rate_dfilt")
        ]
        for idx, (lbl, key) in enumerate(rate_keys):
            add_entry(rate_frame, lbl, key, idx)

        # 3. Outer Angle Frame
        angle_frame = tk.LabelFrame(grid_frame, text=" Outer Angle Gains ", font=("Segoe UI", 9, "bold"), bg=self.card_bg, fg=self.fg_color, bd=0)
        angle_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        angle_keys = [
            ("Roll Gain:", "again_roll"),
            ("Pitch Gain:", "again_pitch"),
            ("Yaw Gain:", "again_yaw")
        ]
        for idx, (lbl, key) in enumerate(angle_keys):
            add_entry(angle_frame, lbl, key, idx)

        # 4. Yaw Control Frame
        yaw_frame = tk.LabelFrame(grid_frame, text=" Yaw PID ", font=("Segoe UI", 9, "bold"), bg=self.card_bg, fg=self.fg_color, bd=0)
        yaw_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        yaw_keys = [
            ("Yaw P:", "yaw_p"),
            ("Yaw I:", "yaw_i"),
            ("Yaw D:", "yaw_d"),
            ("Yaw Clamp:", "yaw_clamp")
        ]
        for idx, (lbl, key) in enumerate(yaw_keys):
            add_entry(yaw_frame, lbl, key, idx)

        # Bottom Buttons
        btn_frame = tk.Frame(self.tab_pid, bg=self.bg_color)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)

        load_btn = tk.Button(
            btn_frame, 
            text="📂 Load Params", 
            font=("Segoe UI", 9, "bold"),
            bg="#34495e", 
            fg=self.fg_color, 
            activebackground="#2c3e50",
            activeforeground=self.fg_color,
            bd=0, 
            height=2,
            command=self.load_pid_params
        )
        load_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        save_btn = tk.Button(
            btn_frame, 
            text="💾 Save to File", 
            font=("Segoe UI", 9, "bold"),
            bg="#7f8c8d", 
            fg=self.fg_color, 
            activebackground="#95a5a6",
            activeforeground=self.fg_color,
            bd=0, 
            height=2,
            command=self.save_pid_params
        )
        save_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        apply_btn = tk.Button(
            btn_frame, 
            text="⚡ Save & Apply (pid)", 
            font=("Segoe UI", 9, "bold"),
            bg=self.success_color, 
            fg=self.bg_color, 
            activebackground="#27ae60",
            activeforeground=self.bg_color,
            bd=0, 
            height=2,
            command=self.save_and_apply_pid
        )
        apply_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=3)

    # ==================== Piloting (keyboard + mouse) ====================
    # Protocol note: SIL_App holds only ONE transient attitude axis at a time
    # (roll OR pitch OR yaw). Altitude ('a') is an independent persistent target.
    # So the control loop climbs/descends freely while commanding the single
    # dominant tilt/yaw axis each tick.
    PILOT_KEYS = {"w", "s", "a", "d", "q", "e", "r", "f", "t", "space", "z", "g",
                  "up", "down", "left", "right"}

    def _raw_send(self, cmd_str):
        """Send a UDP command WITHOUT logging (used by the 10 Hz pilot loop)."""
        ip = self.ip_entry.get().strip()
        try:
            port = int(self.port_entry.get().strip())
        except ValueError:
            return
        try:
            self._pilot_sock.sendto(cmd_str.encode("utf-8"), (ip, port))
        except Exception:
            pass

    def setup_pilot_tab(self):
        # pilot runtime state
        self._pilot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.piloting_active = False
        self.pressed = set()
        self.target_alt = 5.0
        self.js_roll = 0.0
        self.js_pitch = 0.0
        self.yaw_offset = 0.0   # accumulated heading offset (deg) from Q/E

        parent = self.tab_pilot

        # --- info / warning ---
        note = tk.Label(
            parent,
            text=("Manual flight. Click START, then keep this window focused.\n"
                  "Note: the auto-tuner (if running) owns SIL_App — pilot after it finishes."),
            bg=self.bg_color, fg=self.warning_color, font=("Segoe UI", 8), justify="left")
        note.pack(fill=tk.X, padx=10, pady=(6, 2))

        # --- START/STOP + status ---
        top = tk.Frame(parent, bg=self.bg_color)
        top.pack(fill=tk.X, padx=10, pady=4)
        self.pilot_toggle_btn = tk.Button(
            top, text="▶ START PILOTING", font=("Segoe UI", 11, "bold"),
            bg=self.success_color, fg=self.bg_color, activebackground="#27ae60",
            bd=0, height=2, command=self.toggle_piloting)
        self.pilot_toggle_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        tk.Button(top, text="🛫 Takeoff (T)", font=("Segoe UI", 9, "bold"),
                  bg=self.accent_color, fg=self.fg_color, bd=0, height=2,
                  command=self.pilot_takeoff).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        tk.Button(top, text="🚨 Reset (Z)", font=("Segoe UI", 9, "bold"),
                  bg=self.danger_color, fg=self.fg_color, bd=0, height=2,
                  command=lambda: self.send_command("reset")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        # --- flight-mode selector (default LOITER) ---
        mode_row = tk.Frame(parent, bg=self.bg_color)
        mode_row.pack(fill=tk.X, padx=10, pady=2)
        tk.Label(mode_row, text="Flight Mode:", bg=self.bg_color, fg=self.fg_color,
                 font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(2, 8))
        self.mode_var = tk.StringVar(value="LOITER")
        for name, desc in [("LOITER", "GPS position hold"), ("ALTHOLD", "manual attitude")]:
            tk.Radiobutton(mode_row, text=f"{name}", variable=self.mode_var, value=name,
                           bg=self.entry_bg, fg=self.fg_color, selectcolor=self.accent_color,
                           activebackground=self.entry_bg, activeforeground=self.fg_color,
                           indicatoron=False, width=10, bd=0, padx=6, pady=5,
                           font=("Segoe UI", 9, "bold"),
                           command=self.set_mode).pack(side=tk.LEFT, padx=3)

        self.pilot_status = tk.Label(
            parent, text="STOPPED", bg=self.log_bg, fg="#888",
            font=("Consolas", 10, "bold"), anchor="w", padx=8, pady=4)
        self.pilot_status.pack(fill=tk.X, padx=10, pady=2)

        body = tk.Frame(parent, bg=self.bg_color)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        # --- left: virtual joystick (roll/pitch) ---
        js_frame = tk.LabelFrame(body, text=" Mouse Stick (roll/pitch) ", font=("Segoe UI", 9, "bold"),
                                 bg=self.card_bg, fg=self.fg_color, bd=0)
        js_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)
        self._js_size = 170
        self._js_r = self._js_size / 2 - 12
        self.joy = tk.Canvas(js_frame, width=self._js_size, height=self._js_size,
                             bg=self.log_bg, highlightthickness=0)
        self.joy.pack(padx=8, pady=8)
        c = self._js_size / 2
        self.joy.create_oval(c - self._js_r, c - self._js_r, c + self._js_r, c + self._js_r,
                             outline="#3a3b45", width=2)
        self.joy.create_line(c, c - self._js_r, c, c + self._js_r, fill="#2a2b36")
        self.joy.create_line(c - self._js_r, c, c + self._js_r, c, fill="#2a2b36")
        self._knob = self.joy.create_oval(c - 12, c - 12, c + 12, c + 12,
                                          fill=self.accent_color, outline="")
        self.joy.bind("<Button-1>", self.on_joy)
        self.joy.bind("<B1-Motion>", self.on_joy)
        self.joy.bind("<ButtonRelease-1>", self.on_joy_release)

        # --- right: settings + key map ---
        right = tk.Frame(body, bg=self.bg_color)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        set_frame = tk.LabelFrame(right, text=" Settings ", font=("Segoe UI", 9, "bold"),
                                  bg=self.card_bg, fg=self.fg_color, bd=0)
        set_frame.pack(fill=tk.X, pady=2)
        self.tilt_var = tk.StringVar(value="12")
        self.yawstep_var = tk.StringVar(value="30")
        self.altrate_var = tk.StringVar(value="1.5")
        for i, (lbl, var) in enumerate([("Tilt angle (deg)", self.tilt_var),
                                        ("Yaw rate (deg/s)", self.yawstep_var),
                                        ("Alt rate (m/s)", self.altrate_var)]):
            tk.Label(set_frame, text=lbl, bg=self.card_bg, fg=self.fg_color,
                     font=("Segoe UI", 8)).grid(row=i, column=0, sticky="w", padx=5, pady=2)
            tk.Entry(set_frame, textvariable=var, bg=self.entry_bg, fg=self.fg_color,
                     insertbackground="white", bd=0, width=7, justify="right").grid(
                row=i, column=1, padx=5, pady=2, sticky="e")
        set_frame.grid_columnconfigure(0, weight=1)

        km = tk.LabelFrame(right, text=" Keys ", font=("Segoe UI", 9, "bold"),
                           bg=self.card_bg, fg=self.fg_color, bd=0)
        km.pack(fill=tk.BOTH, expand=True, pady=2)
        keymap = ("W/S or ↑/↓   fwd / back\n"
                  "A/D or ←/→   left / right\n"
                  "Q/E   yaw left / right\n"
                  "R/F   altitude up / down\n"
                  "T  takeoff      G  land\n"
                  "Space  level    Z  reset\n"
                  "(Loiter: stick = move & hold)")
        tk.Label(km, text=keymap, bg=self.card_bg, fg="#bbb", font=("Consolas", 9),
                 justify="left").pack(anchor="w", padx=8, pady=6)

        # global key bindings (guarded by piloting_active + focus check)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

    def toggle_piloting(self):
        self.piloting_active = not self.piloting_active
        if self.piloting_active:
            self.pressed.clear()
            self.pilot_toggle_btn.config(text="■ STOP PILOTING", bg=self.danger_color)
            self.log("Piloting ENABLED — WASD/QE/RF, keep window focused.")
            self.pilot_tick()
        else:
            self.pilot_toggle_btn.config(text="▶ START PILOTING", bg=self.success_color)
            self.pressed.clear()
            self.js_roll = self.js_pitch = 0.0
            self._raw_send(f"c 0 0 {self.yaw_offset:.1f} 0.3")  # level on stop
            self.pilot_status.config(text="STOPPED", fg="#888")
            self.log("Piloting DISABLED.")

    def pilot_takeoff(self):
        self.yaw_offset = 0.0   # heading reference resets at takeoff
        self.send_command(f"a {self.target_alt:.1f} 10")

    def set_mode(self):
        m = self.mode_var.get()
        self.send_command("mode loiter" if m == "LOITER" else "mode althold")

    def _flt(self, var, default):
        try:
            return float(var.get())
        except Exception:
            return default

    def on_key_press(self, event):
        if not self.piloting_active:
            return
        if isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry)):
            return
        k = event.keysym.lower()
        if k not in self.PILOT_KEYS:
            return
        if k not in self.pressed:          # fire one-shots once per physical press
            if k == "t":
                self.pilot_takeoff()
            elif k == "space":
                self._raw_send(f"c 0 0 {self.yaw_offset:.1f} 0.3")
            elif k == "z":
                self.yaw_offset = 0.0
                self.send_command("reset")
        self.pressed.add(k)
        return "break"

    def on_key_release(self, event):
        if not self.piloting_active:
            return
        self.pressed.discard(event.keysym.lower())

    def on_joy(self, event):
        c = self._js_size / 2
        dx = event.x - c
        dy = event.y - c
        import math
        dist = math.hypot(dx, dy)
        if dist > self._js_r:
            dx *= self._js_r / dist
            dy *= self._js_r / dist
        self.joy.coords(self._knob, c + dx - 12, c + dy - 12, c + dx + 12, c + dy + 12)
        tilt = self._flt(self.tilt_var, 12.0)
        self.js_roll = dx / self._js_r * tilt
        self.js_pitch = dy / self._js_r * tilt   # stick up (dy<0) = forward = nose-down = -pitch

    def on_joy_release(self, event):
        c = self._js_size / 2
        self.joy.coords(self._knob, c - 12, c - 12, c + 12, c + 12)
        self.js_roll = self.js_pitch = 0.0
        # next pilot_tick sends the combined level command automatically

    def pilot_tick(self):
        if not self.piloting_active:
            return
        tilt = self._flt(self.tilt_var, 12.0)
        yaw_rate = self._flt(self.yawstep_var, 30.0)   # deg/s
        alt_rate = self._flt(self.altrate_var, 1.5)
        tick_ms = 100
        dt = tick_ms / 1000.0
        dur = 0.30

        p = self.pressed
        fwd = ("w" in p) or ("up" in p)
        back = ("s" in p) or ("down" in p)
        right = ("d" in p) or ("right" in p)
        left = ("a" in p) or ("left" in p)
        # forward = nose DOWN = negative pitch on this airframe (+pitch is nose-up = backward)
        pitch = (-tilt if fwd else 0.0) + (tilt if back else 0.0) + self.js_pitch
        roll = (tilt if right else 0.0) - (tilt if left else 0.0) + self.js_roll
        roll = max(-tilt, min(tilt, roll))
        pitch = max(-tilt, min(tilt, pitch))

        # yaw: rate control -> accumulate a heading offset while Q/E held
        if "e" in p:
            self.yaw_offset += yaw_rate * dt
        if "q" in p:
            self.yaw_offset -= yaw_rate * dt

        # altitude: independent persistent target
        d_alt = alt_rate * dt
        alt_changed = False
        if "r" in self.pressed:
            self.target_alt += d_alt
            alt_changed = True
        if "f" in self.pressed:
            self.target_alt = max(0.0, self.target_alt - d_alt)
            alt_changed = True
        if "g" in self.pressed:
            self.target_alt = 0.0
            alt_changed = True
        if alt_changed:
            self._raw_send(f"a {self.target_alt:.2f} 10")

        # combined multi-axis setpoint (roll+pitch+yaw held together)
        self._raw_send(f"c {roll:.1f} {pitch:.1f} {self.yaw_offset:.1f} {dur}")

        self.pilot_status.config(
            text=(f"FLYING | alt={self.target_alt:5.2f}m  roll={roll:+5.1f}  "
                  f"pitch={pitch:+5.1f}  yaw={self.yaw_offset:+6.1f}"),
            fg=self.success_color)
        self.root.after(tick_ms, self.pilot_tick)

    # ==================== Auto-Tune (runs autotune.py) ====================
    def setup_tune_tab(self):
        self._tune_proc = None
        parent = self.tab_tune

        note = tk.Label(
            parent,
            text=("Runs apps/autotune.py: commands attitude steps and searches PID gains.\n"
                  "The simulator (sblocks.bat 1000) must be running. Auto-Tune OWNS SIL_App\n"
                  "(restarts it each trial) — don't pilot at the same time. ~15-25 min."),
            bg=self.bg_color, fg=self.warning_color, font=("Segoe UI", 8), justify="left")
        note.pack(fill=tk.X, padx=10, pady=(6, 2))

        cfg = tk.LabelFrame(parent, text=" Settings ", font=("Segoe UI", 9, "bold"),
                            bg=self.card_bg, fg=self.fg_color, bd=0)
        cfg.pack(fill=tk.X, padx=10, pady=4)
        tk.Label(cfg, text="Phase:", bg=self.card_bg, fg=self.fg_color,
                 font=("Segoe UI", 9)).grid(row=0, column=0, sticky="w", padx=5, pady=4)
        self.tune_phase_var = tk.StringVar(value="attitude")
        ttk.Combobox(cfg, textvariable=self.tune_phase_var, values=["attitude", "altitude"],
                     width=10, state="readonly").grid(row=0, column=1, padx=5, pady=4)
        tk.Label(cfg, text="Evals:", bg=self.card_bg, fg=self.fg_color,
                 font=("Segoe UI", 9)).grid(row=0, column=2, sticky="w", padx=15, pady=4)
        self.tune_evals_var = tk.StringVar(value="35")
        tk.Entry(cfg, textvariable=self.tune_evals_var, bg=self.entry_bg, fg=self.fg_color,
                 insertbackground="white", bd=0, width=6, justify="right").grid(row=0, column=3, padx=5, pady=4)

        btns = tk.Frame(parent, bg=self.bg_color)
        btns.pack(fill=tk.X, padx=10, pady=2)
        self.tune_start_btn = tk.Button(btns, text="▶ Start Auto-Tune", font=("Segoe UI", 10, "bold"),
                                        bg=self.success_color, fg=self.bg_color, bd=0, height=2,
                                        command=self.start_autotune)
        self.tune_start_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        tk.Button(btns, text="■ Stop", font=("Segoe UI", 10, "bold"),
                  bg=self.danger_color, fg=self.fg_color, bd=0, height=2,
                  command=self.stop_autotune).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        self.tune_status = tk.Label(parent, text="idle", bg=self.log_bg, fg="#888",
                                    font=("Consolas", 10, "bold"), anchor="w", padx=8, pady=4)
        self.tune_status.pack(fill=tk.X, padx=10, pady=2)

        out_frame = tk.Frame(parent, bg=self.bg_color)
        out_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
        self.tune_out = tk.Text(out_frame, bg=self.log_bg, fg="#7fd0ff", font=("Consolas", 8),
                                bd=0, state=tk.DISABLED, height=10)
        self.tune_out.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        sb = ttk.Scrollbar(out_frame, command=self.tune_out.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tune_out.config(yscrollcommand=sb.set)

    def _tune_write(self, text):
        self.tune_out.config(state=tk.NORMAL)
        self.tune_out.insert(tk.END, text)
        self.tune_out.see(tk.END)
        self.tune_out.config(state=tk.DISABLED)

    def start_autotune(self):
        if self._tune_proc is not None and self._tune_proc.poll() is None:
            self.log("Auto-Tune already running.", is_error=True)
            return
        try:
            evals = int(self.tune_evals_var.get())
        except ValueError:
            self.log("Evals must be an integer.", is_error=True)
            return
        phase = self.tune_phase_var.get()
        autotune_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), "autotune.py")
        if not os.path.exists(autotune_py):
            self.log(f"autotune.py not found at {autotune_py}", is_error=True)
            return
        cmd = [sys.executable, "-u", autotune_py, "--phase", phase, "--evals", str(evals)]
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        flags = 0x08000000 if os.name == "nt" else 0  # CREATE_NO_WINDOW
        try:
            self._tune_proc = subprocess.Popen(
                cmd, cwd=os.path.dirname(autotune_py), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, env=env, creationflags=flags)
        except Exception as e:
            self.log(f"Failed to start Auto-Tune: {e}", is_error=True)
            return
        self.tune_out.config(state=tk.NORMAL)
        self.tune_out.delete(1.0, tk.END)
        self.tune_out.config(state=tk.DISABLED)
        self.tune_status.config(text=f"running ({phase}, {evals} evals)...", fg=self.warning_color)
        self.tune_start_btn.config(state=tk.DISABLED)
        self.log(f"Auto-Tune started: phase={phase}, evals={evals}")
        threading.Thread(target=self._autotune_reader, daemon=True).start()

    def _autotune_reader(self):
        try:
            for line in self._tune_proc.stdout:
                self.root.after(0, self._on_tune_line, line.rstrip("\n"))
        except Exception:
            pass
        self.root.after(0, self._on_tune_done)

    def _on_tune_line(self, line):
        self._tune_write(line + "\n")
        # live status from "[NN] ... (best=...)" lines and the BEST summary
        if line.startswith("[") and "best=" in line:
            head = line.split("|")[0].strip()
            best = line.split("best=")[-1].rstrip(")")
            self.tune_status.config(text=f"{head}  best={best}", fg=self.warning_color)

    def _on_tune_done(self):
        rc = self._tune_proc.poll() if self._tune_proc else None
        self.tune_start_btn.config(state=tk.NORMAL)
        if rc == 0:
            self.tune_status.config(text="DONE — best gains written. Reloading PID tab...", fg=self.success_color)
            self.log("Auto-Tune finished. Best gains saved to pid_params_z30.txt.")
            self.load_pid_params()  # refresh the PID Tuning tab with the new gains
        else:
            self.tune_status.config(text=f"stopped/failed (rc={rc})", fg=self.danger_color)

    def stop_autotune(self):
        if self._tune_proc is not None and self._tune_proc.poll() is None:
            try:
                self._tune_proc.terminate()
            except Exception:
                pass
            # autotune spawns SIL_App per trial; make sure it's not left running
            subprocess.run(["taskkill", "/F", "/IM", "SIL_App.exe"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.tune_status.config(text="stopped", fg=self.danger_color)
            self.tune_start_btn.config(state=tk.NORMAL)
            self.log("Auto-Tune stopped.")
        else:
            self.log("Auto-Tune is not running.")

    def load_pid_params(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        possible_paths = [
            os.path.join(script_dir, "../build/pid_params_z30.txt"),
            os.path.join(script_dir, "build/pid_params_z30.txt"),
            os.path.join(os.getcwd(), "build/pid_params_z30.txt"),
            os.path.join(os.getcwd(), "pid_params_z30.txt"),
            "build/pid_params_z30.txt",
            "pid_params_z30.txt"
        ]
        self.param_file_path = None
        for path in possible_paths:
            full_path = os.path.abspath(path)
            if os.path.exists(full_path):
                self.param_file_path = full_path
                break
                
        if not self.param_file_path:
            self.param_file_path = os.path.abspath("build/pid_params_z30.txt")
            self.log(f"Warning: Parameter file not found. Defaulting to: {self.param_file_path}")
            return
            
        self.file_lines = []
        if os.path.exists(self.param_file_path):
            try:
                with open(self.param_file_path, "r", encoding="utf-8") as f:
                    self.file_lines = f.readlines()
            except Exception as e:
                self.log(f"Failed to read param file: {str(e)}", is_error=True)
                return

        self.params_dict = {}
        for line in self.file_lines:
            clean = line.split('#')[0].strip()
            if '=' in clean:
                key, val = clean.split('=', 1)
                self.params_dict[key.strip()] = val.strip()

        for key, entry in self.param_entries.items():
            if key in self.params_dict:
                entry.delete(0, tk.END)
                entry.insert(0, self.params_dict[key])
        self.log(f"Loaded PID params from {os.path.basename(self.param_file_path)}")

    def save_pid_params(self):
        if not self.param_file_path:
            self.log("Error: Parameter file path is not resolved.", is_error=True)
            return False
            
        new_values = {}
        for key, entry in self.param_entries.items():
            new_values[key] = entry.get().strip()
            
        updated_lines = []
        written_keys = set()
        
        for line in self.file_lines:
            comment = ""
            if '#' in line:
                parts = line.split('#', 1)
                code_part = parts[0]
                comment = "#" + parts[1]
            else:
                code_part = line
                
            if '=' in code_part:
                key, val = code_part.split('=', 1)
                key_strip = key.strip()
                if key_strip in new_values:
                    # Align key and value with comment if present
                    if comment:
                        new_line = f"{key_strip.ljust(15)} = {new_values[key_strip].ljust(8)} {comment}"
                    else:
                        new_line = f"{key_strip} = {new_values[key_strip]}\n"
                    
                    if not new_line.endswith('\n'):
                        new_line += '\n'
                    updated_lines.append(new_line)
                    written_keys.add(key_strip)
                    continue
            updated_lines.append(line)
            
        for key, val in new_values.items():
            if key not in written_keys:
                updated_lines.append(f"{key} = {val}\n")
                
        try:
            os.makedirs(os.path.dirname(self.param_file_path), exist_ok=True)
            with open(self.param_file_path, "w", encoding="utf-8") as f:
                f.writelines(updated_lines)
            self.file_lines = updated_lines
            self.log(f"Saved PID params to {os.path.basename(self.param_file_path)}")
            return True
        except Exception as e:
            self.log(f"Failed to save param file: {str(e)}", is_error=True)
            return False

    def save_and_apply_pid(self):
        if self.save_pid_params():
            self.send_command("pid")

    def clear_logs(self):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = SendCmdGUI(root)
    app.log("SendCmd GUI Initialized. Ready to send commands.")
    root.mainloop()
