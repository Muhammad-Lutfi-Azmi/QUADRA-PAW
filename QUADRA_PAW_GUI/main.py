import math
import os
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BAUD_DEFAULT = 115200
READ_INTERVAL_MS = 55
ANIMATION_INTERVAL_MS = 55

DEFAULTS = {
    "SPEED": 220,
    "LIFTF": 650,
    "LIFTB": 900,
    "S5A": 1200,
    "S5B": 1800,
    "S6MAJU": 1450,
    "S6MUNDUR": 1500,
    "S6KANAN": 1330,
    "S6KIRI": 1100,
}

PARAM_META = [
    ("SPEED", "Speed / Durasi", 80, 350, ""),
    ("LIFTF", "Angkat Kaki Depan", 400, 1000, ""),
    ("LIFTB", "Angkat Kaki Belakang", 400, 1050, ""),
    ("S5A", "S5 A", 1000, 1500, ""),
    ("S5B", "S5 B", 1500, 2000, ""),
    ("S6MAJU", "S6 Lurus Maju", 1000, 2000, ""),
    ("S6MUNDUR", "S6 Lurus Mundur", 1000, 2000, ""),
    ("S6KANAN", "S6 Belok Kanan", 900, 2100, ""),
    ("S6KIRI", "S6 Belok Kiri", 900, 2100, ""),
]

COLOR_BG = "#E9EEF4"
COLOR_CARD = "#FFFFFF"
COLOR_NAVY = "#061B33"
COLOR_NAVY_2 = "#0D2E4F"
COLOR_BLUE = "#1E6D93"
COLOR_CYAN = "#0EA5B7"
COLOR_CYAN_DARK = "#087A8B"
COLOR_YELLOW = "#F2C300"
COLOR_RED = "#D64545"
COLOR_GREEN = "#2EA44F"
COLOR_ORANGE = "#F39C12"
COLOR_GRAY_TEXT = "#53616E"
COLOR_SOFT = "#F8FBFE"
COLOR_DARK_LOG = "#06111F"
COLOR_HEADER = "#FFFFFF"
COLOR_HEADER_2 = "#EAF1F7"
COLOR_BORDER = "#C9D7E5"


def asset_path(name):
    base = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, "assets", name)


def load_scaled_photo(path, max_width, max_height):
    if not os.path.exists(path):
        return None
    try:
        img = tk.PhotoImage(file=path)
        scale = max(1, math.ceil(max(img.width() / max_width, img.height() / max_height)))
        if scale > 1:
            img = img.subsample(scale, scale)
        return img
    except Exception:
        return None


class SerialWorker:
    def __init__(self, incoming_queue):
        self.incoming_queue = incoming_queue
        self.ser = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()

    def connect(self, port, baudrate):
        if serial is None:
            raise RuntimeError("PySerial belum terpasang. Jalankan: py -m pip install pyserial")
        self.disconnect()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, write_timeout=0.5)
        time.sleep(2.2)
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send_line(self, text):
        if not self.is_connected():
            raise RuntimeError("ESP32 belum terhubung.")
        line = text.strip() + "\n"
        with self.lock:
            self.ser.write(line.encode("utf-8", errors="ignore"))
            self.ser.flush()

    def _read_loop(self):
        buffer = b""
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(256)
                if data:
                    buffer += data
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        line = raw.decode("utf-8", errors="replace").strip()
                        if line:
                            self.incoming_queue.put(line)
            except Exception as exc:
                self.incoming_queue.put(f"[ERROR SERIAL] {exc}")
                break


class RobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("QUADRA-PAW Control Center REV10 Label Fix | TRIK UGM")
        self.geometry("1520x880")
        self.minsize(1260, 760)
        self.configure(bg=COLOR_BG)

        self._apply_style()

        self.incoming = queue.Queue()
        self.serial_worker = SerialWorker(self.incoming)

        self.connection_var = tk.StringVar(value="Disconnected")
        self.last_ack_var = tk.StringVar(value="-")
        self.mode_var = tk.StringVar(value="-")
        self.phase_var = tk.StringVar(value="-")
        self.mpu_var = tk.StringVar(value="-")
        self.roll_var = tk.StringVar(value="0.00")
        self.pitch_var = tk.StringVar(value="0.00")
        self.az_var = tk.StringVar(value="0.00")
        self.s5_var = tk.StringVar(value="-")
        self.s6_var = tk.StringVar(value="-")

        self.param_vars = {name: tk.IntVar(value=DEFAULTS[name]) for name, *_ in PARAM_META}
        self.param_entry_vars = {name: tk.StringVar(value=str(DEFAULTS[name])) for name, *_ in PARAM_META}
        self.auto_send_var = tk.BooleanVar(value=True)
        self.sync_on_connect_var = tk.BooleanVar(value=True)
        self.apply_esp_params_until = 0.0
        self.param_widgets = {}

        self.s5_now_us = 1500
        self.s5_target_us = 1500
        self.s6_now_us = 1500
        self.s6_target_us = 1500
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.az_g = 0.0
        self._images = {}
        self._walk_offset = 0
        self._pulse = 0.0

        self._load_images()
        self._build_layout()
        self.refresh_ports()
        self.after(READ_INTERVAL_MS, self.process_incoming)
        self.after(ANIMATION_INTERVAL_MS, self.update_animation)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _apply_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TFrame", background=COLOR_BG)
        style.configure("Card.TFrame", background=COLOR_CARD)
        style.configure("TLabel", background=COLOR_BG, foreground=COLOR_NAVY, font=("Segoe UI", 9))
        style.configure("Card.TLabel", background=COLOR_CARD, foreground=COLOR_NAVY, font=("Segoe UI", 9))
        style.configure("Muted.TLabel", background=COLOR_CARD, foreground=COLOR_GRAY_TEXT, font=("Segoe UI", 8))
        style.configure("Title.TLabel", background=COLOR_CARD, foreground=COLOR_NAVY, font=("Segoe UI", 20, "bold"))
        style.configure("Subtitle.TLabel", background=COLOR_CARD, foreground=COLOR_GRAY_TEXT, font=("Segoe UI", 10))
        style.configure("Section.TLabel", background=COLOR_CARD, foreground=COLOR_NAVY, font=("Segoe UI", 11, "bold"))
        style.configure("Status.TLabel", background=COLOR_CARD, foreground=COLOR_NAVY, font=("Consolas", 10))
        style.configure("Connected.TLabel", background=COLOR_CARD, foreground=COLOR_GREEN, font=("Segoe UI", 9, "bold"))
        style.configure("Disconnected.TLabel", background=COLOR_CARD, foreground=COLOR_RED, font=("Segoe UI", 9, "bold"))
        style.configure("TButton", font=("Segoe UI", 9), padding=(10, 6), relief="flat")
        style.configure("Primary.TButton", background=COLOR_CYAN, foreground="white", font=("Segoe UI", 9, "bold"), padding=(10, 6))
        style.map("Primary.TButton", background=[("active", COLOR_CYAN_DARK)])
        style.configure("Danger.TButton", background=COLOR_RED, foreground="white", font=("Segoe UI", 9, "bold"), padding=(10, 6))
        style.map("Danger.TButton", background=[("active", "#B83232")])
        style.configure("Warn.TButton", background=COLOR_YELLOW, foreground=COLOR_NAVY, font=("Segoe UI", 9, "bold"), padding=(10, 6))
        style.map("Warn.TButton", background=[("active", "#D9AD00")])
        style.configure("Turn.TButton", background=COLOR_BLUE, foreground="white", font=("Segoe UI", 9, "bold"), padding=(10, 6))
        style.map("Turn.TButton", background=[("active", "#16536F")])
        style.configure("TLabelframe", background=COLOR_CARD, bordercolor=COLOR_BORDER, relief="solid")
        style.configure("TLabelframe.Label", background=COLOR_CARD, foreground=COLOR_NAVY, font=("Segoe UI", 10, "bold"))
        style.configure("TEntry", fieldbackground="white")
        style.configure("TCombobox", fieldbackground="white")
        style.configure("Horizontal.TScale", background=COLOR_CARD)

    def _load_images(self):
        self._images["otomasi"] = load_scaled_photo(asset_path("logo_otomasi.png"), 96, 96)
        self._images["trik"] = load_scaled_photo(asset_path("logo_trik.png"), 260, 75)

    def _build_layout(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        self._build_header(root)
        self._build_connection_bar(root)

        main = ttk.Frame(root)
        main.pack(fill="both", expand=True, pady=(10, 0))
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.columnconfigure(2, weight=0)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        center = ttk.Frame(main)
        center.grid(row=0, column=1, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(main)
        right.grid(row=0, column=2, sticky="nse")

        self._build_left_panel(left)
        self._build_center_panel(center)
        self._build_right_panel(right)
        self._build_footer(root)

    def _build_header(self, parent):
        header = tk.Frame(parent, bg=COLOR_HEADER, highlightbackground=COLOR_BORDER, highlightthickness=1)
        header.pack(fill="x", ipady=6)
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=0)

        brand = tk.Frame(header, bg=COLOR_HEADER)
        brand.grid(row=0, column=0, sticky="w", padx=(18, 8), pady=4)

        if self._images.get("otomasi"):
            tk.Label(brand, image=self._images["otomasi"], bg=COLOR_HEADER).pack(side="left", padx=(0, 10))
        else:
            tk.Label(brand, text="OTOMASI", bg=COLOR_HEADER, fg=COLOR_CYAN, font=("Segoe UI", 13, "bold")).pack(side="left", padx=(0, 10))

        title_box = tk.Frame(brand, bg=COLOR_HEADER)
        title_box.pack(side="left", anchor="center")
        tk.Label(
            title_box,
            text="QUADRA-PAW CONTROL CENTER",
            bg=COLOR_HEADER,
            fg=COLOR_NAVY,
            font=("Segoe UI", 25, "bold"),
        ).pack(anchor="w")

        if self._images.get("trik"):
            tk.Label(header, image=self._images["trik"], bg=COLOR_HEADER).grid(row=0, column=1, sticky="e", padx=(8, 18), pady=4)
        else:
            tk.Label(header, text="TRIK UGM", bg=COLOR_HEADER, fg=COLOR_NAVY, font=("Segoe UI", 14, "bold")).grid(row=0, column=1, sticky="e", padx=(8, 18), pady=4)

    def _build_connection_bar(self, parent):
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.pack(fill="x", pady=(10, 0))
        card.columnconfigure(11, weight=1)

        ttk.Label(card, text="Koneksi ESP32", style="Section.TLabel").grid(row=0, column=0, padx=(0, 12), sticky="w")
        ttk.Label(card, text="Port", style="Card.TLabel").grid(row=0, column=1, padx=4, sticky="w")
        self.port_combo = ttk.Combobox(card, width=18, state="readonly")
        self.port_combo.grid(row=0, column=2, padx=4)
        ttk.Button(card, text="Refresh", command=self.refresh_ports).grid(row=0, column=3, padx=4)
        ttk.Label(card, text="Baud", style="Card.TLabel").grid(row=0, column=4, padx=4, sticky="w")
        self.baud_entry = ttk.Entry(card, width=10)
        self.baud_entry.insert(0, str(BAUD_DEFAULT))
        self.baud_entry.grid(row=0, column=5, padx=4)
        ttk.Button(card, text="Connect", style="Primary.TButton", command=self.connect_serial).grid(row=0, column=6, padx=4)
        ttk.Button(card, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=7, padx=4)
        self.connection_label = ttk.Label(card, textvariable=self.connection_var, style="Disconnected.TLabel")
        self.connection_label.grid(row=0, column=8, padx=12, sticky="w")
        ttk.Button(card, text="PING", command=lambda: self.send_command("PING")).grid(row=0, column=9, padx=4)
        ttk.Button(card, text="PARAM?", command=lambda: self.send_command("PARAM?")).grid(row=0, column=10, padx=4)

    def _build_left_panel(self, parent):
        status = ttk.LabelFrame(parent, text="Status Robot", padding=10)
        status.pack(fill="x", pady=(0, 10))
        self._status_row(status, 0, "Mode", self.mode_var)
        self._status_row(status, 1, "Fase", self.phase_var)
        self._status_row(status, 2, "MPU", self.mpu_var)
        self._status_row(status, 3, "Roll", self.roll_var)
        self._status_row(status, 4, "Pitch", self.pitch_var)
        self._status_row(status, 5, "Az", self.az_var)
        self._status_row(status, 6, "S5", self.s5_var)
        self._status_row(status, 7, "S6", self.s6_var)
        self._status_row(status, 8, "ACK", self.last_ack_var)

        control = ttk.LabelFrame(parent, text="Kontrol Gerak", padding=10)
        control.pack(fill="x", pady=(0, 10))
        ttk.Button(control, text="MAJU", style="Primary.TButton", command=lambda: self.send_command("MAJU")).grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="MUNDUR", style="Turn.TButton", command=lambda: self.send_command("MUNDUR")).grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="BELOK KIRI", style="Turn.TButton", command=lambda: self.send_command("KIRI")).grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="BELOK KANAN", style="Turn.TButton", command=lambda: self.send_command("KANAN")).grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="STOP", style="Danger.TButton", command=lambda: self.send_command("STOP")).grid(row=2, column=0, columnspan=2, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="BERDIRI", command=lambda: self.send_command("STAND")).grid(row=3, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="JONGKOK", command=lambda: self.send_command("JONGKOK")).grid(row=3, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="CEK MPU", command=lambda: self.send_command("MPU")).grid(row=4, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="SCAN I2C", command=lambda: self.send_command("I2C")).grid(row=4, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="PRINT PARAM", command=lambda: self.send_command("PRINT")).grid(row=5, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(control, text="KIRIM SEMUA", style="Warn.TButton", command=self.send_all_parameters).grid(row=5, column=1, padx=4, pady=4, sticky="ew")
        control.columnconfigure(0, weight=1)
        control.columnconfigure(1, weight=1)

        manual = ttk.LabelFrame(parent, text="Command Manual", padding=10)
        manual.pack(fill="x")
        self.manual_entry = ttk.Entry(manual, width=26)
        self.manual_entry.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.manual_entry.bind("<Return>", lambda event: self.send_manual())
        ttk.Button(manual, text="Kirim", command=self.send_manual).grid(row=0, column=1)
        manual.columnconfigure(0, weight=1)
        ttk.Label(manual, text="Contoh: MAJU, MUNDUR, KIRI, KANAN, S6MAJU=1300, S6MUNDUR=1500", style="Muted.TLabel").grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def _build_center_panel(self, parent):
        anim = ttk.LabelFrame(parent, text="Animasi Robot Tampak Atas", padding=8)
        anim.pack(fill="both", expand=True, pady=(0, 10))
        self.anim_canvas = tk.Canvas(anim, height=430, bg=COLOR_SOFT, highlightthickness=1, highlightbackground="#D6E0EA")
        self.anim_canvas.pack(fill="both", expand=True)

        monitor = ttk.LabelFrame(parent, text="Serial Monitor", padding=10)
        monitor.pack(fill="both", expand=True)
        self.log_text = tk.Text(monitor, wrap="word", height=14, bg=COLOR_DARK_LOG, fg="#DDEFFF", insertbackground="white", font=("Consolas", 9), relief="flat")
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(monitor, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _build_right_panel(self, parent):
        tuning = ttk.LabelFrame(parent, text="Tuning Parameter", padding=10)
        tuning.pack(fill="both", expand=True)
        ttk.Label(tuning, text="Parameter aktif", style="Muted.TLabel").grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 4))
        ttk.Checkbutton(tuning, text="Auto kirim slider", variable=self.auto_send_var).grid(row=1, column=0, columnspan=5, sticky="w", pady=(0, 2))
        ttk.Checkbutton(tuning, text="Kirim nilai GUI saat connect", variable=self.sync_on_connect_var).grid(row=2, column=0, columnspan=5, sticky="w", pady=(0, 8))
        row_cursor = 3
        for meta in PARAM_META:
            self._param_row(tuning, row_cursor, *meta)
            row_cursor += 1
        ttk.Button(tuning, text="Kirim Semua Parameter", style="Primary.TButton", command=self.send_all_parameters).grid(row=row_cursor, column=0, columnspan=5, sticky="ew", pady=(12, 2))
        ttk.Button(tuning, text="Ambil Parameter dari ESP", command=self.pull_parameters_from_esp).grid(row=row_cursor + 1, column=0, columnspan=5, sticky="ew", pady=(4, 0))
        ttk.Button(tuning, text="Reset Nilai GUI", command=self.reset_parameter_inputs).grid(row=row_cursor + 2, column=0, columnspan=5, sticky="ew", pady=(4, 0))
        ttk.Label(tuning, text="", style="Muted.TLabel").grid(row=row_cursor + 3, column=0, columnspan=5, sticky="w", pady=(10, 0))

    def _build_footer(self, parent):
        footer = ttk.Frame(parent, style="Card.TFrame", padding=(10, 8))
        footer.pack(fill="x", pady=(10, 0))
        ttk.Button(footer, text="Clear Log", command=lambda: self.log_text.delete("1.0", "end")).pack(side="left")
        ttk.Label(footer, text="Connect -> BERDIRI -> Kirim Semua Parameter -> MAJU / MUNDUR / KIRI / KANAN", style="Card.TLabel").pack(side="left", padx=12)
        ttk.Label(footer, text="REV10 LABEL FIX", style="Muted.TLabel").pack(side="right")

    def _status_row(self, parent, row, label, var):
        ttk.Label(parent, text=label, width=9, style="Card.TLabel").grid(row=row, column=0, padx=4, pady=3, sticky="w")
        ttk.Label(parent, text=":", style="Card.TLabel").grid(row=row, column=1, padx=2, pady=3)
        ttk.Label(parent, textvariable=var, width=18, style="Status.TLabel").grid(row=row, column=2, padx=4, pady=3, sticky="w")

    def _param_row(self, parent, row, command_name, label, min_val, max_val, hint):
        ttk.Label(parent, text=label, style="Card.TLabel", width=18).grid(row=row, column=0, sticky="w", padx=4, pady=5)
        slider = ttk.Scale(parent, from_=min_val, to=max_val, orient="horizontal")
        slider.set(DEFAULTS[command_name])
        slider.grid(row=row, column=1, sticky="ew", padx=4, pady=5)
        entry = ttk.Entry(parent, width=9, textvariable=self.param_entry_vars[command_name], justify="center")
        entry.grid(row=row, column=2, padx=4, pady=5)
        ttk.Button(parent, text="Set", command=lambda n=command_name: self.set_single_parameter(n)).grid(row=row, column=3, padx=4, pady=5)
        ttk.Label(parent, text=f"{min_val}..{max_val}", style="Muted.TLabel", width=12).grid(row=row, column=4, padx=(0, 4), sticky="w")
        # hint disembunyikan agar panel tuning lebih bersih

        def slider_changed(_value=None, name=command_name):
            value = int(float(slider.get()))
            self.param_vars[name].set(value)
            self.param_entry_vars[name].set(str(value))

        slider.configure(command=slider_changed)
        slider.bind("<ButtonRelease-1>", lambda event, n=command_name: self.set_single_parameter(n) if self.auto_send_var.get() else None)
        entry.bind("<Return>", lambda event, n=command_name: self.set_single_parameter(n))
        entry.bind("<FocusOut>", lambda event, n=command_name: self.set_single_parameter(n) if (self.auto_send_var.get() and self.serial_worker.is_connected()) else self._sync_entry_to_slider(n))
        parent.columnconfigure(1, weight=1)
        self.param_widgets[command_name] = {"slider": slider, "entry": entry, "min": min_val, "max": max_val}

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.current(0)

    def connect_serial(self):
        port = self.port_combo.get().strip()
        if not port:
            messagebox.showwarning("Port kosong", "Pilih COM port ESP32 dulu.")
            return
        try:
            baud = int(self.baud_entry.get().strip())
        except ValueError:
            messagebox.showwarning("Baud salah", "Baud harus angka, contoh 115200.")
            return
        try:
            self.serial_worker.connect(port, baud)
            self.connection_var.set(f"Connected: {port}")
            self.connection_label.configure(style="Connected.TLabel")
            self.append_log(f"[GUI] Connected to {port} at {baud}")
            self.append_log("[GUI] Nilai tuning GUI dipertahankan, tidak di-reset dari ESP.")
            self.after(250, lambda: self.send_command("PING", quiet=False))
            if self.sync_on_connect_var.get():
                self.after(850, self.send_all_parameters)
            else:
                self.after(850, lambda: self.send_command("PARAM?", quiet=False))
        except Exception as exc:
            messagebox.showerror("Gagal connect", str(exc))

    def disconnect_serial(self):
        self.serial_worker.disconnect()
        self.connection_var.set("Disconnected")
        self.connection_label.configure(style="Disconnected.TLabel")
        self.append_log("[GUI] Disconnected")

    def send_manual(self):
        text = self.manual_entry.get().strip()
        if text:
            self.send_command(text)
            self.manual_entry.delete(0, "end")

    def _get_param_value_from_entry(self, name):
        meta = self.param_widgets[name]
        raw = self.param_entry_vars[name].get().strip()
        try:
            value = int(float(raw))
        except ValueError:
            messagebox.showwarning("Input salah", f"Nilai {name} harus angka.")
            return None
        value = max(meta["min"], min(meta["max"], value))
        self.param_vars[name].set(value)
        self.param_entry_vars[name].set(str(value))
        meta["slider"].set(value)
        return value

    def _sync_entry_to_slider(self, name):
        self._get_param_value_from_entry(name)

    def set_single_parameter(self, name):
        value = self._get_param_value_from_entry(name)
        if value is not None:
            if self.serial_worker.is_connected():
                self.send_command(f"{name}={value}")
                self.after(180, lambda: self.send_command("PARAM?", quiet=True))
            else:
                self.append_log(f"[GUI] {name}={value} disimpan lokal. Nilai akan dikirim saat Connect/Kirim Semua.")

    def send_all_parameters(self):
        commands = []
        for name, *_ in PARAM_META:
            value = self._get_param_value_from_entry(name)
            if value is not None:
                commands.append(f"{name}={value}")
        commands.append("PARAM?")
        self._send_command_sequence(commands)

    def pull_parameters_from_esp(self):
        self.apply_esp_params_until = time.monotonic() + 2.0
        self.send_command("PARAM?")

    def _send_command_sequence(self, commands, index=0):
        if index >= len(commands):
            return
        quiet = commands[index] == "PARAM?"
        self.send_command(commands[index], quiet=quiet)
        self.after(90, lambda: self._send_command_sequence(commands, index + 1))

    def reset_parameter_inputs(self):
        for name, *_ in PARAM_META:
            value = DEFAULTS[name]
            self.param_vars[name].set(value)
            self.param_entry_vars[name].set(str(value))
            self.param_widgets[name]["slider"].set(value)
        self.append_log("[GUI] Nilai parameter GUI dikembalikan ke default lokal.")

    def send_command(self, command, quiet=False):
        try:
            self.serial_worker.send_line(command)
            if not quiet:
                self.append_log(f">> {command}")
        except Exception as exc:
            self.append_log(f"[GUI ERROR] {exc}")

    def process_incoming(self):
        while not self.incoming.empty():
            line = self.incoming.get_nowait()
            self.append_log(line)
            self.parse_status_line(line)
        self.after(READ_INTERVAL_MS, self.process_incoming)

    def parse_status_line(self, line):
        data = {}
        for part in re.split(r"[,;\s]+", line):
            if "=" in part:
                key, value = part.split("=", 1)
                data[key.strip().upper()] = value.strip()

        if "ACK" in data:
            self.last_ack_var.set(data["ACK"])
            ack_name = data["ACK"].upper()
            if ack_name in self.param_widgets and "VALUE" in data:
                value = self._safe_int(data["VALUE"], None)
                if value is not None:
                    self.param_entry_vars[ack_name].set(str(value))
                    self.param_widgets[ack_name]["slider"].set(value)
            if ack_name == "S6BELOK" and "VALUE" in data:
                value = self._safe_int(data["VALUE"], None)
                if value is not None:
                    for key in ("S6KANAN", "S6KIRI"):
                        if key in self.param_widgets:
                            self.param_entry_vars[key].set(str(value))
                            self.param_widgets[key]["slider"].set(value)
        if "READY" in data:
            self.last_ack_var.set("READY")
        if "MODE" in data:
            self.mode_var.set(data["MODE"])
        if "PHASE" in data or "FASE" in data:
            self.phase_var.set(data.get("PHASE", data.get("FASE", "-")))
        if "PHASE_CHANGE" in data:
            self.phase_var.set(data["PHASE_CHANGE"])
        if "MPU" in data:
            self.mpu_var.set(data["MPU"])
        if "ROLL" in data:
            self.roll_var.set(data["ROLL"])
            self.roll_deg = self._safe_float(data["ROLL"], self.roll_deg)
        if "PITCH" in data:
            self.pitch_var.set(data["PITCH"])
            self.pitch_deg = self._safe_float(data["PITCH"], self.pitch_deg)
        if "AZ" in data:
            self.az_var.set(data["AZ"])
            self.az_g = self._safe_float(data["AZ"], self.az_g)
        if "S5" in data:
            self.s5_var.set(data["S5"])
            self.s5_now_us, self.s5_target_us = self._parse_servo_pair(data["S5"], self.s5_now_us, self.s5_target_us)
        if "S6" in data:
            self.s6_var.set(data["S6"])
            self.s6_now_us, self.s6_target_us = self._parse_servo_pair(data["S6"], self.s6_now_us, self.s6_target_us)
        boleh_update_dari_esp = time.monotonic() < self.apply_esp_params_until
        for name, *_ in PARAM_META:
            if name in data and boleh_update_dari_esp:
                value = self._safe_int(data[name], None)
                if value is not None and name in self.param_widgets:
                    self.param_entry_vars[name].set(str(value))
                    self.param_widgets[name]["slider"].set(value)

        if "LIFT" in data and "LIFTF" not in data and "LIFTB" not in data and boleh_update_dari_esp:
            value = self._safe_int(data["LIFT"], None)
            if value is not None:
                for key in ("LIFTF", "LIFTB"):
                    if key in self.param_widgets:
                        self.param_entry_vars[key].set(str(value))
                        self.param_widgets[key]["slider"].set(value)

        if "S6TRIM" in data and "S6MAJU" not in data and boleh_update_dari_esp:
            value = self._safe_int(data["S6TRIM"], None)
            if value is not None and "S6MAJU" in self.param_widgets:
                self.param_entry_vars["S6MAJU"].set(str(value))
                self.param_widgets["S6MAJU"]["slider"].set(value)
        if "S6K" in data and boleh_update_dari_esp:
            value = self._safe_int(data["S6K"], None)
            if value is not None:
                for key in ("S6KANAN", "S6KIRI"):
                    if key in self.param_widgets and key not in data:
                        self.param_entry_vars[key].set(str(abs(value)))
                        self.param_widgets[key]["slider"].set(abs(value))

    def _parse_servo_pair(self, text, default_now, default_target):
        match = re.match(r"(-?\d+)(?:/(-?\d+))?", text.strip())
        if not match:
            return default_now, default_target
        now = int(match.group(1))
        target = int(match.group(2)) if match.group(2) is not None else now
        return now, target

    def _safe_float(self, text, default):
        try:
            return float(text)
        except Exception:
            return default

    def _safe_int(self, text, default):
        try:
            return int(float(text))
        except Exception:
            return default

    def update_animation(self):
        self._pulse += 0.12
        mode = self.mode_var.get().upper()
        if mode in ["MAJU", "MUNDUR", "KIRI", "KANAN"]:
            self._walk_offset = (self._walk_offset + 1) % 40
        self.draw_robot_animation()
        self.after(ANIMATION_INTERVAL_MS, self.update_animation)

    def draw_robot_animation(self):
        c = self.anim_canvas
        c.delete("all")
        w = max(c.winfo_width(), 760)
        h = max(c.winfo_height(), 430)
        cx = w // 2
        cy = h // 2 + 10

        phase = self.phase_var.get().upper()
        mode = self.mode_var.get().upper()
        moving = mode in ["MAJU", "MUNDUR", "KIRI", "KANAN"]

        c.create_rectangle(0, 0, w, h, fill=COLOR_SOFT, outline="")
        for x in range(0, w, 32):
            c.create_line(x, 0, x, h, fill="#EEF4FA")
        for y in range(0, h, 32):
            c.create_line(0, y, w, y, fill="#EEF4FA")

        c.create_text(18, 18, anchor="w", text=f"Mode: {self.mode_var.get()} | Fase: {self.phase_var.get()}", fill=COLOR_NAVY, font=("Segoe UI", 11, "bold"))
        c.create_text(18, 40, anchor="w", text=f"Roll {self.roll_deg:.1f} deg   Pitch {self.pitch_deg:.1f} deg   MPU {self.mpu_var.get()}", fill=COLOR_GRAY_TEXT, font=("Segoe UI", 9))
        c.create_text(cx, 25, text="DEPAN", fill=COLOR_NAVY, font=("Segoe UI", 12, "bold"))
        c.create_line(cx, 42, cx, 72, arrow="first", fill=COLOR_CYAN, width=3)

        self._draw_motion_arrow(c, mode, cx, cy, w)

        body_w = 210
        body_h = 175
        body_x1 = cx - body_w // 2
        body_y1 = cy - body_h // 2
        body_x2 = cx + body_w // 2
        body_y2 = cy + body_h // 2

        tilt_x = max(-12, min(12, self.roll_deg * 0.45))
        tilt_y = max(-10, min(10, self.pitch_deg * 0.32))
        shadow = 6

        c.create_polygon(
            body_x1 + tilt_x + shadow, body_y1 - tilt_y + shadow,
            body_x2 + tilt_x + shadow, body_y1 + tilt_y + shadow,
            body_x2 - tilt_x + shadow, body_y2 + tilt_y + shadow,
            body_x1 - tilt_x + shadow, body_y2 - tilt_y + shadow,
            fill="#AEBBC5", outline="",
        )
        c.create_polygon(
            body_x1 + tilt_x, body_y1 - tilt_y,
            body_x2 + tilt_x, body_y1 + tilt_y,
            body_x2 - tilt_x, body_y2 + tilt_y,
            body_x1 - tilt_x, body_y2 - tilt_y,
            fill="#0F3A5E", outline=COLOR_NAVY, width=3,
        )
        c.create_rectangle(body_x1 + 22, body_y1 + 22, body_x2 - 22, body_y2 - 22, fill="#D6DEE6", outline="#7C8B98", width=2)
        c.create_rectangle(body_x1 + 34, body_y1 + 38, body_x2 - 34, body_y2 - 38, fill="#AEB7BF", outline="#6C7882")
        c.create_text(cx, body_y1 + 45, text="BODY", fill=COLOR_NAVY, font=("Segoe UI", 14, "bold"))
        c.create_text(cx, body_y2 - 26, text="S5 + S6 di belakang", fill=COLOR_GRAY_TEXT, font=("Segoe UI", 9))

        p1_active = phase.startswith("P1") or phase.startswith("S5_GERAK_1")
        p2_active = phase.startswith("P2") or phase.startswith("S5_GERAK_2")
        p1_lifted = p1_active and not ("TEPAK" in phase or "STABIL" in phase or "TURUN" in phase)
        p2_lifted = p2_active and not ("TEPAK" in phase or "STABIL" in phase or "TURUN" in phase)
        if not moving:
            p1_lifted = False
            p2_lifted = False

        legs = [
            ("S1", body_x1 + 4, body_y1 + 32, cx - 195, body_y1 - 18, p2_lifted, "LF"),
            ("S2", body_x2 - 4, body_y1 + 32, cx + 195, body_y1 - 18, p1_lifted, "RF"),
            ("S3", body_x1 + 4, body_y2 - 32, cx - 195, body_y2 + 60, p1_lifted, "LB"),
            ("S4", body_x2 - 4, body_y2 - 32, cx + 195, body_y2 + 60, p2_lifted, "RB"),
        ]
        for code, sx, sy, fx, fy, lifted, _pos in legs:
            self._draw_leg(c, code, sx, sy, fx, fy, lifted)

        self._draw_slider_and_steering(c, cx, body_y2 + 108, mode)
        self._draw_mode_badge(c, mode, 18, h - 70)

    def _draw_motion_arrow(self, canvas, mode, cx, cy, width):
        color = COLOR_CYAN if mode in ["MAJU", "MUNDUR"] else COLOR_ORANGE
        if mode == "MAJU":
            canvas.create_line(cx - 330, cy + 150, cx - 330, cy + 60, arrow="last", fill=color, width=5)
            canvas.create_text(cx - 330, cy + 170, text="MAJU", fill=color, font=("Segoe UI", 12, "bold"))
        elif mode == "MUNDUR":
            canvas.create_line(cx - 330, cy + 60, cx - 330, cy + 150, arrow="last", fill=color, width=5)
            canvas.create_text(cx - 330, cy + 170, text="MUNDUR", fill=color, font=("Segoe UI", 12, "bold"))
        elif mode == "KIRI":
            canvas.create_arc(cx - 360, cy - 40, cx - 260, cy + 90, start=260, extent=230, style="arc", outline=color, width=5)
            canvas.create_line(cx - 352, cy - 20, cx - 373, cy - 35, arrow="last", fill=color, width=4)
            canvas.create_text(cx - 320, cy + 120, text="BELOK KIRI", fill=color, font=("Segoe UI", 12, "bold"))
        elif mode == "KANAN":
            canvas.create_arc(cx + 260, cy - 40, cx + 360, cy + 90, start=50, extent=230, style="arc", outline=color, width=5)
            canvas.create_line(cx + 352, cy - 20, cx + 373, cy - 35, arrow="last", fill=color, width=4)
            canvas.create_text(cx + 320, cy + 120, text="BELOK KANAN", fill=color, font=("Segoe UI", 12, "bold"))

    def _draw_leg(self, canvas, code, sx, sy, fx, fy, lifted):
        if lifted:
            lift_phase = 0.5 + 0.5 * math.sin(self._pulse)
            ex = sx + (fx - sx) * (0.55 + 0.08 * lift_phase)
            ey = sy + (fy - sy) * (0.55 + 0.08 * lift_phase)
            line_color = COLOR_ORANGE
            foot_fill = "#FFE0A3"
            dash = (6, 4)
            status = "ANGKAT"
            foot_r = 17
            y_bump = -8 * lift_phase
        else:
            ex = fx
            ey = fy
            line_color = "#202B33"
            foot_fill = "#D95C5C"
            dash = None
            status = "TUMPU"
            foot_r = 22
            y_bump = 0

        canvas.create_oval(sx - 8, sy - 8, sx + 8, sy + 8, fill=COLOR_CYAN, outline=COLOR_NAVY, width=2)
        mid_x = sx + (ex - sx) * 0.53
        mid_y = sy + (ey - sy) * 0.53 + y_bump
        canvas.create_line(sx, sy, mid_x, mid_y, fill="#CF2E2E" if not lifted else COLOR_ORANGE, width=9, dash=dash, capstyle="round")
        canvas.create_line(mid_x, mid_y, ex, ey + y_bump, fill="#F4F7FA" if not lifted else "#FFD08A", width=8, dash=dash, capstyle="round")
        canvas.create_oval(ex - foot_r, ey - foot_r + y_bump, ex + foot_r, ey + foot_r + y_bump, fill=foot_fill, outline=line_color, width=2)
        canvas.create_text(ex, ey - 34 + y_bump, text=code, fill=COLOR_NAVY, font=("Segoe UI", 11, "bold"))
        canvas.create_text(ex, ey + 36 + y_bump, text=status, fill=COLOR_GRAY_TEXT, font=("Segoe UI", 8, "bold"))

    def _draw_slider_and_steering(self, canvas, cx, y, mode):
        rail_x1 = cx - 150
        rail_x2 = cx + 150
        canvas.create_text(cx, y - 40, text="MEKANISME BELAKANG", fill=COLOR_NAVY, font=("Segoe UI", 10, "bold"))
        canvas.create_line(rail_x1, y, rail_x2, y, fill="#6A6A6A", width=6)
        canvas.create_line(rail_x1, y - 13, rail_x1, y + 13, fill="#555555", width=3)
        canvas.create_line(rail_x2, y - 13, rail_x2, y + 13, fill="#555555", width=3)
        canvas.create_text(rail_x1 - 38, y, text="S5", fill=COLOR_NAVY, font=("Segoe UI", 11, "bold"))

        s5_x = self._map_value(self.s5_now_us, 1000, 2000, rail_x1 + 20, rail_x2 - 20)
        s5_t = self._map_value(self.s5_target_us, 1000, 2000, rail_x1 + 20, rail_x2 - 20)
        canvas.create_rectangle(s5_x - 21, y - 16, s5_x + 21, y + 16, fill="#F5F8FA", outline=COLOR_NAVY, width=2)
        canvas.create_rectangle(s5_x - 8, y - 14, s5_x + 8, y + 14, fill=COLOR_CYAN, outline="")
        canvas.create_line(s5_t, y - 30, s5_t, y + 30, fill=COLOR_RED, width=2)
        canvas.create_text(cx, y + 35, text=f"S5 now/target: {self.s5_now_us}/{self.s5_target_us}", fill=COLOR_GRAY_TEXT, font=("Segoe UI", 8))

        steer_y = y + 86
        canvas.create_oval(cx - 26, steer_y - 26, cx + 26, steer_y + 26, fill="#EFF4F8", outline=COLOR_NAVY, width=2)
        servo_angle = self._map_value(self.s6_now_us, 1000, 2000, -55, 55)
        angle_rad = math.radians(90 + servo_angle)
        length = 92
        end_x = cx + math.cos(angle_rad) * length
        end_y = steer_y + math.sin(angle_rad) * length
        canvas.create_line(cx, steer_y, end_x, end_y, arrow="last", fill="#7B2CBF", width=6)
        canvas.create_text(cx - 56, steer_y, text="S6", fill=COLOR_NAVY, font=("Segoe UI", 11, "bold"))
        canvas.create_text(cx, steer_y + 49, text=f"S6 now/target: {self.s6_now_us}/{self.s6_target_us}", fill=COLOR_GRAY_TEXT, font=("Segoe UI", 8))
        if mode in ["KIRI", "KANAN"]:
            pass
        elif mode in ["MAJU", "MUNDUR"]:
            pass

    def _draw_mode_badge(self, canvas, mode, x, y):
        colors = {
            "MAJU": COLOR_CYAN,
            "MUNDUR": COLOR_BLUE,
            "KIRI": COLOR_ORANGE,
            "KANAN": COLOR_ORANGE,
            "STOP": COLOR_RED,
            "STAND": COLOR_GREEN,
            "JONGKOK": COLOR_YELLOW,
        }
        fill = colors.get(mode, "#9AA6B2")
        canvas.create_rectangle(x, y, x + 220, y + 48, fill=fill, outline="")
        canvas.create_text(x + 14, y + 14, anchor="w", text="QUADRA-PAW", fill="white", font=("Segoe UI", 8, "bold"))
        canvas.create_text(x + 14, y + 32, anchor="w", text=mode if mode else "-", fill="white", font=("Segoe UI", 14, "bold"))

    def _map_value(self, value, in_min, in_max, out_min, out_max):
        try:
            v = float(value)
        except Exception:
            v = float(in_min)
        v = max(in_min, min(in_max, v))
        return out_min + (v - in_min) * (out_max - out_min) / (in_max - in_min)

    def append_log(self, text):
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")

    def on_close(self):
        self.serial_worker.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = RobotGUI()
    app.mainloop()
