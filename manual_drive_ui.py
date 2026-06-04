"""
manual_drive_ui.py
------------------
Tkinter dashboard for manual testing of engine-sim-headless.

Features:
  - Engine selector  (scans assets/engines/**/*.mr)
  - Launch / stop engine-sim-headless subprocess
  - Accelerator + brake sliders  with Tesla-style regen physics
  - Speed / pedal / brake / regen state display
  - Backfire toggle
  - Persistent config  (engine_ui_config.json)

Usage:
    python manual_drive_ui.py
    python manual_drive_ui.py --host 192.168.1.5 --port 9999 --hz 40
    python manual_drive_ui.py --exe /path/to/engine-sim-headless
    python manual_drive_ui.py --engines-dir /path/to/assets/engines

Requires:
    Python 3.8+  (standard library only; no extra packages needed)
"""

import argparse
import json
import os
import pathlib
import platform
import socket
import subprocess
import time
import tkinter as tk
from tkinter import messagebox, ttk

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

CONFIG_FILE = "engine_ui_config.json"
_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent


def _default_exe() -> str:
    """Platform-appropriate default path for the engine-sim-headless binary."""
    if platform.system() == "Windows":
        return str(_SCRIPT_DIR / "build" / "engine-sim-headless.exe")
    return str(_SCRIPT_DIR / "build" / "engine-sim-headless")


def _default_engines_dir() -> str:
    return str(_SCRIPT_DIR / "assets" / "engines")


def scan_engines(engines_dir: str) -> list:
    """
    Recursively find all *.mr files under engines_dir.
    Returns [(friendly_name, absolute_path), ...] sorted by friendly_name.
    friendly_name is the path relative to engines_dir, e.g. 'atg-video-2/07_gm_ls.mr'.
    """
    root = pathlib.Path(engines_dir)
    if not root.exists():
        return []
    results = []
    for mr in root.rglob("*.mr"):
        try:
            rel = mr.relative_to(root)
        except ValueError:
            rel = mr
        friendly = str(rel).replace("\\", "/")
        results.append((friendly, str(mr.resolve())))
    return sorted(results, key=lambda t: t[0])


def load_config() -> dict:
    try:
        with open(CONFIG_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(updates: dict) -> None:
    try:
        data = load_config()
        data.update(updates)
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

class App:
    """Main application: engine launcher + UI + physics loop + UDP output."""

    def __init__(
        self,
        root: tk.Tk,
        host: str,
        port: int,
        hz: float,
        exe_path: str,
        engines_dir: str,
    ) -> None:
        self.root      = root
        self.addr      = (host, port)
        self.period_ms = max(10, round(1000.0 / hz))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subprocess state
        self._exe_path        = exe_path
        self._proc            = None    # subprocess.Popen | None
        self._startup_pending = False
        self._startup_after   = 0.0    # monotonic timestamp

        # Physics state
        self.speed_mph  = 0.0
        self.regen_mps2 = 0.0
        self.last_time  = time.monotonic()
        self.last_print = time.monotonic()

        # Engine list
        engines            = scan_engines(engines_dir)
        self._engine_map   = dict(engines)         # friendly -> abs_path
        self._engine_names = [e[0] for e in engines]

        # Restore last selection
        cfg = load_config()
        self._saved_engine = cfg.get("last_engine", "")

        self._build_ui(host, port, hz)
        self._startup()                            # send to any already-running process
        self.root.after(self.period_ms, self._loop)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self, host: str, port: int, hz: float) -> None:
        self.root.title("Manual Drive — engine-sim-headless")
        self.root.resizable(False, False)

        PAD = 12

        # ── header ────────────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg="#1c1c2e", padx=PAD, pady=8)
        hdr.pack(fill=tk.X)
        tk.Label(
            hdr, text="Manual Drive UI",
            fg="#e8e8e8", bg="#1c1c2e",
            font=("TkDefaultFont", 12, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            hdr, text=f"{host}:{port}   {hz:.0f} Hz",
            fg="#8888aa", bg="#1c1c2e",
            font=("TkDefaultFont", 10),
        ).pack(side=tk.RIGHT)

        # ── engine panel ──────────────────────────────────────────────
        eng = tk.LabelFrame(self.root, text=" Engine ", padx=PAD, pady=PAD)
        eng.pack(fill=tk.X, padx=PAD, pady=(PAD, 4))
        eng.columnconfigure(0, weight=1)

        # Row 0: combobox + launch button
        combo_row = tk.Frame(eng)
        combo_row.grid(row=0, column=0, columnspan=2, sticky="ew")
        combo_row.columnconfigure(0, weight=1)

        if self._engine_names:
            self.engine_combo = ttk.Combobox(
                combo_row, values=self._engine_names,
                state="readonly", width=52,
            )
            initial = (
                self._saved_engine
                if self._saved_engine in self._engine_map
                else self._engine_names[0]
            )
            self.engine_combo.set(initial)
        else:
            self.engine_combo = ttk.Combobox(
                combo_row, values=["(no .mr files found)"],
                state="disabled", width=52,
            )
            self.engine_combo.set("(no .mr files found)")

        self.engine_combo.grid(row=0, column=0, sticky="ew", padx=(0, 6))

        self._launch_btn = tk.Button(
            combo_row, text="Launch Engine",
            command=self._on_launch,
            width=14,
        )
        self._launch_btn.grid(row=0, column=1, sticky="e")
        if not self._engine_names:
            self._launch_btn.config(state=tk.DISABLED)

        # Row 1: engine status
        status_row = tk.Frame(eng)
        status_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        tk.Label(status_row, text="Status:", width=7, anchor="w").pack(side=tk.LEFT)
        self.sv_eng_status = tk.StringVar(value="○ Stopped")
        self._status_lbl = tk.Label(
            status_row, textvariable=self.sv_eng_status,
            fg="#888888", anchor="w",
            font=("TkDefaultFont", 10, "bold"),
        )
        self._status_lbl.pack(side=tk.LEFT)

        # Row 2: current path
        path_row = tk.Frame(eng)
        path_row.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(3, 0))

        tk.Label(path_row, text="Path:", width=7, anchor="w").pack(side=tk.LEFT)
        self.sv_eng_path = tk.StringVar(value="—")
        tk.Label(
            path_row, textvariable=self.sv_eng_path,
            anchor="w", fg="#666666",
            font=("TkDefaultFont", 9),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # ── sliders ───────────────────────────────────────────────────
        ctrl = tk.LabelFrame(self.root, text=" Controls ", padx=PAD, pady=PAD)
        ctrl.pack(fill=tk.X, padx=PAD, pady=4)
        ctrl.columnconfigure(1, weight=1)

        self.pedal_var = tk.DoubleVar(value=0.0)
        self.brake_var = tk.DoubleVar(value=0.0)
        self._add_slider(ctrl, row=0, label="Accelerator", var=self.pedal_var)
        self._add_slider(ctrl, row=1, label="Brake",       var=self.brake_var)

        # ── state display ─────────────────────────────────────────────
        disp = tk.LabelFrame(self.root, text=" State ", padx=PAD, pady=PAD)
        disp.pack(fill=tk.X, padx=PAD, pady=4)

        self.sv_speed = tk.StringVar(value="      0.0 mph")
        self.sv_pedal = tk.StringVar(value="        0 %")
        self.sv_brake = tk.StringVar(value="     0.00")
        self.sv_regen = tk.StringVar(value="     0.00 m/s²")

        MONO       = ("Courier", 11)
        SPEED_FONT = ("Courier", 20, "bold")

        for i, (name, sv, fnt) in enumerate([
            ("Speed",  self.sv_speed, SPEED_FONT),
            ("Pedal",  self.sv_pedal, MONO),
            ("Brake",  self.sv_brake, MONO),
            ("Regen",  self.sv_regen, MONO),
        ]):
            tk.Label(disp, text=f"{name}:", width=7, anchor="w").grid(
                row=i, column=0, sticky="w", pady=3)
            tk.Label(disp, textvariable=sv, width=18, anchor="e", font=fnt).grid(
                row=i, column=1, sticky="e")

        # ── options ───────────────────────────────────────────────────
        opts = tk.Frame(self.root, padx=PAD, pady=8)
        opts.pack(fill=tk.X, padx=PAD, pady=(4, PAD))

        self.backfire_var = tk.BooleanVar(value=False)
        self.backfire_var.trace_add("write", self._on_backfire)
        tk.Checkbutton(
            opts, text="Enable backfire",
            variable=self.backfire_var,
            font=("TkDefaultFont", 11),
        ).pack(side=tk.LEFT)

    def _add_slider(
        self, parent: tk.Frame, row: int, label: str, var: tk.DoubleVar
    ) -> None:
        tk.Label(parent, text=label, width=12, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(0, 6), pady=5)

        pct_lbl = tk.Label(parent, text="  0 %", width=6, anchor="e")
        pct_lbl.grid(row=row, column=2, sticky="e")

        def on_move(value: str) -> None:
            pct_lbl.config(text=f"{float(value):3.0f} %")

        tk.Scale(
            parent, variable=var,
            from_=0, to=100, orient=tk.HORIZONTAL,
            length=360, showvalue=False,
            command=on_move,
        ).grid(row=row, column=1, sticky="ew", padx=4)

    # ------------------------------------------------------------------
    # Engine process management
    # ------------------------------------------------------------------

    def _on_launch(self) -> None:
        """Validate and start the selected engine-sim-headless process."""
        if not os.path.isfile(self._exe_path):
            messagebox.showerror(
                "Executable not found",
                f"engine-sim-headless not found:\n{self._exe_path}\n\n"
                "Build the project first (cmake --build build).",
            )
            return

        friendly = self.engine_combo.get()
        if not friendly or friendly not in self._engine_map:
            messagebox.showwarning("No engine selected", "Select an engine first.")
            return

        script = self._engine_map[friendly]
        if not os.path.isfile(script):
            messagebox.showerror(
                "Script not found", f"Engine script not found:\n{script}"
            )
            return

        self._stop_engine()

        cmd = [self._exe_path, "--script", script, "--port", str(self.addr[1])]
        print(f"[ui] Launch: {' '.join(cmd)}")
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=None,   # let stderr flow to this terminal
            )
        except OSError as exc:
            messagebox.showerror("Launch failed", str(exc))
            return

        # Schedule startup commands after engine has 2 s to initialise
        self._startup_pending = True
        self._startup_after   = time.monotonic() + 2.0

        save_config({"last_engine": friendly})
        self.sv_eng_path.set(script)
        self._refresh_status()
        print(f"[ui] PID {self._proc.pid} started; "
              f"startup commands in 2 s...")

    def _stop_engine(self) -> None:
        if self._proc is not None:
            print(f"[ui] Stopping PID {self._proc.pid}...")
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
            self._proc = None
        self._startup_pending = False
        self._refresh_status()

    def _engine_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _refresh_status(self) -> None:
        if self._engine_running():
            self.sv_eng_status.set("● Running")
            self._status_lbl.config(fg="#22cc44")
        else:
            self.sv_eng_status.set("○ Stopped")
            self._status_lbl.config(fg="#888888")
            if self._proc is not None:          # process died unexpectedly
                self._proc = None

    # ------------------------------------------------------------------
    # UDP helpers
    # ------------------------------------------------------------------

    def _send(self, cmd: str) -> None:
        try:
            self.sock.sendto((cmd + "\n").encode(), self.addr)
        except OSError:
            pass

    def _startup(self) -> None:
        for cmd in ("mode hybrid", "virtual_gears on", "vgear 0", "backfire off"):
            self._send(cmd)

    def _on_backfire(self, *_) -> None:
        self._send("backfire on" if self.backfire_var.get() else "backfire off")

    # ------------------------------------------------------------------
    # Main update loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        now = time.monotonic()
        dt  = min(now - self.last_time, 0.2)
        self.last_time = now

        # Send startup commands once the engine has had time to start
        if self._startup_pending and now >= self._startup_after:
            self._startup_pending = False
            self._startup()
            print("[ui] Startup commands sent.")

        # Keep status indicator fresh
        self._refresh_status()

        pedal_pct = self.pedal_var.get()
        brake_pct = self.brake_var.get()

        # ── physics ────────────────────────────────────────────────────
        accel_mps2 = pedal_pct / 100.0 * 8.5

        if pedal_pct < 5.0:
            self.regen_mps2 = 3.5 * min(self.speed_mph / 30.0, 1.0)
        else:
            self.regen_mps2 = 0.0

        brake_mps2 = brake_pct / 100.0 * 9.0
        net_accel  = accel_mps2 - self.regen_mps2 - brake_mps2

        self.speed_mph += net_accel * dt * 2.23694
        self.speed_mph  = max(0.0, min(130.0, self.speed_mph))

        brake_out = brake_pct / 100.0   # regen NOT included

        # ── UDP ────────────────────────────────────────────────────────
        self._send(f"speed {self.speed_mph:.2f}")
        self._send(f"pedal {pedal_pct:.1f}")
        self._send(f"brake {brake_out:.3f}")

        # ── display ────────────────────────────────────────────────────
        self.sv_speed.set(f"{self.speed_mph:8.1f} mph")
        self.sv_pedal.set(f"{pedal_pct:8.0f} %")
        self.sv_brake.set(f"{brake_out:8.2f}")
        self.sv_regen.set(f"{self.regen_mps2:8.2f} m/s²")

        # ── console log every 0.5 s ────────────────────────────────────
        if now - self.last_print >= 0.5:
            self.last_print = now
            print(
                f"speed={self.speed_mph:6.1f} mph  "
                f"pedal={pedal_pct:5.1f}%  "
                f"brake={brake_out:.2f}  "
                f"regen={self.regen_mps2:.2f} m/s²"
            )

        self.root.after(self.period_ms, self._loop)

    # ------------------------------------------------------------------

    def _on_close(self) -> None:
        self._stop_engine()
        self.sock.close()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual drive UI + engine launcher for engine-sim-headless"
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="headless UDP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9999,
                        help="UDP port (default: 9999)")
    parser.add_argument("--hz",   type=float, default=20.0,
                        help="update rate in Hz (default: 20)")
    parser.add_argument("--exe",  default=_default_exe(),
                        help=f"engine-sim-headless binary path "
                             f"(default: {_default_exe()})")
    parser.add_argument("--engines-dir", default=_default_engines_dir(),
                        help=f"directory to scan for .mr scripts "
                             f"(default: {_default_engines_dir()})")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.host, args.port, args.hz, args.exe, args.engines_dir)
    root.mainloop()


if __name__ == "__main__":
    main()
