#!/usr/bin/env python3
"""
main.py
-------
One-click launcher for the Tesla CAN -> engine-sim sound system.

Manages:
  1. engine-sim-headless subprocess
  2. FeederCore (CAN bridge, runs in-process in a background thread)
  3. tkinter dashboard UI

Usage:
    python main.py
    python main.py --config custom.yaml
"""

import argparse
import os
import pathlib
import platform
import subprocess

_SYS = platform.system()   # 'Darwin' | 'Linux' | 'Windows'
_DEFAULT_IFACE   = "socketcan"          if _SYS == "Linux"  else (
                   "slcan"              if _SYS == "Darwin" else "slcan")
_DEFAULT_CHANNEL = "can0"              if _SYS == "Linux"  else (
                   "/dev/tty.usbmodem0" if _SYS == "Darwin" else "COM3")
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ── feeder import ─────────────────────────────────────────────────────────────
try:
    from feeder import FeederCore, FeederStatus, load_config, DEFAULT_CONFIG, _HAS_YAML
except ImportError as _e:
    print(f"ERROR: cannot import feeder.py — {_e}", file=sys.stderr)
    sys.exit(1)

try:
    import yaml
    _HAS_YAML_MAIN = True
except ImportError:
    _HAS_YAML_MAIN = False

_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
_CONFIG_FILE = "config.yaml"


# ── helpers ───────────────────────────────────────────────────────────────────

def _default_exe() -> str:
    name = "engine-sim-headless.exe" if platform.system() == "Windows" else "engine-sim-headless"
    return str(_SCRIPT_DIR / "build" / name)


def _scan_engines(engines_dir: str) -> list:
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


def _save_config(cfg: dict, path: str = _CONFIG_FILE) -> None:
    if not _HAS_YAML_MAIN:
        return
    try:
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)
    except Exception as e:
        print(f"[ui] Could not save config: {e}", file=sys.stderr)


# ── App ───────────────────────────────────────────────────────────────────────

class App:
    POLL_MS = 250   # UI refresh interval

    def __init__(self, root: tk.Tk, config_path: str) -> None:
        self.root = root
        self.config_path = config_path
        self.cfg = load_config(config_path)

        self._eng_proc: "subprocess.Popen | None" = None
        self._feeder: "FeederCore | None" = None
        self._manual_mode = False   # True when manual test slider overrides CAN

        # Engine list
        engines_dir = str(_SCRIPT_DIR / "assets" / "engines")
        engines = _scan_engines(engines_dir)
        self._engine_map = dict(engines)
        self._engine_names = [e[0] for e in engines]

        self._build_ui()
        self._restore_config()
        self.root.after(self.POLL_MS, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root.title("Tesla Engine Sound Bridge")
        self.root.resizable(False, False)
        PAD = 10

        # ── header ────────────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg="#0d0d1e", padx=PAD, pady=8)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="Tesla Engine Sound Bridge",
                 fg="#e8e8ff", bg="#0d0d1e",
                 font=("TkDefaultFont", 13, "bold")).pack(side=tk.LEFT)

        # ── engine-sim panel ──────────────────────────────────────────
        ep = tk.LabelFrame(self.root, text=" Engine-Sim ", padx=PAD, pady=PAD)
        ep.pack(fill=tk.X, padx=PAD, pady=(PAD, 3))
        ep.columnconfigure(0, weight=1)

        # Row 0: exe path
        exe_row = tk.Frame(ep)
        exe_row.grid(row=0, column=0, columnspan=3, sticky="ew")
        tk.Label(exe_row, text="Executable:", width=11, anchor="w").pack(side=tk.LEFT)
        self.sv_exe = tk.StringVar(value=_default_exe())
        tk.Entry(exe_row, textvariable=self.sv_exe, width=38).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(exe_row, text="Browse", command=self._browse_exe, width=7).pack(side=tk.LEFT)

        # Row 1: engine script
        eng_row = tk.Frame(ep)
        eng_row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        tk.Label(eng_row, text="Engine:", width=11, anchor="w").pack(side=tk.LEFT)
        if self._engine_names:
            self.engine_combo = ttk.Combobox(eng_row, values=self._engine_names,
                                             state="readonly", width=36)
            self.engine_combo.set(self._engine_names[0])
        else:
            self.engine_combo = ttk.Combobox(eng_row, values=["(none found)"],
                                             state="disabled", width=36)
            self.engine_combo.set("(none found)")
        self.engine_combo.pack(side=tk.LEFT, padx=(0, 4))

        # Row 2: engine status + buttons
        eng_btn_row = tk.Frame(ep)
        eng_btn_row.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.sv_eng_status = tk.StringVar(value="○ Stopped")
        self._eng_status_lbl = tk.Label(eng_btn_row, textvariable=self.sv_eng_status,
                                         fg="#888888", font=("TkDefaultFont", 10, "bold"), width=14)
        self._eng_status_lbl.pack(side=tk.LEFT)
        tk.Button(eng_btn_row, text="Launch", command=self._launch_engine, width=9).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(eng_btn_row, text="Stop",   command=self._stop_engine,  width=9).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(eng_btn_row, text="Restart", command=self._restart_engine, width=9).pack(side=tk.LEFT)

        # ── CAN interface panel ───────────────────────────────────────
        cp = tk.LabelFrame(self.root, text=" CAN Interface ", padx=PAD, pady=PAD)
        cp.pack(fill=tk.X, padx=PAD, pady=3)

        # Row 0: interface type, channel, bitrate
        r0 = tk.Frame(cp)
        r0.pack(fill=tk.X)
        tk.Label(r0, text="Interface:", anchor="w", width=9).pack(side=tk.LEFT)
        self.sv_iface = tk.StringVar(value=_DEFAULT_IFACE)
        ttk.Combobox(r0, textvariable=self.sv_iface, width=10,
                     values=["slcan", "gs_usb", "pcan", "kvaser",
                             "socketcan", "cantact", "virtual"],
                     state="readonly").pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(r0, text="Channel:", anchor="w").pack(side=tk.LEFT)
        self.sv_channel = tk.StringVar(value=_DEFAULT_CHANNEL)
        tk.Entry(r0, textvariable=self.sv_channel, width=12).pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(r0, text="Bitrate:", anchor="w").pack(side=tk.LEFT)
        self.sv_bitrate = tk.StringVar(value="500000")
        tk.Entry(r0, textvariable=self.sv_bitrate, width=8).pack(side=tk.LEFT)

        # Row 1: DBC file
        r1 = tk.Frame(cp)
        r1.pack(fill=tk.X, pady=(6, 0))
        tk.Label(r1, text="DBC file:", anchor="w", width=9).pack(side=tk.LEFT)
        self.sv_dbc = tk.StringVar(value="Model3CAN.dbc")
        tk.Entry(r1, textvariable=self.sv_dbc, width=38).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(r1, text="Browse", command=self._browse_dbc, width=7).pack(side=tk.LEFT)

        # Row 2: feeder buttons + manual test toggle
        r2 = tk.Frame(cp)
        r2.pack(fill=tk.X, pady=(8, 0))
        self.sv_feeder_status = tk.StringVar(value="○ Stopped")
        self._feeder_status_lbl = tk.Label(r2, textvariable=self.sv_feeder_status,
                                            fg="#888888", font=("TkDefaultFont", 10, "bold"), width=14)
        self._feeder_status_lbl.pack(side=tk.LEFT)
        tk.Button(r2, text="Start Feeder",   command=self._start_feeder,   width=12).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(r2, text="Stop Feeder",    command=self._stop_feeder,    width=12).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(r2, text="Restart Feeder", command=self._restart_feeder, width=12).pack(side=tk.LEFT)

        # ── live status panel ─────────────────────────────────────────
        lp = tk.LabelFrame(self.root, text=" Live Status ", padx=PAD, pady=PAD)
        lp.pack(fill=tk.X, padx=PAD, pady=3)
        lp.columnconfigure(1, weight=1)
        lp.columnconfigure(3, weight=1)

        MONO = ("Courier", 11)
        self.sv_can_conn  = tk.StringVar(value="—")
        self.sv_fps       = tk.StringVar(value="—")
        self.sv_age       = tk.StringVar(value="—")
        self.sv_speed     = tk.StringVar(value="—")
        self.sv_pedal     = tk.StringVar(value="—")
        self.sv_brake_d   = tk.StringVar(value="—")
        self.sv_gear_d    = tk.StringVar(value="—")
        self.sv_failsafe  = tk.StringVar(value="—")
        self.sv_sent      = tk.StringVar(value="—")

        rows = [
            ("CAN",        self.sv_can_conn,  "Frames/s",  self.sv_fps),
            ("Last frame", self.sv_age,        "Failsafe",  self.sv_failsafe),
            ("Speed",      self.sv_speed,      "Sent cmds", self.sv_sent),
            ("Pedal",      self.sv_pedal,      "Gear",      self.sv_gear_d),
            ("Brake",      self.sv_brake_d,    "",          None),
        ]
        for i, (la, va, lb, vb) in enumerate(rows):
            tk.Label(lp, text=f"{la}:", width=11, anchor="w").grid(row=i, column=0, sticky="w", pady=2)
            tk.Label(lp, textvariable=va, font=MONO, width=14, anchor="w").grid(row=i, column=1, sticky="w")
            if lb:
                tk.Label(lp, text=f"{lb}:", width=11, anchor="w").grid(row=i, column=2, sticky="w", padx=(12, 0))
            if vb:
                tk.Label(lp, textvariable=vb, font=MONO, width=14, anchor="w").grid(row=i, column=3, sticky="w")

        # ── manual test panel ─────────────────────────────────────────
        mp = tk.LabelFrame(self.root, text=" Manual Test (override CAN) ", padx=PAD, pady=PAD)
        mp.pack(fill=tk.X, padx=PAD, pady=3)
        mp.columnconfigure(1, weight=1)

        self.manual_var = tk.BooleanVar(value=False)
        self.manual_var.trace_add("write", self._on_manual_toggle)
        tk.Checkbutton(mp, text="Enable manual test mode",
                       variable=self.manual_var).grid(row=0, column=0, columnspan=3, sticky="w")

        self.sv_m_speed = tk.DoubleVar(value=60.0)
        self.sv_m_pedal = tk.DoubleVar(value=40.0)
        self._m_speed_lbl = tk.Label(mp, text=" 60 mph", width=7)
        self._m_pedal_lbl = tk.Label(mp, text=" 40 %",   width=7)

        tk.Label(mp, text="Speed:", width=8, anchor="w").grid(row=1, column=0, sticky="w", pady=3)
        self._m_speed_scale = tk.Scale(mp, variable=self.sv_m_speed, from_=0, to=130,
                                       orient=tk.HORIZONTAL, length=260, showvalue=False,
                                       command=lambda v: self._m_speed_lbl.config(text=f"{float(v):4.0f} mph"))
        self._m_speed_scale.grid(row=1, column=1, sticky="ew", padx=4)
        self._m_speed_lbl.grid(row=1, column=2, sticky="w")

        tk.Label(mp, text="Pedal:", width=8, anchor="w").grid(row=2, column=0, sticky="w", pady=3)
        self._m_pedal_scale = tk.Scale(mp, variable=self.sv_m_pedal, from_=0, to=100,
                                       orient=tk.HORIZONTAL, length=260, showvalue=False,
                                       command=lambda v: self._m_pedal_lbl.config(text=f"{float(v):4.0f} %"))
        self._m_pedal_scale.grid(row=2, column=1, sticky="ew", padx=4)
        self._m_pedal_lbl.grid(row=2, column=2, sticky="w")

        self._set_manual_sliders_state(tk.DISABLED)

        # ── errors panel ──────────────────────────────────────────────
        erp = tk.LabelFrame(self.root, text=" Errors ", padx=PAD, pady=6)
        erp.pack(fill=tk.X, padx=PAD, pady=(3, PAD))
        self._err_text = tk.Text(erp, height=3, state=tk.DISABLED,
                                  wrap=tk.WORD, font=("Courier", 9),
                                  fg="#cc4444", bg=self.root.cget("bg"))
        self._err_text.pack(fill=tk.X)

    def _set_manual_sliders_state(self, state) -> None:
        self._m_speed_scale.config(state=state)
        self._m_pedal_scale.config(state=state)

    # ── config restore / save ─────────────────────────────────────────────────

    def _restore_config(self) -> None:
        can = self.cfg.get("can", {})
        self.sv_iface.set(can.get("interface", _DEFAULT_IFACE))
        self.sv_channel.set(can.get("channel",    _DEFAULT_CHANNEL))
        self.sv_bitrate.set(str(can.get("bitrate", 500000)))
        self.sv_dbc.set(can.get("dbc_path", "Model3CAN.dbc"))

        eng_sim = self.cfg.get("engine_sim", {})
        if "executable" in eng_sim:
            self.sv_exe.set(eng_sim["executable"])

        saved_script = eng_sim.get("engine_script", "")
        if saved_script and saved_script in self._engine_map:
            self.engine_combo.set(saved_script)
        elif self._engine_names:
            self.engine_combo.set(self._engine_names[0])

    def _gather_config(self) -> dict:
        """Build a config dict from current UI values."""
        import copy
        cfg = copy.deepcopy(self.cfg)
        cfg["can"]["interface"] = self.sv_iface.get()
        cfg["can"]["channel"]   = self.sv_channel.get()
        try:
            cfg["can"]["bitrate"] = int(self.sv_bitrate.get())
        except ValueError:
            pass
        cfg["can"]["dbc_path"]  = self.sv_dbc.get()
        cfg["engine_sim"]["executable"]    = self.sv_exe.get()
        cfg["engine_sim"]["engine_script"] = self.engine_combo.get()
        cfg["feeder"]["manual_test"]       = bool(self.manual_var.get())
        return cfg

    # ── browse buttons ────────────────────────────────────────────────────────

    def _browse_exe(self) -> None:
        path = filedialog.askopenfilename(
            title="Select engine-sim-headless",
            filetypes=[("Executable", "*.exe" if _SYS == "Windows" else "*"),
                       ("All", "*.*")])
        if path:
            self.sv_exe.set(path)

    def _browse_dbc(self) -> None:
        path = filedialog.askopenfilename(
            title="Select DBC file", filetypes=[("DBC files", "*.dbc"), ("All", "*.*")])
        if path:
            self.sv_dbc.set(path)

    # ── engine-sim process management ─────────────────────────────────────────

    def _launch_engine(self) -> None:
        exe = self.sv_exe.get()
        if not os.path.isfile(exe):
            self._show_error(f"Executable not found:\n{exe}\n\nBuild the project first.")
            return

        friendly = self.engine_combo.get()
        script = self._engine_map.get(friendly, "")
        if not script or not os.path.isfile(script):
            self._show_error(f"Engine script not found:\n{script}")
            return

        self._stop_engine()
        port = int(self.cfg.get("engine_sim", {}).get("port", 9999))
        cmd = [exe, "--script", script, "--port", str(port)]
        print(f"[ui] Launch engine-sim: {' '.join(cmd)}")
        try:
            self._eng_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=None)
        except OSError as e:
            self._show_error(f"Launch failed: {e}")
            return
        print(f"[ui] engine-sim PID {self._eng_proc.pid}")
        self._save_current_config()

    def _stop_engine(self) -> None:
        if self._eng_proc is not None:
            print(f"[ui] Stopping engine-sim PID {self._eng_proc.pid}")
            self._eng_proc.terminate()
            try:
                self._eng_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._eng_proc.kill()
                self._eng_proc.wait()
            self._eng_proc = None

    def _restart_engine(self) -> None:
        self._stop_engine()
        self._launch_engine()

    def _engine_running(self) -> bool:
        return self._eng_proc is not None and self._eng_proc.poll() is None

    # ── feeder management ─────────────────────────────────────────────────────

    def _start_feeder(self) -> None:
        if self._feeder is not None:
            self._stop_feeder()
        cfg = self._gather_config()
        port = int(cfg.get("engine_sim", {}).get("port", 9999))
        self._feeder = FeederCore(cfg, host="127.0.0.1", port=port)
        self._feeder.start()
        self._save_current_config()

    def _stop_feeder(self) -> None:
        if self._feeder is not None:
            self._feeder.stop()
            self._feeder = None

    def _restart_feeder(self) -> None:
        self._stop_feeder()
        self._start_feeder()

    # ── manual test ───────────────────────────────────────────────────────────

    def _on_manual_toggle(self, *_) -> None:
        enabled = self.manual_var.get()
        self._set_manual_sliders_state(tk.NORMAL if enabled else tk.DISABLED)
        self._manual_mode = enabled
        # If feeder is running, restart it in the new mode
        if self._feeder is not None:
            self._restart_feeder()

    # ── polling loop ──────────────────────────────────────────────────────────

    def _poll(self) -> None:
        # Engine-sim status
        if self._engine_running():
            self.sv_eng_status.set("● Running")
            self._eng_status_lbl.config(fg="#22cc44")
        else:
            self.sv_eng_status.set("○ Stopped")
            self._eng_status_lbl.config(fg="#888888")
            if self._eng_proc is not None:
                self._eng_proc = None

        # Feeder status
        if self._feeder is not None:
            s: FeederStatus = self._feeder.status

            # If manual mode is active, push slider values into feeder
            if self._manual_mode:
                self._feeder.set_manual_values(
                    self.sv_m_speed.get(),
                    self.sv_m_pedal.get(),
                    0.0)

            # CAN connection indicator
            if s.can_connected:
                self.sv_can_conn.set("● Connected")
                self.sv_feeder_status.set("● Running")
                self._feeder_status_lbl.config(fg="#22cc44")
            else:
                self.sv_can_conn.set("○ Not connected")
                self.sv_feeder_status.set("● Running (no CAN)")
                self._feeder_status_lbl.config(fg="#ffaa00")

            age_str = (f"{s.last_frame_age_sec:.2f} s ago"
                       if s.last_frame_age_sec is not None else "—")
            self.sv_fps.set(f"{s.frames_per_sec:.1f}")
            self.sv_age.set(age_str)
            self.sv_speed.set(f"{s.speed_mph:6.1f} mph")
            self.sv_pedal.set(f"{s.pedal_pct:6.1f} %")
            self.sv_brake_d.set(f"{s.brake:.3f}")
            self.sv_gear_d.set(s.gear_str)
            self.sv_failsafe.set("YES" if s.failsafe_active else "no")
            self.sv_sent.set(str(s.sent_commands))

            if s.error:
                self._show_error(s.error)
        else:
            self.sv_feeder_status.set("○ Stopped")
            self._feeder_status_lbl.config(fg="#888888")
            self.sv_can_conn.set("—")
            self.sv_fps.set("—")
            self.sv_age.set("—")
            self.sv_speed.set("—")
            self.sv_pedal.set("—")
            self.sv_brake_d.set("—")
            self.sv_gear_d.set("—")
            self.sv_failsafe.set("—")
            self.sv_sent.set("—")

        self.root.after(self.POLL_MS, self._poll)

    # ── error display ─────────────────────────────────────────────────────────

    def _show_error(self, msg: str) -> None:
        self._err_text.config(state=tk.NORMAL)
        self._err_text.delete("1.0", tk.END)
        self._err_text.insert(tk.END, msg)
        self._err_text.config(state=tk.DISABLED)

    # ── config save ───────────────────────────────────────────────────────────

    def _save_current_config(self) -> None:
        _save_config(self._gather_config(), self.config_path)

    # ── shutdown ──────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        self._save_current_config()
        self._stop_feeder()
        self._stop_engine()
        self.root.destroy()


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Tesla Engine Sound Bridge — launcher UI")
    parser.add_argument("--config", default=_CONFIG_FILE,
                        help=f"Config file path (default: {_CONFIG_FILE})")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.config)
    root.mainloop()


if __name__ == "__main__":
    main()
