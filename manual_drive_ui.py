"""
manual_drive_ui.py
------------------
Tkinter UI for manually testing engine-sim-headless sound tuning.
Move the sliders; speed is simulated with Tesla-style regen physics.

Usage:
    python manual_drive_ui.py
    python manual_drive_ui.py --host 192.168.1.5 --port 9999 --hz 40

Requires:
    Python 3.8+  (standard library only; no extra packages needed)
"""

import argparse
import socket
import time
import tkinter as tk


class App:
    """Main application: UI + physics loop + UDP output."""

    def __init__(self, root: tk.Tk, host: str, port: int, hz: float) -> None:
        self.root      = root
        self.addr      = (host, port)
        self.period_ms = max(10, round(1000.0 / hz))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Physics state
        self.speed_mph  = 0.0
        self.regen_mps2 = 0.0
        self.last_time  = time.monotonic()
        self.last_print = time.monotonic()

        self._build_ui(host, port, hz)
        self._startup()
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

        # ── sliders ───────────────────────────────────────────────────
        ctrl = tk.LabelFrame(self.root, text=" Controls ", padx=PAD, pady=PAD)
        ctrl.pack(fill=tk.X, padx=PAD, pady=(PAD, 4))
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

        display_rows = [
            ("Speed",  self.sv_speed, SPEED_FONT),
            ("Pedal",  self.sv_pedal, MONO),
            ("Brake",  self.sv_brake, MONO),
            ("Regen",  self.sv_regen, MONO),
        ]
        for i, (name, sv, fnt) in enumerate(display_rows):
            tk.Label(
                disp, text=f"{name}:", width=7, anchor="w",
            ).grid(row=i, column=0, sticky="w", pady=3)
            tk.Label(
                disp, textvariable=sv, width=18, anchor="e", font=fnt,
            ).grid(row=i, column=1, sticky="e")

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
        """Add one labelled horizontal slider row to the controls grid."""
        tk.Label(
            parent, text=label, width=12, anchor="w",
        ).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=5)

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
    # Main update loop  (runs via tkinter.after at the requested Hz)
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        now = time.monotonic()
        dt  = min(now - self.last_time, 0.2)    # cap to avoid large jump on stall
        self.last_time = now

        pedal_pct = self.pedal_var.get()
        brake_pct = self.brake_var.get()

        # ── physics ────────────────────────────────────────────────────
        accel_mps2 = pedal_pct / 100.0 * 8.5

        # Tesla-style regen: only when pedal fully released (< 5 %)
        if pedal_pct < 5.0:
            self.regen_mps2 = 3.5 * min(self.speed_mph / 30.0, 1.0)
        else:
            self.regen_mps2 = 0.0

        brake_mps2 = brake_pct / 100.0 * 9.0
        net_accel  = accel_mps2 - self.regen_mps2 - brake_mps2

        self.speed_mph += net_accel * dt * 2.23694   # m/s² * s = m/s; * 2.23694 = mph
        self.speed_mph  = max(0.0, min(130.0, self.speed_mph))

        # brake sent over UDP is brake_pct / 100 only — regen is NOT added
        brake_out = brake_pct / 100.0

        # ── UDP ────────────────────────────────────────────────────────
        self._send(f"speed {self.speed_mph:.2f}")
        self._send(f"pedal {pedal_pct:.1f}")
        self._send(f"brake {brake_out:.3f}")

        # ── update display labels ──────────────────────────────────────
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
        self.sock.close()
        self.root.destroy()


# ----------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Manual drive UI for engine-sim-headless sound tuning"
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="headless UDP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9999,
                        help="UDP port (default: 9999)")
    parser.add_argument("--hz",   type=float, default=20.0,
                        help="update rate in Hz (default: 20)")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.host, args.port, args.hz)
    root.mainloop()


if __name__ == "__main__":
    main()
