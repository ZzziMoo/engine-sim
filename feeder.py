#!/usr/bin/env python3
"""
feeder.py
---------
Real-time CAN-to-engine-sim bridge for Tesla Model 3 / EV sound synthesis.

Reads live CAN frames via python-can, decodes signals using a DBC file
(cantools), translates vehicle state to engine-sim UDP control commands.

Standalone:
    python feeder.py                          # load config.yaml, live CAN
    python feeder.py --manual-test            # ramp profile, no hardware
    python feeder.py --manual-test --mode sine|hold|ramp
    python feeder.py --channel can0 --interface socketcan
    python feeder.py --config custom.yaml

Library (from main.py):
    from feeder import FeederCore, load_config
    core = FeederCore(cfg, host="127.0.0.1", port=9999)
    core.start()
    status = core.status        # FeederStatus snapshot
    core.stop()

Gear -> engine-sim mapping
    D  -> mode hybrid  + virtual_gears on
    R  -> mode throttle (reverse; just pedal pass-through)
    P/N-> pedal forced to 0

CAN failsafe: if no new frames for can_timeout_sec, throttle/pedal -> 0.

Dependencies: python-can  cantools  pyyaml
"""

import argparse
import copy
import logging
import logging.handlers
import math
import os
import pathlib
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

# ── optional imports ──────────────────────────────────────────────────────────
try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent

# ── default configuration ─────────────────────────────────────────────────────
DEFAULT_CONFIG: Dict[str, Any] = {
    "can": {
        "interface": "socketcan",   # socketcan | slcan | pcan | kvaser | virtual
        "channel": "can0",          # can0, /dev/ttyUSB0, PCAN_USBBUS1 …
        "bitrate": 500000,
        "dbc_path": "Model3CAN.dbc",
    },
    # Map roles to DBC signal names.  Change here if your DBC differs.
    "signal_map": {
        "vehicle_speed": "DI_vehicleSpeed",    # kph  (message 0x257)
        "throttle_pedal": "DI_accelPedalPos",  # 0-100 %  (message 0x118)
        "brake_pedal": "DI_brakePedalState",   # enum OFF/ON  (message 0x118)
        "gear": "DI_gear",                      # enum P/R/N/D  (message 0x118)
    },
    # Optional: continuous-valued brake signal overrides brake_pedal boolean.
    # "analog_brake": {
    #     "signal": "IBST_sInputRodDriver",   # 0-47 mm
    #     "max_mm": 20.0,                     # mm that maps to brake=1.0
    # },
    "engine_sim": {
        "port": 9999,
        "startup_mode": "hybrid",   # throttle | hybrid
        "virtual_gears": True,
    },
    "feeder": {
        "update_hz": 20.0,
        "can_timeout_sec": 2.0,     # failsafe threshold
        "manual_test": False,
        "log_level": "INFO",
    },
    "manual_test": {
        "mode": "ramp",         # ramp | sine | hold
        "speed_mph": 60.0,
        "pedal_pct": 40.0,
        "brake": 0.0,
    },
}


def load_config(path: str = "config.yaml") -> Dict[str, Any]:
    """Load config.yaml and deep-merge with DEFAULT_CONFIG."""
    import copy
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if not _HAS_YAML:
        return cfg
    try:
        with open(path, encoding="utf-8") as f:
            user = yaml.safe_load(f) or {}
        for section, values in user.items():
            if isinstance(values, dict) and section in cfg and isinstance(cfg[section], dict):
                cfg[section].update(values)
            else:
                cfg[section] = values
    except FileNotFoundError:
        pass
    except Exception as e:
        logging.getLogger("feeder").warning(f"Could not load {path}: {e}")
    return cfg


# ── status dataclass ──────────────────────────────────────────────────────────
@dataclass
class FeederStatus:
    # CAN layer
    can_connected: bool = False
    frames_per_sec: float = 0.0
    last_frame_age_sec: Optional[float] = None  # seconds since last frame

    # Decoded vehicle state
    speed_mph: float = 0.0
    pedal_pct: float = 0.0
    brake: float = 0.0
    gear_int: int = 0           # 0=unknown 1=P 2=R 3=N 4=D
    gear_str: str = "—"

    # Operational
    failsafe_active: bool = False
    sent_commands: int = 0
    running: bool = False
    error: Optional[str] = None

    # Debug: last raw DBC signal values for the mapped signals
    raw_signals: Dict[str, str] = field(default_factory=dict)


# ── helpers ───────────────────────────────────────────────────────────────────
_GEAR_MAP = {0: "—", 1: "P", 2: "R", 3: "N", 4: "D"}


def _gear_str(g: int) -> str:
    return _GEAR_MAP.get(g, "—")


def _normalize_brake(raw) -> float:
    """Decode a DBC brake signal (enum string, int, or bool) to 0.0 / 1.0."""
    if isinstance(raw, bool):
        return 1.0 if raw else 0.0
    if isinstance(raw, str):
        return 1.0 if raw.upper() in ("ON", "1", "TRUE", "APPLIED",
                                       "DRIVER_APPLYING_BRAKES") else 0.0
    try:
        return 1.0 if int(raw) == 1 else 0.0
    except (TypeError, ValueError):
        return 0.0


def _normalize_gear(raw) -> int:
    """Map DBC gear value (enum string or int) to 1=P 2=R 3=N 4=D 0=unknown."""
    s = str(raw).upper()
    if "DI_GEAR_D" in s or s == "4":  return 4
    if "DI_GEAR_N" in s or s == "3":  return 3
    if "DI_GEAR_R" in s or s == "2":  return 2
    if "DI_GEAR_P" in s or s == "1":  return 1
    try:
        v = int(float(raw))
        if 1 <= v <= 4:
            return v
    except (TypeError, ValueError):
        pass
    return 0


def _setup_logging(level_str: str, log_dir: str = "logs") -> logging.Logger:
    pathlib.Path(log_dir).mkdir(exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_file = str(pathlib.Path(log_dir) / f"feeder_{ts}.log")

    level = getattr(logging, level_str.upper(), logging.INFO)
    logger = logging.getLogger("feeder")
    logger.setLevel(level)
    if logger.handlers:
        return logger   # already configured

    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                            datefmt="%H:%M:%S")
    fh = logging.handlers.RotatingFileHandler(
        log_file, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8")
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    ch = logging.StreamHandler(sys.stderr)
    ch.setFormatter(fmt)
    logger.addHandler(ch)
    logger.info(f"Log file: {log_file}")
    return logger


# ── FeederCore ────────────────────────────────────────────────────────────────
class FeederCore:
    """
    All feeder logic: CAN/manual reader, translator, UDP sender.
    Thread-safe. Call start() then stop().
    """

    def __init__(self, config: Dict[str, Any],
                 host: str = "127.0.0.1", port: int = 9999):
        self._cfg = config
        self._addr = (host, port)

        log_level = config.get("feeder", {}).get("log_level", "INFO")
        self._log = _setup_logging(log_level)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Shared state (reader writes, sender reads) — protected by _state_lock
        self._state_lock = threading.Lock()
        self._speed_mph: float = 0.0
        self._pedal_pct: float = 0.0
        self._brake: float = 0.0
        self._gear_int: int = 0
        self._last_data_time: Optional[float] = None
        self._raw_sigs: Dict[str, str] = {}

        # Frame-rate tracking (atomic via GIL)
        self._frame_count: int = 0
        self._fps: float = 0.0

        # Status (lock-protected, read by UI)
        self._status_lock = threading.Lock()
        self._status = FeederStatus()

        self._stop_event = threading.Event()
        self._threads: list = []

        # DBC
        self._db = None
        self._target_ids: frozenset = frozenset()

    # ── DBC loading ───────────────────────────────────────────────────────────

    def _load_dbc(self, path: str) -> bool:
        try:
            import cantools
        except ImportError:
            self._log.error("cantools not installed — pip install cantools")
            return False
        try:
            self._db = cantools.database.load_file(path)
        except FileNotFoundError:
            self._log.error(f"DBC file not found: {path}")
            return False
        except Exception as e:
            self._log.error(f"Failed to load DBC {path}: {e}")
            return False

        # Build the set of message IDs we actually need
        sig_map = self._cfg.get("signal_map", DEFAULT_CONFIG["signal_map"])
        ids: set = set()
        for role, sig_name in sig_map.items():
            for msg in self._db.messages:
                if any(s.name == sig_name for s in msg.signals):
                    ids.add(msg.frame_id)
        # Also include analog_brake message if configured
        ab = self._cfg.get("analog_brake")
        if ab:
            for msg in self._db.messages:
                if any(s.name == ab["signal"] for s in msg.signals):
                    ids.add(msg.frame_id)
        self._target_ids = frozenset(ids)
        self._log.info(
            f"DBC loaded: {path}  "
            f"watching {len(self._target_ids)} msg IDs: "
            f"{sorted(hex(i) for i in self._target_ids)}")
        return True

    # ── UDP sending ───────────────────────────────────────────────────────────

    def _send(self, cmd: str) -> None:
        try:
            self._sock.sendto((cmd + "\n").encode(), self._addr)
            with self._status_lock:
                self._status.sent_commands += 1
        except OSError:
            pass

    # ── CAN reader thread ─────────────────────────────────────────────────────

    def _can_reader_thread(self) -> None:
        try:
            import can
        except ImportError:
            err = "python-can not installed — pip install python-can"
            self._log.error(err)
            with self._status_lock:
                self._status.error = err
            return

        can_cfg = self._cfg.get("can", {})
        bustype = can_cfg.get("interface", "socketcan")
        channel = can_cfg.get("channel", "can0")
        bitrate = int(can_cfg.get("bitrate", 500000))

        self._log.info(f"Opening CAN: bustype={bustype} channel={channel} bitrate={bitrate}")
        with self._status_lock:
            self._status.can_connected = False

        try:
            bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        except Exception as e:
            err = f"Cannot open CAN bus: {e}"
            self._log.error(err)
            with self._status_lock:
                self._status.error = err
            return

        with self._status_lock:
            self._status.can_connected = True
            self._status.error = None
        self._log.info("CAN bus open")

        sig_map = self._cfg.get("signal_map", DEFAULT_CONFIG["signal_map"])
        speed_sig  = sig_map.get("vehicle_speed",  "DI_vehicleSpeed")
        pedal_sig  = sig_map.get("throttle_pedal", "DI_accelPedalPos")
        brake_sig  = sig_map.get("brake_pedal",    "DI_brakePedalState")
        gear_sig   = sig_map.get("gear",           "DI_gear")
        ab_cfg     = self._cfg.get("analog_brake")
        ab_sig     = ab_cfg["signal"] if ab_cfg else None
        ab_max     = float(ab_cfg.get("max_mm", 20.0)) if ab_cfg else 20.0

        warned_ids: set = set()

        try:
            while not self._stop_event.is_set():
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if self._target_ids and msg.arbitration_id not in self._target_ids:
                    continue

                self._frame_count += 1   # GIL-safe int increment

                if self._db is None:
                    with self._state_lock:
                        self._last_data_time = time.monotonic()
                    continue

                try:
                    sigs = self._db.decode_message(msg.arbitration_id, msg.data)
                except Exception:
                    if msg.arbitration_id not in warned_ids:
                        warned_ids.add(msg.arbitration_id)
                        self._log.warning(
                            f"Cannot decode 0x{msg.arbitration_id:03X} "
                            f"(not in DBC or format mismatch)")
                    continue

                now = time.monotonic()
                raw_update = {}

                with self._state_lock:
                    self._last_data_time = now

                    if speed_sig in sigs:
                        kph = float(sigs[speed_sig])
                        if -10.0 <= kph <= 350.0:   # sanity: discard SNA (4095*0.08-40=287 ok)
                            self._speed_mph = max(0.0, kph * 0.621371)
                        raw_update[speed_sig] = f"{kph:.1f} kph"

                    if pedal_sig in sigs:
                        v = float(sigs[pedal_sig])
                        if 0.0 <= v <= 102.0:        # 255*0.4=102 is SNA sentinel
                            self._pedal_pct = max(0.0, min(100.0, v))
                        raw_update[pedal_sig] = f"{v:.1f}%"

                    if ab_sig and ab_sig in sigs:
                        mm = float(sigs[ab_sig])
                        self._brake = max(0.0, min(1.0, mm / ab_max))
                        raw_update[ab_sig] = f"{mm:.2f}mm"
                    elif brake_sig in sigs:
                        self._brake = _normalize_brake(sigs[brake_sig])
                        raw_update[brake_sig] = str(sigs[brake_sig])

                    if gear_sig in sigs:
                        self._gear_int = _normalize_gear(sigs[gear_sig])
                        raw_update[gear_sig] = str(sigs[gear_sig])

                    self._raw_sigs.update(raw_update)

                if raw_update:
                    self._log.debug(f"CAN 0x{msg.arbitration_id:03X}: {raw_update}")

        except Exception as e:
            self._log.error(f"CAN reader exception: {e}")
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass
            with self._status_lock:
                self._status.can_connected = False
            self._log.info("CAN bus closed")

    # ── manual test thread ────────────────────────────────────────────────────

    def _manual_test_thread(self) -> None:
        mt  = self._cfg.get("manual_test", {})
        mode = mt.get("mode", "ramp")
        h_speed  = float(mt.get("speed_mph", 60.0))
        h_pedal  = float(mt.get("pedal_pct", 40.0))
        h_brake  = float(mt.get("brake",     0.0))

        self._log.info(f"Manual test mode: {mode}")
        t0 = time.monotonic()

        while not self._stop_event.is_set():
            t = time.monotonic() - t0

            if mode == "hold":
                speed, pedal, brake = h_speed, h_pedal, h_brake

            elif mode == "sine":
                speed = max(0.0, 60.0 + 40.0 * math.sin(t * 0.25))
                pedal = max(0.0, 50.0 + 45.0 * math.sin(t * 0.35))
                brake = 0.0

            else:   # ramp  0->100 mph in 10 s, cruise 20 s, brake to 0, repeat
                cyc = t % 55.0
                if cyc < 10.0:
                    speed = cyc / 10.0 * 100.0
                    pedal, brake = 90.0, 0.0
                elif cyc < 30.0:
                    speed = 100.0
                    pedal, brake = 20.0, 0.0
                elif cyc < 35.0:
                    frac  = (cyc - 30.0) / 5.0
                    speed = 100.0 * (1.0 - frac)
                    pedal, brake = 0.0, 0.8
                else:
                    speed, pedal, brake = 0.0, 0.0, 0.0

            with self._state_lock:
                self._speed_mph     = speed
                self._pedal_pct     = pedal
                self._brake         = brake
                self._gear_int      = 4   # D
                self._last_data_time = time.monotonic()

            with self._status_lock:
                self._status.can_connected = True   # "connected" in manual mode

            self._stop_event.wait(0.05)   # 20 Hz update

    # ── sender thread ─────────────────────────────────────────────────────────

    def _sender_thread(self) -> None:
        hz          = float(self._cfg.get("feeder", {}).get("update_hz", 20.0))
        timeout_sec = float(self._cfg.get("feeder", {}).get("can_timeout_sec", 2.0))
        period      = 1.0 / max(1.0, hz)

        eng_cfg     = self._cfg.get("engine_sim", {})
        mode        = eng_cfg.get("startup_mode", "hybrid")
        vg          = eng_cfg.get("virtual_gears", True)

        # Startup sequence — wait briefly for engine-sim to be ready
        self._stop_event.wait(0.8)
        startup_cmds = [
            "mode hybrid" if mode == "hybrid" else "mode throttle",
            "virtual_gears on" if vg else "virtual_gears off",
            "vgear 0",
            "backfire off",
        ]
        for cmd in startup_cmds:
            self._send(cmd)
        self._log.info(f"Startup: mode={mode} virtual_gears={'on' if vg else 'off'}")

        prev_gear = -1
        t_fps_reset = time.monotonic()
        frame_snap  = 0

        while not self._stop_event.is_set():
            t_frame = time.monotonic()

            # Update FPS every second
            if t_frame - t_fps_reset >= 1.0:
                cnt = self._frame_count
                self._frame_count = 0
                self._fps = cnt / (t_frame - t_fps_reset)
                t_fps_reset = t_frame

            with self._state_lock:
                speed    = self._speed_mph
                pedal    = self._pedal_pct
                brake    = self._brake
                gear     = self._gear_int
                last_t   = self._last_data_time
                raw_copy = dict(self._raw_sigs)

            # Failsafe
            failsafe = False
            age: Optional[float] = None
            if last_t is None:
                failsafe = True
            else:
                age = time.monotonic() - last_t
                if age > timeout_sec:
                    failsafe = True

            if failsafe:
                pedal = 0.0
                brake = 0.0

            # Gear change handling
            if gear != prev_gear and gear > 0:
                prev_gear = gear
                if gear == 4:       # D — normal hybrid drive
                    self._send("mode hybrid")
                    self._send("virtual_gears on")
                elif gear == 2:     # R — reverse; pass pedal as throttle, no virtual gears
                    self._send("mode throttle")
                    self._send("virtual_gears off")
                elif gear in (1, 3):  # P / N — neutral, kill pedal
                    self._send("pedal 0")
                self._log.info(f"Gear change: {_gear_str(gear)}")

            # Send control state
            self._send(f"speed {speed:.2f}")
            self._send(f"pedal {pedal:.1f}")
            self._send(f"brake {brake:.3f}")

            self._log.debug(
                f"speed={speed:.1f}mph pedal={pedal:.1f}% brake={brake:.3f} "
                f"gear={_gear_str(gear)} failsafe={failsafe}")

            # Update public status
            with self._status_lock:
                s = self._status
                s.speed_mph        = speed
                s.pedal_pct        = pedal
                s.brake            = brake
                s.gear_int         = gear
                s.gear_str         = _gear_str(gear)
                s.failsafe_active  = failsafe
                s.frames_per_sec   = self._fps
                s.last_frame_age_sec = age
                s.raw_signals      = raw_copy

            sleep = t_frame + period - time.monotonic()
            if sleep > 0:
                self._stop_event.wait(sleep)

    # ── public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._stop_event.clear()
        with self._status_lock:
            self._status.running = True
            self._status.error   = None

        is_manual = self._cfg.get("feeder", {}).get("manual_test", False)

        if is_manual:
            t = threading.Thread(target=self._manual_test_thread,
                                 name="manual-test", daemon=True)
        else:
            dbc_path = self._cfg.get("can", {}).get("dbc_path", "Model3CAN.dbc")
            self._load_dbc(str(_SCRIPT_DIR / dbc_path)
                           if not os.path.isabs(dbc_path) else dbc_path)
            t = threading.Thread(target=self._can_reader_thread,
                                 name="can-reader", daemon=True)

        sender = threading.Thread(target=self._sender_thread,
                                  name="udp-sender", daemon=True)
        self._threads = [t, sender]
        t.start()
        sender.start()
        self._log.info(f"Feeder started (manual={is_manual}) -> {self._addr}")

    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=3.0)
        self._threads.clear()
        try:
            self._sock.close()
        except OSError:
            pass
        with self._status_lock:
            self._status.running = False
        self._log.info("Feeder stopped")

    @property
    def status(self) -> FeederStatus:
        """Thread-safe snapshot of the current feeder status."""
        with self._status_lock:
            return copy.copy(self._status)

    def set_manual_values(self, speed_mph: float, pedal_pct: float,
                          brake: float) -> None:
        """Inject values from the UI manual test sliders (thread-safe)."""
        with self._state_lock:
            self._speed_mph     = float(speed_mph)
            self._pedal_pct     = float(pedal_pct)
            self._brake         = float(brake)
            self._gear_int      = 4
            self._last_data_time = time.monotonic()


# ── standalone CLI ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="CAN-to-engine-sim bridge (Tesla Model 3)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config",    default="config.yaml",
                        help="YAML config file path")
    parser.add_argument("--host",      default=None,
                        help="engine-sim UDP host (overrides config)")
    parser.add_argument("--port",      type=int, default=None,
                        help="engine-sim UDP port (overrides config)")
    parser.add_argument("--manual-test", action="store_true",
                        help="Use generated data instead of CAN hardware")
    parser.add_argument("--mode",      choices=["ramp", "sine", "hold"],
                        default=None, help="Manual test drive profile")
    parser.add_argument("--interface", default=None,
                        help="CAN interface type (socketcan, slcan, pcan, kvaser, virtual)")
    parser.add_argument("--channel",   default=None,
                        help="CAN channel (can0, /dev/ttyUSB0, …)")
    parser.add_argument("--bitrate",   type=int, default=None,
                        help="CAN bitrate in bps")
    parser.add_argument("--dbc",       default=None,
                        help="DBC file path")
    parser.add_argument("--hz",        type=float, default=None,
                        help="Send rate to engine-sim (Hz)")
    parser.add_argument("--timeout",   type=float, default=None,
                        help="CAN failsafe timeout (seconds)")
    parser.add_argument("--log-level", default=None,
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    cfg = load_config(args.config)

    # CLI overrides
    if args.manual_test:
        cfg["feeder"]["manual_test"] = True
    if args.mode:
        cfg["manual_test"]["mode"] = args.mode
    if args.interface:
        cfg["can"]["interface"] = args.interface
    if args.channel:
        cfg["can"]["channel"] = args.channel
    if args.bitrate:
        cfg["can"]["bitrate"] = args.bitrate
    if args.dbc:
        cfg["can"]["dbc_path"] = args.dbc
    if args.hz:
        cfg["feeder"]["update_hz"] = args.hz
    if args.timeout:
        cfg["feeder"]["can_timeout_sec"] = args.timeout
    if args.log_level:
        cfg["feeder"]["log_level"] = args.log_level

    host = args.host or "127.0.0.1"
    port = args.port or cfg.get("engine_sim", {}).get("port", 9999)

    core = FeederCore(cfg, host=host, port=port)
    core.start()

    print(f"[feeder] Running  ->  {host}:{port}  "
          f"({'manual-test' if cfg['feeder']['manual_test'] else 'CAN'})")
    print("  Ctrl-C to stop\n")
    print(f"  {'age':>6}  {'speed':>8}  {'pedal':>7}  {'brake':>6}  "
          f"{'gear':>4}  {'fps':>6}  {'fs':>3}")
    print("  " + "-" * 52)

    t_print = time.monotonic()
    try:
        while True:
            time.sleep(0.1)
            if time.monotonic() - t_print < 0.5:
                continue
            t_print = time.monotonic()

            s = core.status
            age = f"{s.last_frame_age_sec:.2f}s" if s.last_frame_age_sec is not None else " —"
            print(f"  {age:>6}  {s.speed_mph:8.1f}  {s.pedal_pct:7.1f}  "
                  f"{s.brake:6.3f}  {s.gear_str:>4}  {s.frames_per_sec:6.1f}  "
                  f"{'YES' if s.failsafe_active else ' no':>3}")

    except KeyboardInterrupt:
        print("\n[feeder] stopping…")
    finally:
        core.stop()


if __name__ == "__main__":
    main()
