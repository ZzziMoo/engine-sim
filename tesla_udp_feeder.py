"""
tesla_udp_feeder.py
-------------------
Feeds speed / pedal / brake state to engine-sim-headless over UDP.

Usage:
    python tesla_udp_feeder.py --mode tesla3p          # simulated Model 3 P run
    python tesla_udp_feeder.py --mode manual            # edit MANUAL_* below
    python tesla_udp_feeder.py --mode can \\
        --dbc Model3CAN.dbc --channel can0 --bustype socketcan

    python tesla_udp_feeder.py --host 192.168.1.5 --port 9999 --hz 50 --mode tesla3p

CAN mode requirements:
    pip install python-can cantools

engine-sim-headless must be running with:
    engine-sim-headless --script <engine.mr> [--port 9999]
"""

import argparse
import socket
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Manual-mode inputs  (only used when --mode manual)
# Edit these before running, or modify them while the script is paused.
# ---------------------------------------------------------------------------
MANUAL_SPEED_MPH  = 60.0    # 0–130 mph
MANUAL_PEDAL_PCT  = 40.0    # 0–100 %
MANUAL_BRAKE      = 0.0     # 0–1

# ---------------------------------------------------------------------------
# Tesla Model 3 Performance keyframe table
# (matches the keyframes baked into main_headless.cpp drive_test tesla3p)
#
#  t(s)   speed(mph)  pedal(0-100)  brake(0-1)
# ---------------------------------------------------------------------------
TESLA3P_KF = [
    ( 0.00,   0.0,  100.0,  0.0),   # launch
    ( 0.50,  10.0,  100.0,  0.0),
    ( 1.00,  22.0,  100.0,  0.0),
    ( 1.50,  35.0,  100.0,  0.0),
    ( 2.00,  48.0,  100.0,  0.0),
    ( 2.50,  57.0,  100.0,  0.0),
    ( 2.90,  60.0,  100.0,  0.0),
    ( 3.50,  70.0,  100.0,  0.0),
    ( 4.50,  82.0,  100.0,  0.0),
    ( 5.50,  92.0,  100.0,  0.0),
    ( 6.70, 100.0,  100.0,  0.0),   # peak speed, full throttle
    ( 6.72, 100.0,    0.0,  0.0),   # lift-off -> backfire trigger (prevPedal=100->0)
    ( 6.90, 100.0,   35.0,  0.0),   # regen-coast
    ( 8.00, 100.0,   35.0,  0.0),
    ( 8.02, 100.0,   15.0,  0.0),   # light cruise
    (15.00, 100.0,   15.0,  0.0),
    (15.02, 100.0,    0.0,  0.0),   # full lift-off from cruise
    (17.00, 100.0,    0.0,  0.0),
    (17.02, 100.0,    0.0,  0.2),   # braking begins
    (22.00,  50.0,    0.0,  0.2),
    (22.02,  50.0,    0.0,  0.6),   # harder brake
    (25.00,   0.0,    0.0,  0.6),   # stopped
    (25.02,   0.0,    0.0,  0.0),   # release brake
    (27.00,   0.0,    0.0,  0.0),   # hold
]
TESLA3P_DURATION = 27.0


# ---------------------------------------------------------------------------
# CAN signal mapping  (Model 3 / Model Y)
#
#   Message 0x257 (599):
#     DI_vehicleSpeed   kph  -> speed_mph = value * 0.621371
#
#   Message 0x118 (280):
#     DI_accelPedalPos  0–100 % -> pedal_pct
#     DI_brakePedalState bool/enum -> brake = 1.0 if ON else 0.0
# ---------------------------------------------------------------------------

# IDs we care about (set for O(1) lookup in the hot path)
_CAN_TARGET_IDS = frozenset({0x118, 0x257})


def _brake_value(raw) -> float:
    """Normalise DBC-decoded brake signal to 0.0 / 1.0."""
    if isinstance(raw, bool):
        return 1.0 if raw else 0.0
    if isinstance(raw, (int, float)):
        return 1.0 if raw else 0.0
    if isinstance(raw, str):
        return 1.0 if raw.lower() in ("on", "active", "1", "true", "applied") else 0.0
    return 0.0


class CanState:
    """Thread-safe store for the latest decoded CAN values."""

    def __init__(self):
        self._lock      = threading.Lock()
        self._speed_mph = 0.0
        self._pedal_pct = 0.0
        self._brake     = 0.0
        # Track which signals have been received at least once
        self._got_speed = False
        self._got_pedal = False
        self._got_brake = False

    def update_speed(self, kph: float) -> None:
        with self._lock:
            self._speed_mph = kph * 0.621371
            self._got_speed = True

    def update_pedal(self, pct: float) -> None:
        with self._lock:
            self._pedal_pct = max(0.0, min(100.0, float(pct)))
            self._got_pedal = True

    def update_brake(self, raw) -> None:
        with self._lock:
            self._brake     = _brake_value(raw)
            self._got_brake = True

    def get(self) -> tuple:
        """Return (speed_mph, pedal_pct, brake) — safe defaults until first update."""
        with self._lock:
            return self._speed_mph, self._pedal_pct, self._brake

    def received_flags(self) -> tuple:
        """Return (got_speed, got_pedal, got_brake) for startup diagnostics."""
        with self._lock:
            return self._got_speed, self._got_pedal, self._got_brake


def _can_reader(bus, db, state: CanState) -> None:
    """
    Background thread: read CAN frames, decode via DBC, update CanState.
    Never raises — any error is swallowed so the thread keeps running.
    """
    warned_ids: set = set()   # suppress repeated decode-error prints per ID

    while True:
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id not in _CAN_TARGET_IDS:
                continue

            try:
                signals = db.decode_message(msg.arbitration_id, msg.data)
            except Exception:
                if msg.arbitration_id not in warned_ids:
                    warned_ids.add(msg.arbitration_id)
                    print(f"[CAN] warning: could not decode 0x{msg.arbitration_id:03X} "
                          f"(check DBC has this message)", file=sys.stderr)
                continue

            if "DI_vehicleSpeed" in signals:
                state.update_speed(float(signals["DI_vehicleSpeed"]))
            if "DI_accelPedalPos" in signals:
                state.update_pedal(float(signals["DI_accelPedalPos"]))
            if "DI_brakePedalState" in signals:
                state.update_brake(signals["DI_brakePedalState"])

        except Exception:
            pass  # never let reader crash; bus errors are transient


def interp_keyframes(t: float, keyframes: list) -> tuple:
    """Linear interpolation over a keyframe list of (t, speed, pedal, brake)."""
    if t <= keyframes[0][0]:
        return keyframes[0][1], keyframes[0][2], keyframes[0][3]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1], keyframes[-1][2], keyframes[-1][3]
    for i in range(len(keyframes) - 1):
        t0, s0, p0, b0 = keyframes[i]
        t1, s1, p1, b1 = keyframes[i + 1]
        if t0 <= t < t1:
            span = t1 - t0
            frac = (t - t0) / span if span > 1e-9 else 1.0
            return (
                s0 + frac * (s1 - s0),
                p0 + frac * (p1 - p0),
                b0 + frac * (b1 - b0),
            )
    return keyframes[-1][1], keyframes[-1][2], keyframes[-1][3]


def send_cmd(sock: socket.socket, addr: tuple, cmd: str) -> None:
    """Send a single newline-terminated command string over UDP."""
    sock.sendto((cmd + "\n").encode(), addr)


def send_vehicle_state(
    sock: socket.socket,
    addr: tuple,
    speed_mph: float,
    pedal_pct: float,
    brake: float,
) -> None:
    """Send the three vehicle-state commands in one burst."""
    send_cmd(sock, addr, f"speed {speed_mph:.2f}")
    send_cmd(sock, addr, f"pedal {pedal_pct:.1f}")
    send_cmd(sock, addr, f"brake {brake:.3f}")


def startup_sequence(sock: socket.socket, addr: tuple) -> None:
    """Send init commands to put engine-sim into hybrid EV mode."""
    for cmd in (
        "mode hybrid",
        "virtual_gears on",
        "vgear 0",          # auto virtual gear
        "backfire off",
    ):
        send_cmd(sock, addr, cmd)
        time.sleep(0.02)    # small gap so headless processes each line
    print(f"[feeder] Init sent to {addr[0]}:{addr[1]}")


def run_tesla3p(sock: socket.socket, addr: tuple, hz: float) -> None:
    """Replay the Model 3 Performance launch/cruise/brake profile."""
    period   = 1.0 / hz
    print_interval = 0.5
    t_start  = time.monotonic()
    t_print  = t_start

    print("[feeder] tesla3p: 27 s profile starting")
    print(f"         {hz:.0f} Hz update, print every {print_interval:.1f} s")
    print(f"  {'t':>6}  {'speed':>7}  {'pedal':>7}  {'brake':>6}")
    print("  " + "-" * 33)

    while True:
        t_frame = time.monotonic()
        elapsed = t_frame - t_start

        speed, pedal, brake = interp_keyframes(elapsed, TESLA3P_KF)
        send_vehicle_state(sock, addr, speed, pedal, brake)

        if t_frame - t_print >= print_interval:
            t_print = t_frame
            print(f"  {elapsed:6.1f}  {speed:7.1f}  {pedal:7.1f}  {brake:6.3f}")

        if elapsed >= TESLA3P_DURATION:
            print(f"\n[feeder] tesla3p complete ({elapsed:.1f} s)")
            break

        sleep_until = t_frame + period
        now = time.monotonic()
        if sleep_until > now:
            time.sleep(sleep_until - now)


def run_manual(sock: socket.socket, addr: tuple, hz: float) -> None:
    """Send the MANUAL_* constants at the configured rate until Ctrl-C."""
    period       = 1.0 / hz
    print_interval = 0.5
    t_print      = time.monotonic()

    print("[feeder] manual mode")
    print(f"  speed={MANUAL_SPEED_MPH} mph  pedal={MANUAL_PEDAL_PCT}%  brake={MANUAL_BRAKE}")
    print("  Ctrl-C to stop\n")

    try:
        while True:
            t_frame = time.monotonic()
            send_vehicle_state(sock, addr,
                               MANUAL_SPEED_MPH,
                               MANUAL_PEDAL_PCT,
                               MANUAL_BRAKE)

            if t_frame - t_print >= print_interval:
                t_print = t_frame
                print(f"  speed={MANUAL_SPEED_MPH:.1f}  "
                      f"pedal={MANUAL_PEDAL_PCT:.1f}  "
                      f"brake={MANUAL_BRAKE:.3f}")

            sleep_until = t_frame + period
            now = time.monotonic()
            if sleep_until > now:
                time.sleep(sleep_until - now)
    except KeyboardInterrupt:
        print("\n[feeder] stopped")


def run_can(
    sock: socket.socket,
    addr: tuple,
    hz: float,
    dbc_path: str,
    channel: str,
    bustype: str,
) -> None:
    """Read live CAN data and forward to engine-sim-headless at a fixed rate."""
    try:
        import can
        import cantools
    except ImportError as exc:
        sys.exit(
            "[feeder] CAN mode requires python-can and cantools.\n"
            "  pip install python-can cantools\n"
            f"  ({exc})"
        )

    # Load DBC
    print(f"[feeder] Loading DBC: {dbc_path}")
    try:
        db = cantools.database.load_file(dbc_path)
    except FileNotFoundError:
        sys.exit(f"[feeder] DBC file not found: {dbc_path}")
    except Exception as exc:
        sys.exit(f"[feeder] Failed to load DBC: {exc}")

    # Warn if expected messages are absent from the DBC
    expected = {0x118: "0x118", 0x257: "0x257"}
    for mid, label in expected.items():
        try:
            db.get_message_by_frame_id(mid)
        except KeyError:
            print(f"[CAN] warning: message {label} not found in DBC — "
                  "related signals will not be decoded", file=sys.stderr)

    # Open CAN bus
    print(f"[feeder] Opening CAN bus: channel={channel} bustype={bustype}")
    try:
        bus = can.interface.Bus(channel=channel, bustype=bustype)
    except Exception as exc:
        sys.exit(f"[feeder] Failed to open CAN bus: {exc}")

    state = CanState()

    reader_thread = threading.Thread(
        target=_can_reader,
        args=(bus, db, state),
        daemon=True,   # exits automatically when main thread ends
        name="can-reader",
    )
    reader_thread.start()

    period         = 1.0 / hz
    print_interval = 0.5
    t_print        = time.monotonic()

    print(f"[feeder] CAN mode running at {hz:.0f} Hz  ->  {addr[0]}:{addr[1]}")
    print("  Waiting for CAN frames… (Ctrl-C to stop)")
    print(f"  {'speed':>8}  {'pedal':>7}  {'brake':>6}  {'flags':>12}")
    print("  " + "-" * 42)

    try:
        while True:
            t_frame = time.monotonic()

            speed, pedal, brake = state.get()
            send_vehicle_state(sock, addr, speed, pedal, brake)

            if t_frame - t_print >= print_interval:
                t_print = t_frame
                gs, gp, gb = state.received_flags()
                flags = f"{'S' if gs else '-'}{'P' if gp else '-'}{'B' if gb else '-'}"
                print(f"  {speed:8.1f}  {pedal:7.1f}  {brake:6.3f}  {flags:>12}")

            sleep_until = t_frame + period
            now = time.monotonic()
            if sleep_until > now:
                time.sleep(sleep_until - now)

    except KeyboardInterrupt:
        print("\n[feeder] stopped")
    finally:
        bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UDP feeder for engine-sim-headless (EV / Tesla sound project)",
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="engine-sim-headless UDP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9999,
                        help="UDP port (default: 9999)")
    parser.add_argument("--hz",   type=float, default=20.0,
                        help="update rate in Hz (default: 20)")
    parser.add_argument(
        "--mode",
        choices=["tesla3p", "manual", "can"],
        default="tesla3p",
        help=(
            "tesla3p = replay 3P launch profile; "
            "manual = send MANUAL_* constants; "
            "can = decode live CAN bus (requires --dbc) "
            "(default: tesla3p)"
        ),
    )
    # CAN-mode arguments (ignored for other modes)
    parser.add_argument("--dbc",     default="Model3CAN.dbc",
                        help="DBC file path (CAN mode, default: Model3CAN.dbc)")
    parser.add_argument("--channel", default="can0",
                        help="CAN channel (CAN mode, default: can0)")
    parser.add_argument("--bustype", default="socketcan",
                        help="python-can bus type (CAN mode, default: socketcan)")
    args = parser.parse_args()

    addr = (args.host, args.port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        startup_sequence(sock, addr)

        if args.mode == "tesla3p":
            run_tesla3p(sock, addr, args.hz)
        elif args.mode == "manual":
            run_manual(sock, addr, args.hz)
        else:  # can
            run_can(sock, addr, args.hz, args.dbc, args.channel, args.bustype)


if __name__ == "__main__":
    main()
