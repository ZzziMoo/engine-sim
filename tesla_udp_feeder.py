"""
tesla_udp_feeder.py
-------------------
Feeds speed / pedal / brake state to engine-sim-headless over UDP.

Usage:
    python tesla_udp_feeder.py --mode tesla3p          # simulated Model 3 P run
    python tesla_udp_feeder.py --mode manual            # edit MANUAL_* below
    python tesla_udp_feeder.py --host 192.168.1.5 --port 9999 --hz 50 --mode tesla3p

engine-sim-headless must be running with:
    engine-sim-headless --script <engine.mr> [--port 9999]
"""

import argparse
import socket
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
# TODO: Tesla CAN integration
#
# Replace the simulated values in the main loop with real decoded CAN signals.
# Suggested mapping (Model 3 / Model Y CAN):
#
#   Message ID  Signal                  -> variable
#   ----------  ------                  -----------
#   0x118       DI_vehicleSpeed (kph)   -> speed_mph = value * 0.621371
#   0x118       DI_accelPedalPos (0-255 or 0-100%) -> pedal_pct = value
#   0x20A       VCRIGHT_brakeApplied (bool) or
#   0x20A       VCRIGHT_brakePedalPos    -> brake = value  (0.0–1.0)
#
# Libraries to consider:
#   python-can  (pip install python-can)
#   cantools    (pip install cantools)  + Tesla DBC file
#
# Example stub:
#   import can
#   bus = can.interface.Bus(channel='can0', bustype='socketcan')
#   msg = bus.recv(timeout=0.05)
#   if msg and msg.arbitration_id == 0x118:
#       speed_mph = decode_speed(msg.data) * 0.621371
#       pedal_pct = decode_pedal(msg.data)
# ---------------------------------------------------------------------------


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
    parser.add_argument("--mode", choices=["tesla3p", "manual"], default="tesla3p",
                        help="tesla3p = replay 3P launch profile; "
                             "manual = send MANUAL_* constants (default: tesla3p)")
    args = parser.parse_args()

    addr = (args.host, args.port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        startup_sequence(sock, addr)

        if args.mode == "tesla3p":
            run_tesla3p(sock, addr, args.hz)
        else:
            run_manual(sock, addr, args.hz)


if __name__ == "__main__":
    main()
