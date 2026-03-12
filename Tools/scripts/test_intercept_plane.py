#!/usr/bin/env python3
"""
ArduPlane INTERCEPT mode (27) SITL test.
Single TCP:5760 connection — injects SEEKER_TARGET and verifies roll/pitch response.

Controller under test (mode_intercept.cpp):
  nav_roll_cd  = los_rate_x * INTERCEPT_ROLL  (default 1000 cd / rad/s)
  nav_pitch_cd = los_rate_y * INTERCEPT_PTCH  (default 1000 cd / rad/s)
  Timeout → level flight (both 0)

Usage:
  python3 test_intercept_plane.py [--connect tcp:127.0.0.1:5760]
"""

import math
import sys
import time
import argparse

from sitl_intercept_utils import (
    connect, send_cmd, set_mode, set_param, get_relative_alt, wait_ekf_ready)
from pymavlink import mavutil

# ── ArduPlane mode numbers ────────────────────────────────────────────────────
MANUAL    = 0
TAKEOFF   = 13
INTERCEPT = 27

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='tcp:127.0.0.1:5760')
parser.add_argument('--takeoff-alt', type=float, default=100.0,
                    help='Target altitude AGL for takeoff (m)')
parser.add_argument('--seeker-tout', type=float, default=8.0,
                    help='Seconds before stopping seeker data')
args = parser.parse_args()

# ── Connect ───────────────────────────────────────────────────────────────────
mav = connect(args.connect)

# ── Attitude helper ───────────────────────────────────────────────────────────
def get_attitude():
    """Return freshest ATTITUDE message (drains stale frames first)."""
    msg = None
    for _ in range(20):
        m = mav.recv_match(type='ATTITUDE', blocking=False)
        if m:
            msg = m
        else:
            break
    if msg is None:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    return msg

# ── Step 0: Wait for EKF ─────────────────────────────────────────────────────
wait_ekf_ready(mav, settle=2)

# ── Step 1: Set INTERCEPT params ──────────────────────────────────────────────
print('\n── Step 1: Set INTERCEPT_ params ──')
for name, val in [('INTERCEPT_THR', 50.0), ('INTERCEPT_ROLL', 1000.0),
                  ('INTERCEPT_PTCH', 1000.0), ('INTERCEPT_ANG', 0.0),
                  ('INTERCEPT_TOUT', 500.0)]:
    set_param(mav, name, val)
    print(f'  {name} = {val}')

# ── Step 2: Arm and takeoff ───────────────────────────────────────────────────
print('\n── Step 2: Arm and takeoff ──')

# Arm in MANUAL — fewest constraints for ArduPlane
set_mode(mav, MANUAL)
time.sleep(0.5)
send_cmd(mav, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=2989)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'ARM ack: result={ack.result if ack else "no ack"}')
if not ack or ack.result != 0:
    print('ARM FAILED — aborting')
    sys.exit(1)

# TAKEOFF mode (13) climbs to TKOFF_ALT then auto-exits.
# It blocks all external mode switches while climbing — wait for auto-exit.
set_param(mav, 'TKOFF_ALT', args.takeoff_alt)
set_mode(mav, TAKEOFF)
time.sleep(1)
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
print(f'Mode after TAKEOFF switch: {hb.custom_mode if hb else "?"}')

print(f'Waiting for TAKEOFF mode to complete (TKOFF_ALT={args.takeoff_alt}m) …',
      end='', flush=True)
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and hb.custom_mode != TAKEOFF:
        rel_alt = get_relative_alt(mav)
        alt_str = f'{rel_alt:.1f}' if rel_alt is not None else 'N/A'
        print(f' auto-exited to mode {hb.custom_mode} at {alt_str}m AGL')
        break
    print('.', end='', flush=True)
else:
    print()
    rel_alt = get_relative_alt(mav)
    alt_str = f'{rel_alt:.1f}' if rel_alt is not None else 'N/A'
    print(f'TAKEOFF timeout — still in TAKEOFF mode at {alt_str}m AGL. Trying anyway.')

# ── Step 3: Switch to INTERCEPT (mode 27) ────────────────────────────────────
print('\n── Step 3: Switch to INTERCEPT mode 27 ──')
set_mode(mav, INTERCEPT)
time.sleep(1.5)
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
mode = hb.custom_mode if hb else None
print(f'Mode after INTERCEPT switch: custom_mode={mode}')
if mode != INTERCEPT:
    print(f'ERROR: mode switch to {INTERCEPT} REJECTED (got {mode}) — aborting')
    sys.exit(1)

# ── Step 4: Seeker injection loop ─────────────────────────────────────────────
print(f'\n── Step 4: Seeker loop ({args.seeker_tout}s live, then 8s dead) ──')
print(f'Expected: roll ≈ los_rate_x × 1000 cd, pitch ≈ los_rate_y × 1000 cd')
print()
print(f"{'t(s)':>6} {'los_rx':>8} {'los_ry':>8} "
      f"{'roll(°)':>9} {'pitch(°)':>9} {'exp_roll(°)':>12} {'exp_pit(°)':>12} {'seeker':>8}")

t_start = time.monotonic()
seeker_live = True

ROLL_GAIN  = 1000.0  # cd / (rad/s)
PITCH_GAIN = 1000.0

while True:
    t = time.monotonic() - t_start

    if t > args.seeker_tout and seeker_live:
        seeker_live = False
        print(f'\n[t={t:.1f}s] SEEKER STOPPED — expecting level flight …')

    if seeker_live:
        # Sinusoidal LOS rates: ±0.3 rad/s roll-axis, ±0.2 rad/s pitch-axis
        los_rate_x = 0.3 * math.sin(2 * math.pi * t / 4.0)   # 4s period
        los_rate_y = 0.2 * math.sin(2 * math.pi * t / 6.0)   # 6s period
        tboot = int(time.monotonic() * 1000) & 0xFFFFFFFF
        mav.mav.seeker_target_send(tboot, los_rate_x, los_rate_y, 0.0, 0.0, 1)
        expected_roll_cd  = los_rate_x * ROLL_GAIN
        expected_pitch_cd = los_rate_y * PITCH_GAIN
    else:
        los_rate_x = los_rate_y = 0.0
        expected_roll_cd = expected_pitch_cd = 0.0

    att = get_attitude()
    if att:
        roll_deg  = math.degrees(att.roll)
        pitch_deg = math.degrees(att.pitch)
    else:
        roll_deg = pitch_deg = float('nan')

    print(f'{t:6.1f} {los_rate_x:8.3f} {los_rate_y:8.3f} '
          f'{roll_deg:9.2f} {pitch_deg:9.2f} '
          f'{expected_roll_cd/100:12.2f} {expected_pitch_cd/100:12.2f} '
          f'{"LIVE" if seeker_live else "DEAD":>8}')

    if t > args.seeker_tout + 8.0:
        print('\n── Test complete ──')
        break

    time.sleep(0.1)

# Final state
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
mode = hb.custom_mode if hb else None
rel_alt = get_relative_alt(mav)
print(f'Final mode={mode}, alt={rel_alt:.1f}m AGL')
