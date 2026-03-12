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

sys.path.insert(0, '/home/tom/ardupilot/modules/mavlink')
from pymavlink import mavutil

# ── ArduPlane mode numbers ────────────────────────────────────────────────────
MANUAL    = 0
FBWA      = 5
GUIDED    = 15
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
print(f'Connecting to {args.connect} …')
mav = mavutil.mavlink_connection(args.connect, source_system=255)
mav.wait_heartbeat()
print(f'Heartbeat: sysid={mav.target_system} mode={mav.flightmode}')

mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(0.5)

# ── Helpers ───────────────────────────────────────────────────────────────────
def set_mode(mode_id):
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode_id)

def send_cmd(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        cmd, 0, p1, p2, p3, p4, p5, p6, p7)

def set_param(name, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)

def get_attitude():
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

def get_relative_alt():
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    return msg.relative_alt / 1e3 if msg else None

def current_mode():
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    return hb.custom_mode if hb else None

# ── Step 0: Wait for EKF active ───────────────────────────────────────────────
print('\n── Step 0: Waiting for EKF active ──')
t_boot = time.monotonic()
ekf_active = ready = False
while not (ekf_active and ready):
    msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
    if msg:
        text = msg.text
        print(f'  [{time.monotonic()-t_boot:.1f}s] {text}')
        if 'EKF' in text and 'active' in text:
            ekf_active = True
        if 'Ready' in text:
            ready = True
    if time.monotonic() - t_boot > 60:
        print('  timeout — proceeding')
        break

print('Waiting for EKF GPS lock …')
t_ekf = time.monotonic()
while True:
    msg = mav.recv_match(type=['STATUSTEXT', 'EKF_STATUS_REPORT'],
                         blocking=True, timeout=2)
    if msg:
        if msg.get_type() == 'STATUSTEXT':
            print(f'  [{time.monotonic()-t_ekf:.1f}s] {msg.text}')
        elif msg.get_type() == 'EKF_STATUS_REPORT' and msg.flags & 0x100:
            print(f'  EKF_STATUS_REPORT flags=0x{msg.flags:04x} — position ready')
            break
    if time.monotonic() - t_ekf > 60:
        print('  timeout — proceeding anyway')
        break
time.sleep(2)

# ── Step 1: Set INTERCEPT params ──────────────────────────────────────────────
print('\n── Step 1: Set INTERCEPT_ params ──')
for name, val in [('INTERCEPT_THR', 50.0), ('INTERCEPT_ROLL', 1000.0),
                  ('INTERCEPT_PTCH', 1000.0), ('INTERCEPT_ANG', 0.0),
                  ('INTERCEPT_TOUT', 500.0)]:
    set_param(name, val)
    print(f'  {name} = {val}')

# ── Step 2: Arm and takeoff ───────────────────────────────────────────────────
print('\n── Step 2: Arm and takeoff ──')

# Arm in MANUAL — fewest constraints for ArduPlane
set_mode(MANUAL)
time.sleep(0.5)
send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=2989)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'ARM ack: result={ack.result if ack else "no ack"}')
if not ack or ack.result != 0:
    print('ARM FAILED — aborting')
    sys.exit(1)

# Set takeoff altitude, then switch to TAKEOFF mode (mode 13).
# TAKEOFF mode blocks all external mode switches until it self-completes at TKOFF_ALT.
# We must wait for it to auto-exit before we can switch to INTERCEPT.
set_param('TKOFF_ALT', args.takeoff_alt)
set_mode(TAKEOFF)
time.sleep(1)
mode = current_mode()
print(f'Mode after TAKEOFF switch: {mode}')

# Wait for TAKEOFF mode to complete (auto-switches to CRUISE/FBWA when TKOFF_ALT reached)
print(f'Waiting for TAKEOFF mode to complete (TKOFF_ALT={args.takeoff_alt}m) …', end='', flush=True)
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and hb.custom_mode != TAKEOFF:
        rel_alt = get_relative_alt()
        print(f' auto-exited to mode {hb.custom_mode} at {rel_alt:.1f}m AGL')
        break
    print('.', end='', flush=True)
else:
    print()
    rel_alt = get_relative_alt()
    alt_str = f'{rel_alt:.1f}' if rel_alt is not None else 'N/A'
    print(f'TAKEOFF timeout — still in TAKEOFF mode at {alt_str}m AGL. Trying anyway.')

# ── Step 3: Switch to INTERCEPT (mode 27) ────────────────────────────────────
print('\n── Step 3: Switch to INTERCEPT mode 27 ──')
set_mode(INTERCEPT)
time.sleep(1.5)
mode = current_mode()
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

# Test phases: inject known LOS rates, verify attitude matches
ROLL_GAIN  = 1000.0  # cd / (rad/s)
PITCH_GAIN = 1000.0

while True:
    t = time.monotonic() - t_start

    if t > args.seeker_tout and seeker_live:
        seeker_live = False
        print(f'\n[t={t:.1f}s] SEEKER STOPPED — expecting level flight …')

    # Vary LOS rates over time to create an interesting test signal
    if seeker_live:
        # Sinusoidal LOS rates: ±0.3 rad/s yaw, ±0.2 rad/s pitch
        los_rate_x = 0.3 * math.sin(2 * math.pi * t / 4.0)   # 4s period yaw
        los_rate_y = 0.2 * math.sin(2 * math.pi * t / 6.0)   # 6s period pitch
        centroid_x = 0.0
        centroid_y = 0.0
        tboot = int(time.monotonic() * 1000) & 0xFFFFFFFF
        mav.mav.seeker_target_send(
            tboot, los_rate_x, los_rate_y, centroid_x, centroid_y, 1)
        expected_roll_cd  = los_rate_x * ROLL_GAIN
        expected_pitch_cd = los_rate_y * PITCH_GAIN
    else:
        los_rate_x = 0.0
        los_rate_y = 0.0
        expected_roll_cd  = 0.0
        expected_pitch_cd = 0.0

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
mode = current_mode()
rel_alt = get_relative_alt()
print(f'Final mode={mode}, alt={rel_alt:.1f}m AGL')
