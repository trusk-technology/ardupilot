#!/usr/bin/env python3
"""
Interactive OSD seeker_box test — INTERCEPT mode.

Usage:
    python3 Tools/scripts/test_seeker_osd.py
"""

import math
import sys
import time

from sitl_intercept_utils import (
    connect, send_cmd, set_mode, set_param, wait_ekf_ready)
from pymavlink import mavutil

mav = connect('tcp:127.0.0.1:5760')
wait_ekf_ready(mav, settle=2)

print('\n── Setting params ──')
for name, val in [
    ('OSD_TYPE',          2),     # SITL SFML renderer
    ('OSD1_ENABLE',       1),
    ('OSD1_SEEKRBOX_EN',  1),
    ('OSD1_SEEKRBOX_X',  15),
    ('OSD1_SEEKRBOX_Y',   8),
    ('OSD1_CALLSIGN_EN',  1),
    ('OSD1_CALLSIGN_X',   1),
    ('OSD1_CALLSIGN_Y',  20),
    ('INTC_SPEED',       3.0),
    ('INTC_YAW_P',       2.0),
    ('INTC_YAW_D',       0.3),
    ('INTC_VRT_P',       3.0),
    ('INTC_ACMP',        0.5),
    ('INTC_TOUT',      500.0),
]:
    set_param(mav, name, val)
    print(f'  {name} = {val}')

print('\n── Arm → GUIDED → Takeoff ──')
set_mode(mav, 0)   # STABILIZE
time.sleep(0.5)
send_cmd(mav, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=2989)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'ARM: {ack.result if ack else "no ack"}')
if not ack or ack.result != 0:
    sys.exit('ARM failed')

set_mode(mav, 4)   # GUIDED
time.sleep(0.5)
send_cmd(mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=20.0)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'TAKEOFF: {ack.result if ack else "no ack"}')

print('Climbing …', end='', flush=True)
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg and msg.relative_alt / 1000 > 15:
        print(f' {msg.relative_alt/1000:.1f} m AGL')
        break
    print('.', end='', flush=True)

print('\n── Switching to INTERCEPT (mode 29) ──')
set_mode(mav, 29)
time.sleep(1)
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if not hb or hb.custom_mode != 29:
    sys.exit(f'Mode switch failed (got {hb.custom_mode if hb else "no hb"})')
print(f'Mode confirmed: INTERCEPT ({hb.custom_mode})')

print("""
── Seeker phases — watch the OSD window ──

  Phase 1  0–8s   Centred, bbox expands from 20%→35% of FOV
                   You should see: crosshair + 4 corners spreading apart
                   No guidance arrow (LOS rate ≈ 0)

  Phase 2  8–14s  Target drifts right; at cx>0.5 corners leave screen
                   You should see: corners march right, then '>'' edge indicator

  Phase 3 14–20s  Target back to centre with PN arrow (target moving left)
                   You should see: box re-centres, left-pointing arrow appears

  Phase 4 20–22s  Seeker silent — panel should vanish after ~1 s
""")

PHASES = [
    # (duration, label)
    (8,  'Phase 1 — centred, growing box'),
    (6,  'Phase 2 — drift right → edge indicator'),
    (6,  'Phase 3 — return to centre + PN arrow'),
    (4,  'Phase 4 — seeker silent'),
]

t0 = time.monotonic()

while True:
    t = time.monotonic() - t0
    total = sum(d for d, _ in PHASES)
    if t >= total:
        break

    tboot = int(time.monotonic() * 1000) & 0xFFFFFFFF

    if t < 8:
        # Phase 1: centred, bbox grows from 0.20 to 0.35
        cx, cy   = 0.0, 0.0
        bw = bh  = 0.20 + 0.02 * t   # 0.20 → 0.36 over 8 s
        los_x = los_y = 0.0
        target_found = 1

    elif t < 14:
        # Phase 2: drift right 0→0.65, constant bbox, LOS rate points right
        frac     = (t - 8) / 6.0
        cx       = 0.65 * frac        # 0 → 0.65 (crosses off-screen at 0.5)
        cy       = 0.0
        bw = bh  = 0.25
        los_x    = 0.4                # strong rightward LOS rate → arrow right
        los_y    = 0.0
        target_found = 1

    elif t < 20:
        # Phase 3: return to centre, sinusoidal dip, LOS rate points left
        frac     = (t - 14) / 6.0
        cx       = 0.65 * (1.0 - frac)
        cy       = -0.15 * math.sin(frac * math.pi)
        bw = bh  = 0.25
        los_x    = -0.35              # leftward LOS rate → arrow left
        los_y    = -0.05
        target_found = 1

    else:
        # Phase 4: silent
        time.sleep(0.1)
        continue

    mav.mav.seeker_target_send(
        tboot, los_x, los_y, cx, cy, target_found, bw, bh)

    hw = max(1, round(bw * 20 / 2))
    hh = max(1, round(bh * 10 / 2))
    screen_x = 15 + round(cx * 20)
    screen_y = 8  - round(cy * 10)
    on_screen = abs(cx) <= 0.5 and abs(cy) <= 0.5

    print(f't={t:5.1f}s  cx={cx:+.2f} cy={cy:+.2f}  bw={bw:.2f}  '
          f'OSD: centre=({screen_x},{screen_y}) hw={hw} hh={hh}  '
          f'{"ON" if on_screen else ">> OFFSCREEN <<"}')

    time.sleep(0.1)

print('\nSeeker silent — panel should disappear within 1 s.')
time.sleep(2)
print('Done.')
