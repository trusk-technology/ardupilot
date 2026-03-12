#!/usr/bin/env python3
"""
Combined INTERCEPT mode test + seeker simulator for ArduCopter SITL.
Single TCP:5760 connection — sends SEEKER_TARGET and controls/monitors the vehicle.

Usage:
    python3 test_intercept_copter.py [--target-lat LAT] [--target-lon LON] [--target-alt ALT]
"""

import math
import sys
import time
import argparse

# Use the ArduPilot bundled mavlink for SEEKER_TARGET support
sys.path.insert(0, '/home/tom/ardupilot/modules/mavlink')

from pymavlink import mavutil

# ── CLI ──────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--target-lat',  type=float, default=-35.362881)
parser.add_argument('--target-lon',  type=float, default=149.165222)
parser.add_argument('--target-alt',  type=float, default=600.0,
                    help='Target altitude (m AMSL)')
parser.add_argument('--takeoff-alt', type=float, default=20.0)
parser.add_argument('--connect',     default='tcp:127.0.0.1:5760')
parser.add_argument('--seeker-tout', type=float, default=10.0,
                    help='Seconds before killing seeker data (timeout test)')
args = parser.parse_args()

# ── Connect ──────────────────────────────────────────────────────────────────
print(f'Connecting to {args.connect} …')
mav = mavutil.mavlink_connection(args.connect, source_system=255)
mav.wait_heartbeat()
print(f'Heartbeat: sysid={mav.target_system} cmpid={mav.target_component} '
      f'mode={mav.flightmode}')

# Request telemetry streams (bare SITL binary doesn't auto-start them)
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(0.5)

# ── Helpers ──────────────────────────────────────────────────────────────────
def send_cmd(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        cmd, 0, p1, p2, p3, p4, p5, p6, p7)

def set_mode(mode_id):
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode_id)

def set_param(name, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)

def get_relative_alt_m():
    """Return relative altitude in metres (above takeoff point)."""
    msg = None
    for _ in range(50):
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if m:
            msg = m
        else:
            break
    if msg is None:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg is None:
        return None, None
    return msg.relative_alt / 1e3, msg   # relative_alt is mm

def get_pos():
    """Return (lat_deg, lon_deg, alt_m_amsl, yaw_deg) from GLOBAL_POSITION_INT."""
    msg = None
    for _ in range(50):
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if m:
            msg = m
        else:
            break
    if msg is None:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg is None:
        return None
    return (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3, msg.hdg / 100.0)

def bearing_to(lat1, lon1, lat2, lon2):
    """Return bearing from point 1 to point 2 in radians."""
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(x, y)

def elevation_to(alt_vehicle, target_alt_amsl, distance_m):
    """Return elevation angle (rad) from vehicle to target."""
    if distance_m < 1.0:
        return 0.0
    dalt = target_alt_amsl - alt_vehicle
    return math.atan2(dalt, distance_m)

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ── Step 0: Wait for EKF to become active ────────────────────────────────────
print('\n── Step 0: Waiting for EKF active ──')
print('(watching STATUSTEXT…)', end='', flush=True)
t_boot = time.monotonic()
ekf_active = False
ready = False
while not (ekf_active and ready):
    msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
    if msg:
        text = msg.text
        print(f'\n  [{time.monotonic()-t_boot:.1f}s] {text}', end='', flush=True)
        if 'EKF' in text and 'active' in text:
            ekf_active = True
        if 'Ready' in text:
            ready = True
    if time.monotonic() - t_boot > 60:
        print(f'\n  timeout (ekf_active={ekf_active} ready={ready}) — proceeding')
        break
print()

# Wait for EKF origin to be set AND position estimate to be valid
print('Waiting for EKF GPS lock and position …', end='', flush=True)
t_origin = time.monotonic()
origin_count = 0
while True:
    msg = mav.recv_match(type=['STATUSTEXT', 'EKF_STATUS_REPORT'],
                         blocking=True, timeout=2)
    if msg:
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text
            print(f'\n  [{time.monotonic()-t_origin:.1f}s] {text}', end='', flush=True)
            if 'origin set' in text.lower():
                origin_count += 1
        elif msg.get_type() == 'EKF_STATUS_REPORT':
            # flags bit 3 (0x08) = pred_horiz_pos_rel, bit 4 (0x10) = pred_horiz_pos_abs
            # bit 8 (0x100) = horiz_pos_abs, bit 9 (0x200) = horiz_pos_rel
            if msg.flags & 0x108:  # horiz_pos_abs or pred_horiz_pos_abs + pred_rel
                print(f'\n  EKF_STATUS_REPORT flags=0x{msg.flags:04x} — position ready')
                break
    if time.monotonic() - t_origin > 40:
        print(f' timeout (origins={origin_count}) — proceeding anyway')
        break
print()
# Give position + altitude estimate time to stabilise
time.sleep(3)

# ── Step 1: Set INTC_ params, arm in STABILIZE, switch to GUIDED ─────────────
print('\n── Step 1: Set INTC_ params, arm, guided ──')

# Set INTC_ params explicitly (AP_SUBGROUPPTR init ordering can leave them 0)
for name, val in [('INTC_SPEED', 3.0), ('INTC_YAW_P', 2.0), ('INTC_YAW_D', 0.3),
                  ('INTC_VRT_P', 3.0), ('INTC_ACMP', 0.5), ('INTC_TOUT', 500.0)]:
    set_param(name, val)
    print(f'  {name} = {val}')

# Arm in STABILIZE — no position estimate required (alt_checks still applies,
# but ekf_alt_ok() should pass once EKF3 is active)
set_mode(0)  # STABILIZE
time.sleep(0.5)
send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1, p2=2989)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'ARM ack: result={ack.result if ack else "no ack"}')
if not ack or ack.result != 0:
    print('ARM FAILED — aborting')
    sys.exit(1)

# Switch to GUIDED for takeoff
set_mode(4)  # GUIDED
time.sleep(1)

# ── Step 2: Takeoff ───────────────────────────────────────────────────────────
print(f'\n── Step 2: Takeoff to {args.takeoff_alt} m ──')
send_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=args.takeoff_alt)
ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print(f'TAKEOFF ack: result={ack.result if ack else "no ack"}')
if ack and ack.result != 0:
    print('TAKEOFF FAILED — aborting')
    sys.exit(1)

# Wait for relative altitude (not AMSL)
print('Waiting for altitude …', end='', flush=True)
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    rel_alt, _ = get_relative_alt_m()
    if rel_alt is not None and rel_alt > args.takeoff_alt * 0.75:
        print(f' reached {rel_alt:.1f} m AGL')
        break
    print('.', end='', flush=True)
    time.sleep(1)
else:
    print(' TIMEOUT waiting for altitude — aborting')
    sys.exit(1)

# ── Step 3: Switch to INTERCEPT (mode 29) ────────────────────────────────────
print('\n── Step 3: Switch to INTERCEPT mode 29 ──')
set_mode(29)
time.sleep(1.5)

# Confirm — flush until we get a heartbeat
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if hb:
    print(f'Mode after switch: custom_mode={hb.custom_mode}')
    if hb.custom_mode != 29:
        print(f'ERROR: mode switch to 29 REJECTED (got {hb.custom_mode}) — aborting')
        sys.exit(1)
else:
    print('No heartbeat received — aborting')
    sys.exit(1)

# ── Step 4: Main loop — send seeker data, monitor convergence ────────────────
print(f'\n── Step 4: Seeker loop (will run {args.seeker_tout}s then go silent) ──')
print(f'Target: lat={args.target_lat}, lon={args.target_lon}, alt={args.target_alt} m AMSL')

HFOV = math.radians(10.0)   # half-FOV width  (20° full width)
VFOV = math.radians(10.0)   # half-FOV height

t_start = time.monotonic()
prev_az   = None
prev_el   = None
prev_time = None
seeker_live = True

print(f"\n{'t(s)':>6} {'dist(m)':>8} {'az_err(°)':>10} {'el_err(°)':>10} "
      f"{'cx':>7} {'cy':>7} {'yaw_rate':>10} {'seeker':>8}")

while True:
    t = time.monotonic() - t_start

    # Kill seeker data after timeout to test position-hold fallback
    if t > args.seeker_tout and seeker_live:
        seeker_live = False
        print(f'\n[t={t:.1f}s] SEEKER DATA STOPPED — expecting position hold …')

    pos = get_pos()
    if pos is None:
        time.sleep(0.05)
        continue

    lat, lon, alt, hdg_deg = pos
    dist = haversine_m(lat, lon, args.target_lat, args.target_lon)

    # Body-frame azimuth error (positive = target to the right)
    az_world = bearing_to(lat, lon, args.target_lat, args.target_lon)
    yaw_rad  = math.radians(hdg_deg) if hdg_deg < 999 else 0.0
    az_err   = az_world - yaw_rad
    # Wrap to ±π
    az_err = (az_err + math.pi) % (2 * math.pi) - math.pi

    el_err = elevation_to(alt, args.target_alt, dist)  # positive = target above

    # LOS rates (derivative)
    now = time.monotonic()
    if prev_az is not None and (now - prev_time) > 0:
        dt = now - prev_time
        los_rate_x = (az_err - prev_az) / dt
        los_rate_y = (el_err - prev_el) / dt
    else:
        los_rate_x = 0.0
        los_rate_y = 0.0
    prev_az, prev_el, prev_time = az_err, el_err, now

    # Map to centroid fractions (clamp to ±1)
    cx = max(-1.0, min(1.0, az_err / HFOV))
    cy = max(-1.0, min(1.0, el_err / VFOV))

    if seeker_live:
        tboot = int(time.monotonic() * 1000) & 0xFFFFFFFF
        target_found = 1
        mav.mav.seeker_target_send(
            tboot,
            los_rate_x, los_rate_y,
            cx, cy,
            target_found)

    # Get yaw rate for display
    att = mav.recv_match(type='ATTITUDE', blocking=False)
    yaw_rate = att.yawspeed if att else float('nan')

    print(f'{t:6.1f} {dist:8.1f} {math.degrees(az_err):10.2f} '
          f'{math.degrees(el_err):10.2f} {cx:7.3f} {cy:7.3f} '
          f'{yaw_rate:10.3f} {"LIVE" if seeker_live else "DEAD":>8}')

    # Stop after seeker_tout + 10s total
    if t > args.seeker_tout + 10.0:
        print(f'\n── Test complete ──')
        break

    time.sleep(0.1)

# Final position
pos = get_pos()
if pos:
    print(f'Final position: lat={pos[0]:.6f}, lon={pos[1]:.6f}, alt={pos[2]:.1f} m AMSL')
