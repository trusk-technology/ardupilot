#!/usr/bin/env python3
"""
sitl_intercept_utils.py — shared helpers for INTERCEPT mode SITL test scripts.

Provides: connection setup, MAVLink command helpers, EKF cold-boot wait.
"""

import sys
import time

# Use the ArduPilot bundled mavlink dialect for SEEKER_TARGET support
sys.path.insert(0, '/home/tom/ardupilot/modules/mavlink')
from pymavlink import mavutil  # noqa: E402  (import after sys.path tweak)


def connect(url, source_system=255):
    """Connect to SITL at *url*, request all data streams, return the connection."""
    print(f'Connecting to {url} …')
    mav = mavutil.mavlink_connection(url, source_system=source_system)
    mav.wait_heartbeat()
    print(f'Heartbeat: sysid={mav.target_system} mode={mav.flightmode}')
    # Bare SITL binary doesn't auto-start telemetry; request everything.
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    time.sleep(0.5)
    return mav


def send_cmd(mav, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        cmd, 0, p1, p2, p3, p4, p5, p6, p7)


def set_mode(mav, mode_id):
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode_id)


def set_param(mav, name, value):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)


def get_relative_alt(mav):
    """Return current altitude above home in metres (drains stale frames first)."""
    msg = None
    for _ in range(50):
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if m:
            msg = m
        else:
            break
    if msg is None:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    return msg.relative_alt / 1e3 if msg else None


def wait_ekf_ready(mav, ekf_timeout=60, pos_timeout=60, settle=2):
    """
    Block until the EKF is active and has a valid GPS position.

    Phase 1 — waits for both 'AHRS: EKF3 active' and 'ArduPilot Ready'
              in STATUSTEXT (up to *ekf_timeout* seconds).
    Phase 2 — waits for EKF_STATUS_REPORT with horiz_pos_abs flag set
              (up to *pos_timeout* seconds), then sleeps *settle* seconds.
    """
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
        if time.monotonic() - t_boot > ekf_timeout:
            print(f'  timeout (ekf_active={ekf_active} ready={ready}) — proceeding')
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
        if time.monotonic() - t_ekf > pos_timeout:
            print('  timeout — proceeding anyway')
            break
    time.sleep(settle)
