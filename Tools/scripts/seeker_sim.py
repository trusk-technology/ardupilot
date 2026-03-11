#!/usr/bin/env python3
"""
seeker_sim.py — SITL seeker sensor simulator for ArduPlane INTERCEPT mode.

Connects to a running SITL instance, accepts a target position on the command line,
and sends SEEKER_TARGET MAVLink messages at ~20 Hz simulating what a nose-mounted
seeker would report.

Usage:
  python seeker_sim.py --target-lat=<deg> --target-lon=<deg> --target-alt=<m AMSL>
                       [--connection=udp:127.0.0.1:14550]
                       [--seeker-angle=0]   # deg, seeker mount offset
                       [--rate=20]          # Hz
"""

import argparse
import math
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not found. Install with: pip install pymavlink")
    raise

# MAVLink message ID for SEEKER_TARGET (ardupilotmega extension)
MAVLINK_MSG_ID_SEEKER_TARGET = 11045

def parse_args():
    parser = argparse.ArgumentParser(description="Seeker sensor simulator for ArduPlane INTERCEPT mode")
    parser.add_argument("--target-lat",   type=float, required=True,  help="Target latitude (degrees)")
    parser.add_argument("--target-lon",   type=float, required=True,  help="Target longitude (degrees)")
    parser.add_argument("--target-alt",   type=float, required=True,  help="Target altitude AMSL (metres)")
    parser.add_argument("--connection",   type=str,   default="udp:127.0.0.1:14550",
                        help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    parser.add_argument("--seeker-angle", type=float, default=0.0,
                        help="Seeker mount angle offset from aircraft forward axis (degrees, default 0)")
    parser.add_argument("--rate",         type=float, default=20.0,
                        help="Update rate in Hz (default: 20)")
    return parser.parse_args()


def ned_from_lat_lon_alt(lat1, lon1, alt1, lat2, lon2, alt2):
    """Return (north_m, east_m, down_m) from point 1 to point 2."""
    EARTH_RADIUS = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat_mid = math.radians((lat1 + lat2) / 2.0)
    north = dlat * EARTH_RADIUS
    east  = dlon * EARTH_RADIUS * math.cos(lat_mid)
    down  = -(alt2 - alt1)   # alt increases up; NED down is negative altitude
    return north, east, down


def rotate_ned_to_body(north, east, down, roll, pitch, yaw):
    """Rotate a NED vector into body frame using ZYX Euler angles (radians)."""
    # Rotation matrix NED -> body: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # Row-major NED-to-body rotation (standard aerospace convention)
    bx = (cp*cy)*north          + (cp*sy)*east          + (-sp)*down
    by = (sr*sp*cy - cr*sy)*north + (sr*sp*sy + cr*cy)*east + (sr*cp)*down
    bz = (cr*sp*cy + sr*sy)*north + (cr*sp*sy - sr*cy)*east + (cr*cp)*down
    return bx, by, bz


def main():
    args = parse_args()

    print(f"Connecting to {args.connection} …")
    mav = mavutil.mavlink_connection(args.connection)
    mav.wait_heartbeat()
    print(f"Heartbeat received (system {mav.target_system}, component {mav.target_component})")

    period = 1.0 / args.rate
    seeker_angle_rad = math.radians(args.seeker_angle)

    prev_az  = None
    prev_el  = None
    prev_t   = None

    print(f"Sending SEEKER_TARGET at {args.rate} Hz  (target {args.target_lat:.6f}, "
          f"{args.target_lon:.6f}, alt {args.target_alt:.1f} m AMSL)")
    print("Press Ctrl-C to stop.")

    while True:
        loop_start = time.time()

        # --- 1. Read current aircraft state ---
        pos_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        att_msg = mav.recv_match(type='ATTITUDE',            blocking=False)

        if pos_msg is None or att_msg is None:
            time.sleep(period)
            continue

        ac_lat  = pos_msg.lat  * 1e-7   # degrees
        ac_lon  = pos_msg.lon  * 1e-7   # degrees
        ac_alt  = pos_msg.alt  * 1e-3   # metres AMSL

        roll    = att_msg.roll           # radians
        pitch   = att_msg.pitch          # radians
        yaw     = att_msg.yaw            # radians

        now = time.time()

        # --- 2. Vector from aircraft to target in NED ---
        north, east, down = ned_from_lat_lon_alt(
            ac_lat, ac_lon, ac_alt,
            args.target_lat, args.target_lon, args.target_alt
        )

        dist = math.sqrt(north**2 + east**2 + down**2)
        if dist < 1.0:
            dist = 1.0   # avoid divide-by-zero when on top of target

        # --- 3. Rotate into body frame ---
        bx, by, bz = rotate_ned_to_body(north, east, down, roll, pitch, yaw)

        # --- 4. Apply seeker mount angle offset (rotation around body Y axis) ---
        ca = math.cos(seeker_angle_rad)
        sa = math.sin(seeker_angle_rad)
        sx =  ca * bx + sa * bz
        sy =  by
        sz = -sa * bx + ca * bz

        # --- 5. Azimuth and elevation in seeker frame ---
        az = math.atan2(sy, sx)   # positive = target to the right
        el = math.atan2(-sz, math.sqrt(sx**2 + sy**2))  # positive = target above

        # --- 6. Differentiate to get LOS rates ---
        if prev_az is not None and prev_t is not None:
            dt = now - prev_t
            if dt > 0:
                los_rate_x = (az - prev_az) / dt   # rad/s, X axis (right)
                los_rate_y = (el - prev_el) / dt   # rad/s, Y axis (up)
            else:
                los_rate_x = 0.0
                los_rate_y = 0.0
        else:
            los_rate_x = 0.0
            los_rate_y = 0.0

        prev_az = az
        prev_el = el
        prev_t  = now

        # --- 7. Centroid as fraction of FOV ---
        centroid_x = math.sin(az)
        centroid_y = math.sin(el)
        target_found = 1

        # --- 8. Send SEEKER_TARGET message ---
        # pymavlink may not have this custom message pre-built, so send as raw bytes.
        import struct
        time_boot_ms = int((time.monotonic() % (2**32 / 1000)) * 1000) & 0xFFFFFFFF
        payload = struct.pack('<IfffFFB',
                              time_boot_ms,
                              los_rate_x,
                              los_rate_y,
                              centroid_x,
                              centroid_y,
                              0.0,          # padding to match field order; centroid_y already packed
                              target_found)
        # Re-pack correctly matching XML field order:
        # uint32 time_boot_ms, float los_rate_x, float los_rate_y,
        # float centroid_x, float centroid_y, uint8 target_found
        payload = struct.pack('<IffffB',
                              time_boot_ms,
                              los_rate_x,
                              los_rate_y,
                              centroid_x,
                              centroid_y,
                              target_found)

        mav.mav.send(mavutil.mavlink.MAVLink_unknown_message(
            MAVLINK_MSG_ID_SEEKER_TARGET,
            payload
        )) if hasattr(mavutil.mavlink, 'MAVLink_unknown_message') else None

        # Fallback: try the standard path if the dialect includes SEEKER_TARGET
        try:
            mav.mav.seeker_target_send(
                time_boot_ms,
                los_rate_x,
                los_rate_y,
                centroid_x,
                centroid_y,
                target_found
            )
        except AttributeError:
            # Dialect does not have the message yet; send raw
            buf = mav.mav.pack(
                mav.target_system,
                mav.target_component,
                MAVLINK_MSG_ID_SEEKER_TARGET,
                payload
            )
            mav.mav.file.write(buf)

        print(f"\raz={math.degrees(az):+.2f}°  el={math.degrees(el):+.2f}°  "
              f"los_x={los_rate_x:+.4f} r/s  los_y={los_rate_y:+.4f} r/s  "
              f"dist={dist:.0f} m    ", end="", flush=True)

        elapsed = time.time() - loop_start
        sleep_for = period - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


if __name__ == "__main__":
    main()
