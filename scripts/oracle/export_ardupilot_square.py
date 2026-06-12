"""P3-9 offline oracle: ArduCopter SITL flies the 200 m waypoint square.

Runs the official prebuilt ArduPilot SITL binary inside WSL2 (setup
procedure in tests/fixtures/oracle/README.md), commands the same
geometry the P3-8 bench flies (200 m sides, 50 m AGL, 10 m/s), records
the trajectory over MAVLink and writes
``tests/fixtures/oracle/ardupilot_square.json``.

Offline oracle ONLY (the P1 RotorPy policy): run manually, commit the
JSON, never a runtime dependency. Re-running overwrites the fixture —
treat as a sanctioned re-baseline.

Scope (documented, deliberate): this compares a COMPLETE independent
autopilot (EKF3 + L1/S-curve nav on the default ~2 kg SITL quad)
flying the same mission — an ENVELOPE cross-check of lap time, leg
cross-track class, cruise speed and altitude hold, NOT a model match
(different airframe, different controller, different nav filter).

Usage (from the repo root, Windows):
    .venv/Scripts/python.exe scripts/oracle/export_ardupilot_square.py
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil, mavwp

OUT = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "oracle" \
    / "ardupilot_square.json"
HOME = (-35.363261, 149.165230, 584.0)   # ArduPilot SITL canonical home
ALT = 50.0                               # m AGL
SIDE = 200.0                             # m
SPEED = 10.0                             # m/s (WPNAV_SPEED)
SPEEDUP = 8
M_PER_DEG = 111319.49


def offset(lat: float, lon: float, de: float, dn: float):
    return (lat + dn / M_PER_DEG,
            lon + de / (M_PER_DEG * math.cos(math.radians(lat))))


# Square corners in local east/north metres (same shape as the bench).
CORNERS_EN = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]


def start_sitl() -> subprocess.Popen:
    cmd = ["wsl", "-d", "Ubuntu", "--", "bash", "-lc",
           "cd ~/apsitl && ./arducopter -w --model + "
           f"--speedup {SPEEDUP} --defaults copter.parm "
           f"--home {HOME[0]},{HOME[1]},{HOME[2]},0"]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def ack_ok(m, cmd_id, timeout=5.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg and msg.command == cmd_id:
            return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def upload_mission(m) -> int:
    wp = mavwp.MAVWPLoader()
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    seq = 0

    def add(cmd, lat, lon, alt, p1=0.0):
        nonlocal seq
        wp.add(mavutil.mavlink.MAVLink_mission_item_message(
            m.target_system, m.target_component, seq, frame, cmd,
            0, 1, p1, 0, 0, 0, lat, lon, alt))
        seq += 1

    add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, HOME[0], HOME[1], 0)  # home
    add(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, HOME[0], HOME[1], ALT)
    for de, dn in CORNERS_EN:
        lat, lon = offset(HOME[0], HOME[1], de, dn)
        add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, ALT)
    m.waypoint_clear_all_send()
    m.waypoint_count_send(wp.count())
    for _ in range(wp.count()):
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                           blocking=True, timeout=10)
        if msg is None:
            raise RuntimeError("mission upload stalled")
        m.mav.send(wp.wp(msg.seq))
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"mission rejected: {ack}")
    return wp.count()


def main() -> None:
    sitl = start_sitl()
    try:
        time.sleep(3.0)
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
        m.wait_heartbeat(timeout=60)
        print(f"heartbeat: sys {m.target_system}")

        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            100000, 0, 0, 0, 0, 0)

        m.param_set_send("WPNAV_SPEED", SPEED * 100.0)  # cm/s
        time.sleep(1.0)

        n_items = upload_mission(m)
        print(f"mission uploaded ({n_items} items)")

        m.set_mode_apm("GUIDED")
        armed = False
        for _ in range(120):                 # EKF/GPS readiness: retry arm
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            if ack_ok(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                armed = True
                break
            time.sleep(2.0)
        if not armed:
            raise RuntimeError("arming never accepted (EKF not ready?)")
        print("armed")

        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, ALT)
        t0 = time.time()
        while time.time() - t0 < 120:
            msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True,
                               timeout=5)
            if msg and msg.relative_alt / 1000.0 > ALT - 2.0:
                break
        print("at altitude; AUTO")
        m.set_mode_apm("AUTO")

        lat0 = math.radians(HOME[0])
        rows = []
        last_seq = n_items - 1
        done = False
        t0 = time.time()
        while time.time() - t0 < 600 and not done:
            msg = m.recv_match(
                type=["GLOBAL_POSITION_INT", "MISSION_ITEM_REACHED"],
                blocking=True, timeout=10)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ITEM_REACHED":
                print(f"reached {msg.seq}/{last_seq}")
                done = msg.seq >= last_seq
                continue
            x = (msg.lon / 1e7 - HOME[1]) * M_PER_DEG * math.cos(lat0)
            y = (msg.lat / 1e7 - HOME[0]) * M_PER_DEG
            rows.append({
                "t_boot_s": msg.time_boot_ms / 1000.0,
                "x": round(x, 3), "y": round(y, 3),
                "alt_agl": round(msg.relative_alt / 1000.0, 3),
                "vx": round(msg.vx / 100.0, 3),
                "vy": round(msg.vy / 100.0, 3),
            })
        if not done:
            raise RuntimeError("mission did not complete within timeout")

        OUT.write_text(json.dumps({
            "oracle": "ArduCopter stable SITL (prebuilt x86_64), "
                      "default '+' quad model, EKF3",
            "speedup": SPEEDUP,
            "wpnav_speed_ms": SPEED,
            "alt_agl_m": ALT,
            "corners_en": CORNERS_EN,
            "samples": rows,
        }, indent=1))
        print(f"wrote {OUT} ({len(rows)} samples)")
    finally:
        sitl.kill()
        subprocess.run(["wsl", "-d", "Ubuntu", "--", "bash", "-lc",
                        "pkill -f arducopter || true"], check=False)


if __name__ == "__main__":
    sys.exit(main())
