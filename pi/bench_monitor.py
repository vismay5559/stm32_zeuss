#!/usr/bin/env python3
"""
Everything the board can see, on one terminal screen. No ROS, no Pi.

    python3 pi/bench_monitor.py                      # finds the port
    python3 pi/bench_monitor.py /dev/ttyACM0
    python3 pi/bench_monitor.py --once               # one frame, then exit
    python3 pi/bench_monitor.py --hz 2               # slower refresh

WHY THIS EXISTS. The laptop has no CAN interface, so it cannot look at the bus
directly - only the STM32 can. But the STM32 already puts every CAN value it
reads into the state packet it sends over USB, so plugging the board's DATA
port into the laptop instead of the Pi gets all of it with nothing else
running. This is the single-leg test's console view, for all eight actuators
and the IMU at once.

WHAT IT PROVES, and what it does not. Every number here arrived over CAN or
from the IMU within the last millisecond, so a drive that appears with a fresh
heartbeat is powered, correctly configured for 2.5 Mbit FD, and answering. It
says nothing about whether the joint moves the right way or where its zero is -
those need the robot, and robot_config.c still has every sign at +1 and every
offset at 0.

NOTHING IS SENT TO THE BOARD and nothing can move. The board only drives an
axis after a command arrives with enable set, and this program never sends one.

Needs pyserial. The board must be flashed in NEXUS_MODE_ROBOT: the single-leg
test modes only talk to bus 0.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nexus_proto import (ACT_HB_FRESH, ACT_TELEM_FRESH, FUSION_INVALID,
                         FUSION_CONVERGING, FUSION_OK, JOINT_NAMES,
                         NUM_JOINTS, SPRING_NAMES, STREAM_LEG_TEST)
from nexus_link import NexusLink
from odrive_names import axis_state_name, error_names

FUSION_NAMES = {FUSION_INVALID: "INVALID", FUSION_CONVERGING: "CONVERGING",
                FUSION_OK: "OK"}

# How many joints sit on each bus. The joint map is bus * NODES + (node - 1),
# so this is also how a joint index becomes a bus and a node.
NODES_PER_BUS = NUM_JOINTS // 2

BOLD, DIM, RESET = "\033[1m", "\033[2m", "\033[0m"
RED, GREEN, YELLOW = "\033[31m", "\033[32m", "\033[33m"


def colour(on: bool) -> str:
    return GREEN if on else RED


def find_port(arg: str | None) -> str:
    if arg:
        return arg
    for cand in ("/dev/zeus_stm32",):
        if os.path.exists(cand):
            return cand
    import glob
    hits = sorted(glob.glob("/dev/ttyACM*"))
    if not hits:
        sys.exit("no /dev/ttyACM* and no /dev/zeus_stm32 - is the board's DATA "
                 "port (not the ST-LINK one) plugged into this machine?")
    return hits[0]


def render(pkt, stats, port: str) -> str:
    out = []
    a = out.append

    src = "LEG TEST (bus 0 only!)" if pkt.stream_flags & STREAM_LEG_TEST else "ROBOT"
    a(f"{BOLD}zeus bench monitor{RESET}   {port}   {src}"
      f"   seq {pkt.seq}   {time.strftime('%H:%M:%S')}")
    a(f"  link      {stats}")

    faults = ",".join(pkt.faults()) or "none"
    a(f"  board     safety {BOLD}{pkt.safety_state_name}{RESET}"
      f"   fusion {FUSION_NAMES.get(pkt.fused_valid, '?')}"
      f"   health faults [{faults}]")
    a("")

    # ---- the actuators, grouped by bus, because that is how they are wired --
    a(f"{BOLD}  CAN - 2 buses, {NODES_PER_BUS} nodes each{RESET}")
    a(f"  {DIM}bus node joint              pos rad    vel rad/s   torque Nm"
      f"   state                 hb  tlm  error{RESET}")

    for j in range(NUM_JOINTS):
        bus, node = j // NODES_PER_BUS, (j % NODES_PER_BUS) + 1
        hb = bool(pkt.act_flags[j] & ACT_HB_FRESH)
        tlm = bool(pkt.act_flags[j] & ACT_TELEM_FRESH)
        err = pkt.act_error[j]
        errtxt = ("|".join(error_names(err)) if err
                  else f"{GREEN}-{RESET}")
        # A drive that has never spoken reports nothing rather than a zero that
        # looks like a measurement.
        silent = not hb and not tlm
        # Pad BEFORE colouring: the escape sequences are characters as far as
        # format widths are concerned, so colouring first shifts the row.
        name = f"{JOINT_NAMES[j]:<20}"
        if silent:
            name = f"{RED}{name}{RESET}"
        a(f"   {bus}    {node}  {name} "
          f"{pkt.joint_pos[j]:+9.4f} {pkt.joint_vel[j]:+10.4f} "
          f"{pkt.act_torque[j]:+10.3f}  "
          f"{axis_state_name(pkt.act_state[j]):<20} "
          f"{colour(hb)}{'Y' if hb else 'n'}{RESET}   "
          f"{colour(tlm)}{'Y' if tlm else 'n'}{RESET}   {errtxt}")

    live = sum(1 for j in range(NUM_JOINTS)
               if pkt.act_flags[j] & (ACT_HB_FRESH | ACT_TELEM_FRESH))
    verdict = (f"{GREEN}all {NUM_JOINTS} answering{RESET}" if live == NUM_JOINTS
               else f"{RED}{live} of {NUM_JOINTS} answering{RESET}")
    a(f"  {verdict}   "
      f"{DIM}hb = heartbeat heard recently, tlm = encoder telemetry{RESET}")
    a("")

    # ---- the IMU -----------------------------------------------------------
    ax, ay, az = pkt.imu_accel
    gx, gy, gz = pkt.imu_gyro
    mag = (ax * ax + ay * ay + az * az) ** 0.5
    upright = abs(mag - 9.81) < 1.0 and abs(az) > 9.0
    a(f"{BOLD}  IMU - BNO085 on USART2{RESET}")
    a(f"   accel m/s2   x {ax:+8.3f}  y {ay:+8.3f}  z {az:+8.3f}"
      f"   |a| {mag:6.3f}  "
      + (f"{GREEN}upright{RESET}" if upright
         else f"{YELLOW}not (0,0,+9.81){RESET}"))
    a(f"   gyro  rad/s  x {gx:+8.4f}  y {gy:+8.4f}  z {gz:+8.4f}")
    qw, qx, qy, qz = pkt.imu_quat
    a(f"   quat         w {qw:+7.4f} x {qx:+7.4f} y {qy:+7.4f} z {qz:+7.4f}"
      f"   {DIM}(the sensor's own fusion, for comparison only){RESET}")
    a("")

    # ---- the springs, which may not be wired yet ---------------------------
    a(f"{BOLD}  Spring encoders - 4 x AS5047P on SPI1{RESET}")
    bits = " ".join(
        f"{colour(bool(pkt.enc_valid & (1 << i)))}{SPRING_NAMES[i]}"
        f"={pkt.spring_angle[i]:+.4f}{RESET}" for i in range(len(SPRING_NAMES)))
    a(f"   {bits}")
    a(f"   {DIM}valid 0x{pkt.enc_valid:X}; all red is the expected reading "
      f"until they are wired{RESET}")
    a("")

    a(f"  {DIM}contacts {pkt.contact}   phase {pkt.phase:.3f}   "
      f"loop {pkt.loop_us_max} us max   nothing is being sent to the board"
      f"{RESET}")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--hz", type=float, default=4.0, help="refresh rate")
    ap.add_argument("--once", action="store_true", help="one frame, then exit")
    a = ap.parse_args(argv)

    port = find_port(a.port)
    link = NexusLink(port)
    try:
        link.start()
    except Exception as exc:
        print(f"cannot open {port}: {exc}", file=sys.stderr)
        return 1

    period = 1.0 / max(a.hz, 0.1)
    first = True
    try:
        with link:
            while True:
                pkt = link.latest()
                if pkt is None:
                    if first:
                        print(f"reading {port}, waiting for a packet "
                              f"(ctrl-c to stop)...")
                        first = False
                    # Bytes arriving but nothing parsing is almost always a
                    # protocol version difference, and it looks exactly like a
                    # working cable.
                    if link.stats.bytes_in > 20000 and link.stats.packets == 0:
                        print(f"\n{link.stats.bytes_in} bytes arrived and none "
                              f"parsed. Either the board is on a different "
                              f"protocol version than pi/nexus_proto.py, or "
                              f"something else is holding the port.",
                              file=sys.stderr)
                        return 1
                    time.sleep(0.1)
                    continue

                if not a.once:
                    print("\033[H\033[J", end="")
                print(render(pkt, link.stats, port))
                if a.once:
                    return 0
                first = False
                time.sleep(period)
    except KeyboardInterrupt:
        print()
        return 0


if __name__ == "__main__":
    sys.exit(main())
