"""
Live view of the robot in Rerun.

    pip install rerun-sdk pyserial

WHERE THINGS RUN
----------------

Rerun splits into an SDK and a Viewer, and they do not have to be on the same
machine. That split is the whole answer to "how do I see this on my laptop"
when the Pi has no screen:

    STM32 ──USB──► Raspberry Pi ──network──► your laptop
    1 kHz          this script                Rerun Viewer
    sensing        (the SDK)                  (the window)
                   headless, no display       where you actually look

Three ways to run it, in the order you are likely to want them:

  1. VIEWER ON YOUR LAPTOP, Pi streams to it. Live, lowest friction.

         laptop$  pip install rerun-sdk && rerun          # opens, waits
         pi$      python pi/zeus_viz.py /dev/ttyACM0 --connect 192.168.1.50

  2. RECORD NOW, LOOK LATER. No network needed; best for a run you want to
     keep, and for anything that happens too fast to watch live.

         pi$      python pi/zeus_viz.py /dev/ttyACM0 --save run.rrd
         laptop$  scp pi@raspberrypi:run.rrd .  &&  rerun run.rrd

  3. EVERYTHING ON ONE MACHINE with a display - a laptop with the STM32
     plugged straight in, which is how you will test before the Pi exists.

         python pi/zeus_viz.py COM7 --spawn

WHAT ABOUT MUJOCO
-----------------

MuJoCo is not part of the robot. It is the simulator the policy was trained in,
and it runs on a desktop, not on the Pi and not on the STM32. It appears here
only for an optional comparison: run the simulator playing the same gait, feed
its joint angles to log_sim(), and the simulated and real angles land on one
plot on one timeline. That divergence is where sim-to-real transfer problems
show up first. Skip it entirely if you just want to watch the robot.

WHY RERUN RATHER THAN PLOTTING BY HAND
--------------------------------------

Everything shares a timeline, so "what were the springs doing when that foot
landed" is answered by dragging rather than by correlating timestamps between
tools. And sessions replay, which matters when the interesting event lasted
40 ms and happened once.

The entity tree below is the layout - Rerun groups by path, so `joints/pos/3`
and `joints/ref/3` land next to each other and can be dropped into one plot.
"""

from __future__ import annotations

import sys
import time
from typing import Optional

import rerun as rr

from nexus_link import NexusLink
from nexus_proto import NexusState

# --------------------------------------------------------------------------
# The estimator publishes at 1 kHz. Plots do not need that - the eye cannot use
# it and the viewer has to hold it all. Log every Nth packet for scalars.
#
# Raise it if you are chasing something fast (impact transients live around
# 100-200 Hz, so 1 keeps everything); lower it for long runs.
# --------------------------------------------------------------------------
DECIMATE = 5              # 1 kHz / 5 = 200 Hz of plotted data

JOINT_NAMES = [
    "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_spare",
    "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_spare",
]
CONTACT_NAMES = ["L_toe", "L_heel", "R_toe", "R_heel"]


def setup_styles() -> None:
    """
    Colours and names, logged once as static data.

    Static means "true for all time" - it is not part of the timeline and does
    not repeat per frame. Styling belongs here; measurements do not.
    """
    for i, name in enumerate(JOINT_NAMES):
        # Left leg cool, right leg warm, so a mirrored gait is obvious at a glance.
        colour = [80, 140, 220] if i < 5 else [220, 120, 60]
        rr.log(f"joints/pos/{i}", rr.SeriesLines(colors=colour, names=name),
               static=True)
        rr.log(f"joints/ref/{i}",
               rr.SeriesLines(colors=[150, 150, 150], names=f"{name} ref"),
               static=True)

    for i, name in enumerate(CONTACT_NAMES):
        rr.log(f"contact/{i}", rr.SeriesLines(colors=[60, 180, 110], names=name),
               static=True)

    rr.log("estimator/height",
           rr.SeriesLines(colors=[230, 180, 60], names="pelvis_z"), static=True)
    for i, axis in enumerate(("lateral", "forward", "vertical")):
        rr.log(f"estimator/vel/{axis}",
               rr.SeriesLines(names=f"vel_{axis}"), static=True)


def log_packet(pkt: NexusState) -> None:
    """Log one state packet onto the robot timeline."""

    # seq counts 1 kHz ticks, so seq/1000 is seconds since the STM32 booted.
    # Preferred over timestamp_us, which wraps every 71 minutes.
    rr.set_time("robot", duration=pkt.seq / 1000.0)

    # ---- what the policy consumes -------------------------------------
    rr.log("estimator/height", rr.Scalars(pkt.pelvis_z))
    for axis, v in zip(("lateral", "forward", "vertical"), pkt.vel_hdg):
        rr.log(f"estimator/vel/{axis}", rr.Scalars(v))

    for i in range(10):
        rr.log(f"joints/pos/{i}", rr.Scalars(pkt.joint_pos[i]))
        rr.log(f"joints/vel/{i}", rr.Scalars(pkt.joint_vel[i]))
        rr.log(f"joints/ref/{i}", rr.Scalars(pkt.ref_angle[i]))

    for i in range(4):
        rr.log(f"springs/{i}", rr.Scalars(pkt.spring_angle[i]))
        rr.log(f"contact/{i}", rr.Scalars(pkt.contact[i]))

    rr.log("gait/phase", rr.Scalars(pkt.phase))
    rr.log("feet/right_z", rr.Scalars(pkt.foot_z[0]))
    rr.log("feet/left_z", rr.Scalars(pkt.foot_z[1]))

    # ---- raw IMU, for cross-checking the estimator ---------------------
    for axis, v in zip("xyz", pkt.imu_gyro):
        rr.log(f"imu/gyro/{axis}", rr.Scalars(v))
    for axis, v in zip("xyz", pkt.imu_accel):
        rr.log(f"imu/accel/{axis}", rr.Scalars(v))

    # ---- health, as numbers you can plot against everything else -------
    rr.log("health/fused_valid", rr.Scalars(float(pkt.fused_valid)))
    rr.log("health/faults", rr.Scalars(float(pkt.health)))

    # ---- 3D ------------------------------------------------------------
    #
    # The pelvis pose, and the feet hanging off it. Enough to see the robot
    # move; not a URDF. Add link geometry later if you want it to look like
    # the robot rather than like a frame.
    #
    # Our quaternion is w,x,y,z. Rerun wants x,y,z,w.
    w, x, y, z = pkt.quat
    rr.log("world/pelvis",
           rr.Transform3D(translation=[pkt.fused_pos[0],
                                       pkt.fused_pos[1],
                                       pkt.pelvis_z],
                          rotation=rr.Quaternion(xyzw=[x, y, z, w])))

    # Feet in world coordinates. foot_z is already world height; x and y are
    # not sent, so they sit under the pelvis - vertical motion is the part
    # worth watching during a step anyway.
    rr.log("world/foot_right",
           rr.Points3D([[pkt.fused_pos[0], pkt.fused_pos[1] - 0.1,
                         pkt.foot_z[0]]],
                       colors=[[220, 120, 60]], radii=0.02))
    rr.log("world/foot_left",
           rr.Points3D([[pkt.fused_pos[0], pkt.fused_pos[1] + 0.1,
                         pkt.foot_z[1]]],
                       colors=[[80, 140, 220]], radii=0.02))


def log_sim(joint_pos, height: Optional[float] = None,
            t: Optional[float] = None) -> None:
    """
    Log the simulator's joint angles beside the real ones.

    Call this from your MuJoCo loop with the same joint ordering. Because these
    land under `sim/...` on the same timeline as `joints/...`, dropping both
    into one plot shows the sim-to-real gap directly - which is the single most
    useful thing this whole file exists to produce. A policy that works in
    simulation and not on hardware shows the divergence here first, long before
    the robot tells you by falling over.
    """
    if t is not None:
        rr.set_time("robot", duration=t)

    for i, q in enumerate(joint_pos):
        rr.log(f"sim/joints/{i}", rr.Scalars(float(q)))
    if height is not None:
        rr.log("sim/height", rr.Scalars(float(height)))


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="Live view of the robot in Rerun.")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0",
                    help="serial device the STM32 enumerates as")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--connect", metavar="HOST",
                   help="stream to a Rerun viewer running on HOST "
                        "(start it there with: rerun)")
    g.add_argument("--save", metavar="FILE.rrd",
                   help="record to a file to open later")
    g.add_argument("--spawn", action="store_true",
                   help="open a viewer on THIS machine (needs a display)")
    a = ap.parse_args()

    rr.init("zeus")

    if a.connect:
        url = f"rerun+http://{a.connect}:9876/proxy"
        rr.connect_grpc(url)
        print(f"streaming to {url}")
        print("  if nothing appears, the viewer is not running there, or 9876 "
              "is firewalled")
    elif a.save:
        rr.save(a.save)
        print(f"recording to {a.save} - copy it over and open with: rerun {a.save}")
    elif a.spawn:
        rr.spawn()
    else:
        ap.error("choose one of --connect HOST, --save FILE.rrd, or --spawn")

    setup_styles()

    print(f"reading {a.port} - ctrl-c to stop")
    port = a.port

    n = 0
    last_seq = None
    dropped = 0

    with NexusLink(port) as link:
        try:
            while True:
                pkt = link.latest()
                if pkt is None:
                    time.sleep(0.001)
                    continue

                # latest() returns the newest packet, so the same one appears
                # repeatedly if we poll faster than 1 kHz. Only log new ones.
                if pkt.seq == last_seq:
                    time.sleep(0.0005)
                    continue

                if last_seq is not None:
                    gap = (pkt.seq - last_seq) & 0xFFFFFFFF
                    if gap > 1:
                        dropped += gap - 1

                last_seq = pkt.seq
                n += 1

                if (n % DECIMATE) == 0:
                    log_packet(pkt)

                if (n % 5000) == 0:
                    print(f"  {n} packets, {dropped} skipped by the policy loop, "
                          f"link {link.stats}")

        except KeyboardInterrupt:
            print(f"\nstopped after {n} packets")

    return 0


if __name__ == "__main__":
    sys.exit(main())
