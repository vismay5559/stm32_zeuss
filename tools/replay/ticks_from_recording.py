#!/usr/bin/env python3
"""
Turn a recording of the board's state stream into per-tick estimator inputs
for replay_fusion.c - the exact firmware estimator, run on a laptop.

    python3 ticks_from_recording.py RECORDING OUT.ticks

RECORDING is either raw bytes off /dev/zeus_stm32 or a link-node flight
recording (.flt). Needs zeus_link's nexus_proto on the path (source the
zeus_8dof_deploy workspace).

One fixed-size record per state packet, little-endian (see replay_fusion.c,
which must agree): what the board fed fusion_tick() that tick, as far as the
packet carries it, plus what the board's estimator reported, to compare with.
"""
import math
import struct
import sys

from zeus_link import nexus_proto as P

REC = struct.Struct("<I3f3f8f8I8B4fBB" "f3fB")   # inputs | recorded outputs


def packets(path):
    data = open(path, "rb").read()
    if path.endswith(".flt"):
        from zeus_link.flight_log import read_records
        _, recs = read_records(path)
        data = b"".join(d for direction, _, d in recs if direction == 0)
    buf = bytearray(data)
    while True:
        pkt, buf = P.NexusState.find_and_parse(buf)
        if pkt is None:
            return
        yield pkt


def main():
    n = 0
    with open(sys.argv[2], "wb") as out:
        for p in packets(sys.argv[1]):
            turns = [q / (2.0 * math.pi) for q in p.joint_pos]   # act->pos is output turns
            fresh = [1 if f & P.ACT_TELEM_FRESH else 0 for f in p.act_flags]
            out.write(REC.pack(p.timestamp_us, *p.imu_accel, *p.imu_gyro, *turns,
                               *p.act_error, *fresh, *p.spring_angle, p.enc_valid,
                               p.contacts, p.pelvis_z, *p.fused_vel, p.fused_valid))
            n += 1
    print(f"{n} ticks -> {sys.argv[2]} ({REC.size} bytes each)")


if __name__ == "__main__":
    main()
