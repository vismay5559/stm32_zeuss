#!/usr/bin/env python3
"""
Prove the Pi side can actually receive at 1 kHz, without hardware.

The firmware guarantees 1000 packets/s onto the wire. Whether they reach a
Python control loop is a separate question, and the answer depends entirely on
how fast the parser is - a reader that cannot keep up does not error, it just
falls further behind every second while looking fine.

So this feeds NexusLink a synthetic 1 kHz stream through a fake serial port and
checks that every packet comes out, in order, fast enough.

    python tools/test_pi_link.py
"""

import struct
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "pi"))

import nexus_proto as P                                        # noqa: E402
from nexus_proto import NexusState, STATE_SIZE, crc16, _crc16_slow  # noqa: E402
import nexus_link                                              # noqa: E402
from nexus_link import NexusLink                               # noqa: E402

FAILURES = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not cond:
        FAILURES.append(name)


# --------------------------------------------------------------------------
# Synthetic packets
# --------------------------------------------------------------------------

def make_packet(seq, height=0.65, fused_valid=P.FUSION_OK, contacts=0x30):
    """Build one wire-format state packet, exactly as the STM32 would."""
    body = struct.pack(
        P.STATE_FORMAT[:-1],          # everything but the trailing crc
        P.SYNC, P.MSG_STATE, P.PROTO_VERSION,
        seq, seq * 1000,              # seq, timestamp_us
        1.0, 0.0, 0.0, 0.0,           # imu_quat
        0.0, 0.0, 0.0,                # imu_accel
        0.0, 0.0, 0.0,                # imu_gyro
        seq // 3,                     # imu_seq - 400 Hz against a 1 kHz tick
        *([0.5] * P.NUM_ENCODERS),    # enc_angle
        *([0.0] * P.NUM_JOINTS),      # act_pos
        *([0.0] * P.NUM_JOINTS),      # act_vel
        *([1.5] * P.NUM_JOINTS),      # act_torque
        *([0] * P.NUM_JOINTS),        # act_error
        1.0, 0.0, 0.0, 0.0,           # fused_quat
        0.0, 0.0, height,             # fused_pos
        0.0, 0.0, 0.0,                # fused_vel
        0.0, 0.0, 0.0,                # fused_gyro_bias
        0.0, 0.0, 0.0,                # fused_accel_bias
        100, 100,                     # contact_ticks
        *([8] * P.NUM_JOINTS),        # act_state
        *([3] * P.NUM_JOINTS),        # act_flags
        0x0F,                         # enc_valid
        contacts,
        fused_valid,
        0x00,                         # health
    )
    return body + struct.pack("<H", crc16(body))


class FakePort:
    """A serial.Serial stand-in fed from a background writer."""

    def __init__(self, *a, **kw):
        self._buf = bytearray()
        self._lock = threading.Lock()
        self.written = bytearray()
        self.closed = False

    def feed(self, data):
        with self._lock:
            self._buf += data

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._buf)

    def read(self, n=1):
        # Mimic a blocking read with a timeout: wait briefly for at least one
        # byte, then hand over up to n.
        deadline = time.monotonic() + 0.005
        while True:
            with self._lock:
                if self._buf:
                    out = bytes(self._buf[:n])
                    del self._buf[:n]
                    return out
            if time.monotonic() > deadline or self.closed:
                return b""
            time.sleep(0.0002)

    def write(self, data):
        self.written += data
        return len(data)

    def reset_input_buffer(self):
        with self._lock:
            self._buf.clear()

    def close(self):
        self.closed = True


# --------------------------------------------------------------------------

def test_crc():
    print("\nCRC")
    import os
    same = all(crc16(d) == _crc16_slow(d)
               for d in (b"", b"\x00", os.urandom(7), os.urandom(324), os.urandom(325)))
    check("fast crc16 matches the reference transcription", same)

    pkt = make_packet(1)
    check("packet crc verifies", crc16(pkt[:-2]) == struct.unpack("<H", pkt[-2:])[0])


def test_parse():
    print("\nParsing")
    pkt = make_packet(42, height=0.65)
    st = NexusState.parse(pkt)
    check("round-trips", st is not None)
    check("seq preserved", st.seq == 42, f"got {st.seq}")
    check("height preserved", abs(st.height - 0.65) < 1e-6, f"got {st.height:.4f}")
    check("fusion_usable", st.fusion_usable is True)
    check("both feet down", st.left_foot_down and st.right_foot_down)
    check("no faults", st.faults() == [])
    check("torque preserved", abs(st.act_torque[3] - 1.5) < 1e-6)
    check("encoders preserved", abs(st.enc_angle[2] - 0.5) < 1e-6)

    bad = bytearray(pkt)
    bad[60] ^= 0xFF
    check("corrupt payload rejected", NexusState.parse(bytes(bad)) is None)

    st2 = NexusState.parse(make_packet(1, fused_valid=P.FUSION_CONVERGING))
    check("CONVERGING is not usable", st2.fusion_usable is False)


def test_framing():
    print("\nFraming")
    buf = bytearray(b"\xde\xad\xbe\xef" + make_packet(7))
    pkt, buf = NexusState.find_and_parse(buf)
    check("skips junk before sync", pkt is not None and pkt.seq == 7)

    # Split a packet across two reads, as USB genuinely does.
    whole = make_packet(9)
    buf = bytearray(whole[:100])
    pkt, buf = NexusState.find_and_parse(buf)
    check("partial packet yields nothing", pkt is None)
    check("partial packet is retained", len(buf) == 100, f"kept {len(buf)}")
    buf += whole[100:]
    pkt, buf = NexusState.find_and_parse(buf)
    check("completes across reads", pkt is not None and pkt.seq == 9)

    buf = bytearray(b"".join(make_packet(i) for i in range(5)))
    got = []
    while True:
        pkt, buf = NexusState.find_and_parse(buf)
        if pkt is None:
            break
        got.append(pkt.seq)
    check("drains a burst in order", got == [0, 1, 2, 3, 4], str(got))
    check("buffer emptied", len(buf) == 0)

    # A payload byte pair that happens to equal the sync word must not derail
    # framing - the parser has to try the next byte, not skip a packet.
    buf = bytearray(b"\xa5\xa5" + make_packet(11))
    pkt, _ = NexusState.find_and_parse(buf)
    check("false sync word recovered", pkt is not None and pkt.seq == 11)


def test_throughput():
    print("\nThroughput (parse only)")
    stream = bytearray(b"".join(make_packet(i) for i in range(1000)))

    t = time.perf_counter()
    n = 0
    buf = stream
    while True:
        pkt, buf = NexusState.find_and_parse(buf)
        if pkt is None:
            break
        n += 1
    dt = time.perf_counter() - t

    load = dt * 100.0          # 1000 packets is exactly one second of stream
    print(f"        1000 packets in {dt * 1000:.1f} ms = {load:.1f}% of one core at 1 kHz")
    check("parses 1000 packets", n == 1000, f"got {n}")
    check("parse load under 25% of a core", load < 25.0, f"{load:.1f}%")


def test_link_1khz():
    print("\nEnd-to-end at 1 kHz through NexusLink")

    fake = FakePort()
    real_serial = nexus_link.serial.Serial
    nexus_link.serial.Serial = lambda *a, **kw: fake

    try:
        link = NexusLink("/dev/fake", history=4000)
        link.start()

        total = 2000                       # two seconds of stream
        # Feed in 1 ms chunks so the reader sees a genuine 1 kHz arrival
        # pattern rather than one giant block it can parse at leisure.
        for i in range(total):
            fake.feed(make_packet(i))
            time.sleep(0.001)

        # Let the reader finish what is still queued.
        deadline = time.monotonic() + 2.0
        while link.stats.packets < total and time.monotonic() < deadline:
            time.sleep(0.01)

        got = link.drain()
        stats = link.stats

        check("every packet received", stats.packets == total,
              f"{stats.packets}/{total}")
        check("no sequence gaps", stats.seq_gaps == 0, f"{stats.seq_gaps} gaps")
        check("no junk bytes", stats.junk_bytes == 0, f"{stats.junk_bytes} B")
        check("loss rate zero", stats.loss_rate == 0.0)
        check("history holds them in order",
              [p.seq for p in got] == list(range(total)),
              f"{len(got)} packets")
        check("latest() is the newest", link.latest().seq == total - 1,
              f"seq {link.latest().seq}")

        # The command path, which the policy uses at 250 Hz.
        link.send_command([0.25] * P.NUM_JOINTS)
        check("command is one packet", len(fake.written) == P.COMMAND_SIZE,
              f"{len(fake.written)} B")
        check("command crc valid",
              crc16(bytes(fake.written[:-2])) ==
              struct.unpack("<H", bytes(fake.written[-2:]))[0])

        link.stop()
    finally:
        nexus_link.serial.Serial = real_serial


def test_seq_gap_detection():
    print("\nGap detection")
    fake = FakePort()
    real_serial = nexus_link.serial.Serial
    nexus_link.serial.Serial = lambda *a, **kw: fake

    try:
        link = NexusLink("/dev/fake")
        link.start()

        for i in (0, 1, 2, 7, 8):          # 3,4,5,6 missing = 4 lost
            fake.feed(make_packet(i))
        time.sleep(0.2)

        check("gap counted once", link.stats.seq_gaps == 1, f"{link.stats.seq_gaps}")
        check("four packets reported lost", link.stats.seq_lost == 4,
              f"{link.stats.seq_lost}")
        link.stop()
    finally:
        nexus_link.serial.Serial = real_serial


def main():
    print(f"state packet {STATE_SIZE} B, command {P.COMMAND_SIZE} B, "
          f"{STATE_SIZE * 1000 / 1000:.0f} kB/s at 1 kHz")

    test_crc()
    test_parse()
    test_framing()
    test_throughput()
    test_link_1khz()
    test_seq_gap_detection()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: {', '.join(FAILURES)}")
        return 1
    print("all checks passed - the Pi side sustains 1 kHz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
