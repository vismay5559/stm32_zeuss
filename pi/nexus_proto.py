"""
Python side of the STM32 <-> Raspberry Pi link.

This is the counterpart of Appli/App/link_proto.h and MUST stay identical to
it. tools/check_proto.py compares every field offset and the total size against
the C header; run it after touching either file.

Drop this into zeus_26 (e.g. zeus_can_interface/ or a shared package) and parse
the bytes coming off the serial device. The STM32 owns all real-time sensing
and state estimation - by the time a packet arrives, height and velocity are
already estimated and the Pi only has to run the policy.

Typical use:

    import serial
    from nexus_proto import NexusState, STATE_SIZE, NexusCommand

    port = serial.Serial('/dev/ttyACM0', timeout=0.1)
    buf = bytearray()
    while True:
        buf += port.read(4096)
        pkt, buf = NexusState.find_and_parse(buf)
        if pkt is None:
            continue
        if pkt.fused_valid == FUSION_OK:
            height = pkt.fused_pos[2]
            vel    = pkt.fused_vel
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

# --------------------------------------------------------------------------
# Constants - keep in step with link_proto.h
# --------------------------------------------------------------------------

SYNC = 0xA5A5
PROTO_VERSION = 2

MSG_STATE = 0x01
MSG_COMMAND = 0x02

NUM_JOINTS = 10
NUM_ENCODERS = 4

CONTACT_L_TOE = 1 << 0
CONTACT_L_HEEL = 1 << 1
CONTACT_R_TOE = 1 << 2
CONTACT_R_HEEL = 1 << 3
CONTACT_L_FOOT = 1 << 4
CONTACT_R_FOOT = 1 << 5

FUSION_INVALID = 0
FUSION_CONVERGING = 1
FUSION_OK = 2

ACT_TELEM_FRESH = 1 << 0
ACT_HB_FRESH = 1 << 1

CMD_ENABLE = 1 << 0

# Health bitmask, mirrors Appli/App/health.h
HEALTH_IMU = 1 << 0
HEALTH_ENC = 1 << 1
HEALTH_CAN1 = 1 << 2
HEALTH_CAN2 = 1 << 3
HEALTH_LINK = 1 << 4
HEALTH_TIMING = 1 << 5

HEALTH_NAMES = {
    HEALTH_IMU: "imu",
    HEALTH_ENC: "encoders",
    HEALTH_CAN1: "can1",
    HEALTH_CAN2: "can2",
    HEALTH_LINK: "pi-link",
    HEALTH_TIMING: "loop-timing",
}

# --------------------------------------------------------------------------
# Wire layout
#
# '<' little-endian, no padding - matches __attribute__((packed)) on the C side.
# The C struct is ordered so every 4-byte field lands on a 4-byte boundary, so
# numpy can also view the buffer directly if you prefer that to struct.unpack.
# --------------------------------------------------------------------------

STATE_FORMAT = (
    "<"
    "H"      # sync
    "B"      # msg_id
    "B"      # version
    "I"      # seq
    "I"      # timestamp_us
    "4f"     # imu_quat        w, x, y, z
    "3f"     # imu_accel       m/s^2
    "3f"     # imu_gyro        rad/s
    "I"      # imu_seq
    "4f"     # enc_angle       rad, after-spring
    "10f"    # act_pos         turns
    "10f"    # act_vel         turns/s
    "10f"    # act_torque      Nm
    "10I"    # act_error
    "4f"     # fused_quat
    "3f"     # fused_pos       [2] is height
    "3f"     # fused_vel
    "3f"     # fused_gyro_bias
    "3f"     # fused_accel_bias
    "2H"     # contact_ticks
    "10B"    # act_state
    "10B"    # act_flags
    "B"      # enc_valid
    "B"      # contacts
    "B"      # fused_valid
    "B"      # health
    "H"      # crc
)
STATE_SIZE = struct.calcsize(STATE_FORMAT)

COMMAND_FORMAT = (
    "<"
    "H"      # sync
    "B"      # msg_id
    "B"      # version
    "I"      # seq
    "10f"    # target_pos      turns
    "H"      # flags
    "H"      # crc
)
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)


def crc16(data: bytes) -> int:
    """CRC16-CCITT, init 0xFFFF, poly 0x1021 - same as nexus_crc16() in C."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------------------


@dataclass
class NexusState:
    """One 1 kHz state packet from the STM32."""

    seq: int
    timestamp_us: int

    # IMU, raw
    imu_quat: List[float]        # w, x, y, z
    imu_accel: List[float]       # m/s^2, gravity removed
    imu_gyro: List[float]        # rad/s
    imu_seq: int

    # After-spring joint angles
    enc_angle: List[float]       # rad
    enc_valid: int               # bit per encoder

    # Actuators
    act_pos: List[float]         # turns
    act_vel: List[float]         # turns/s
    act_torque: List[float]      # Nm
    act_error: List[int]
    act_state: List[int]
    act_flags: List[int]

    # Fused state of the lower torso - what the policy actually wants
    fused_quat: List[float]      # w, x, y, z, body -> world
    fused_pos: List[float]       # m, world; [2] is height above ground
    fused_vel: List[float]       # m/s, world
    fused_gyro_bias: List[float]
    fused_accel_bias: List[float]
    fused_valid: int

    contacts: int
    contact_ticks: List[int]
    health: int

    # ---- convenience ----------------------------------------------------

    @property
    def height(self) -> float:
        """Height of the lower torso above the ground, metres."""
        return self.fused_pos[2]

    @property
    def velocity(self) -> List[float]:
        """World-frame velocity of the lower torso, m/s."""
        return self.fused_vel

    @property
    def left_foot_down(self) -> bool:
        return bool(self.contacts & CONTACT_L_FOOT)

    @property
    def right_foot_down(self) -> bool:
        return bool(self.contacts & CONTACT_R_FOOT)

    @property
    def fusion_usable(self) -> bool:
        """True only once the estimator reports it has converged. Treat height
        and velocity as meaningless before this - the filter starts with a
        30 degree orientation and 1 m/s velocity uncertainty."""
        return self.fused_valid == FUSION_OK

    def faults(self) -> List[str]:
        """Names of subsystems the STM32 is reporting as unhealthy."""
        return [n for bit, n in HEALTH_NAMES.items() if self.health & bit]

    # ---- parsing --------------------------------------------------------

    @classmethod
    def parse(cls, raw: bytes) -> Optional["NexusState"]:
        """Parse exactly one packet. Returns None if it fails any check."""
        if len(raw) != STATE_SIZE:
            return None

        f = struct.unpack(STATE_FORMAT, raw)
        if f[0] != SYNC or f[1] != MSG_STATE or f[2] != PROTO_VERSION:
            return None
        if f[-1] != crc16(raw[:-2]):
            return None

        i = 3
        def take(n):
            nonlocal i
            out = f[i:i + n]
            i += n
            return list(out)

        seq, ts = f[3], f[4]
        i = 5
        return cls(
            seq=seq,
            timestamp_us=ts,
            imu_quat=take(4),
            imu_accel=take(3),
            imu_gyro=take(3),
            imu_seq=take(1)[0],
            enc_angle=take(4),
            act_pos=take(10),
            act_vel=take(10),
            act_torque=take(10),
            act_error=take(10),
            fused_quat=take(4),
            fused_pos=take(3),
            fused_vel=take(3),
            fused_gyro_bias=take(3),
            fused_accel_bias=take(3),
            contact_ticks=take(2),
            act_state=take(10),
            act_flags=take(10),
            enc_valid=take(1)[0],
            contacts=take(1)[0],
            fused_valid=take(1)[0],
            health=take(1)[0],
        )

    @classmethod
    def find_and_parse(cls, buf: bytearray) -> Tuple[Optional["NexusState"], bytearray]:
        """
        Pull the first valid packet out of a byte stream.

        Returns (packet_or_None, remaining_buffer). USB is lossless and
        packet-framed, so resync is rare - but a reconnect mid-packet will
        leave junk, and scanning for the sync word recovers from it.
        """
        lo = SYNC & 0xFF
        hi = (SYNC >> 8) & 0xFF

        start = 0
        while start + STATE_SIZE <= len(buf):
            if buf[start] == lo and buf[start + 1] == hi:
                pkt = cls.parse(bytes(buf[start:start + STATE_SIZE]))
                if pkt is not None:
                    return pkt, buf[start + STATE_SIZE:]
            start += 1

        # Keep the tail that might be the start of a packet.
        keep = max(0, len(buf) - STATE_SIZE)
        return None, buf[keep:]


@dataclass
class NexusCommand:
    """Position command to the STM32. Send at ~250 Hz."""

    seq: int = 0
    target_pos: Optional[List[float]] = None   # turns, NUM_JOINTS
    flags: int = 0

    def pack(self) -> bytes:
        pos = self.target_pos if self.target_pos is not None else [0.0] * NUM_JOINTS
        if len(pos) != NUM_JOINTS:
            raise ValueError(f"target_pos must have {NUM_JOINTS} entries")

        body = struct.pack(
            COMMAND_FORMAT[:-1],       # everything except the trailing crc
            SYNC, MSG_COMMAND, PROTO_VERSION, self.seq, *pos, self.flags,
        )
        return body + struct.pack("<H", crc16(body))


if __name__ == "__main__":
    print(f"state packet   : {STATE_SIZE} bytes")
    print(f"command packet : {COMMAND_SIZE} bytes")
    print(f"at 1 kHz       : {STATE_SIZE * 1000 / 1024:.1f} KiB/s up")
