"""Decoder for the IMU_LOG binary stream (Appli/App/test_imu_log.c).

    A5 5A  type  len  payload[len]  sum8(type, len, payload)

type 1 (len 28): u16 frame, u8 n_accel, u8 n_gyro, f32 accel[3], f32 gyro[3]
type 2 (len 20): f32 sensor gyro bias[3], u32 console bytes dropped, u32 ms

Startup text is interleaved with the frames, so the decoder hunts for the
header and trusts a frame only if its checksum agrees. A false header inside
a float is possible; the checksum is what stops it being believed.
"""
import struct

import numpy as np

FRAME_HZ = 200.0
_LEN = {1: 28, 2: 20}


def decode(buf):
    """Return (fast, slow, rejected): fast is an (n, 7) array of
    [frame_counter_unwrapped, ax, ay, az, gx, gy, gz]; slow is (m, 5) of
    [bias_x, bias_y, bias_z, dropped, ms]."""
    fast, slow = [], []
    rejected = 0
    i, n = 0, len(buf)
    wrap = 0
    prev = None
    while i + 5 <= n:
        if buf[i] != 0xA5 or buf[i + 1] != 0x5A:
            i += 1
            continue
        t, ln = buf[i + 2], buf[i + 3]
        if _LEN.get(t) != ln or i + 5 + ln > n:
            if i + 5 + ln > n and _LEN.get(t) == ln:
                break                       # incomplete tail: stop, keep it
            i += 1
            continue
        body = buf[i + 4:i + 4 + ln]
        if (t + ln + sum(body)) & 0xFF != buf[i + 4 + ln]:
            rejected += 1
            i += 1
            continue
        if t == 1:
            c, _, _ = struct.unpack_from("<HBB", body, 0)
            v = struct.unpack_from("<6f", body, 4)
            if prev is not None and c < prev:
                wrap += 65536
            prev = c
            fast.append((c + wrap,) + v)
        else:
            slow.append(struct.unpack_from("<3fII", body, 0))
        i += 5 + ln
    return np.array(fast).reshape(-1, 7), np.array(slow).reshape(-1, 5), rejected, i
