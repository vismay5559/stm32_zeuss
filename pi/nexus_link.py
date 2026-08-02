"""
1 kHz link to the STM32, for the Pi side.

nexus_proto.py defines the bytes; this file makes sure none of them are missed.

The STM32 puts a complete state packet on the wire every millisecond whether
anyone is listening or not. USB does not drop them - they queue up in a kernel
buffer. So the only way to lose 1 kHz data on the Pi is to read too slowly, and
a control loop running at 250 Hz reading one packet per step does exactly that:
it falls three packets further behind every step, forever.

The fix is to decouple reading from using. A background thread does nothing but
drain the port, so reception stays at 1 kHz regardless of what the policy is
doing. The policy then asks for the newest packet and ignores the rest.

    link = NexusLink('/dev/ttyACM0')
    link.start()

    while True:                      # policy at 250 Hz
        pkt = link.latest()
        if pkt is not None and pkt.fusion_usable:
            action = policy(build_obs(pkt))
            link.send_command(action_to_turns(action))

Discarding packets here is not a loss - it is the point. The newest packet is
the only one that is not already out of date. If you also want every packet
(logging, system identification, plotting), turn on the history buffer and call
drain(); reception is 1 kHz either way.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Deque, List, Optional, Sequence

import serial

from nexus_proto import NexusCommand, NexusState, STATE_SIZE


class LinkStats:
    """Counters for link health. Read them; the reader thread owns writing."""

    def __init__(self) -> None:
        self.packets = 0          # good packets parsed
        self.seq_gaps = 0         # times seq jumped by more than 1
        self.seq_lost = 0         # total packets implied missing by those jumps
        self.bytes_in = 0         # everything read off the port
        self.residual = 0         # bytes currently held as a partial packet
        self.first_us = 0
        self.last_us = 0

    @property
    def loss_rate(self) -> float:
        """Fraction of expected packets that never arrived.

        Non-zero means the STM32 dropped a send (its USB endpoint was still
        busy with the previous packet) or the Pi lost bytes. USB itself does
        not lose data, so this counts real gaps, not transport errors."""
        total = self.packets + self.seq_lost
        return (self.seq_lost / total) if total else 0.0

    @property
    def junk_bytes(self) -> int:
        """Bytes read that were never part of a valid packet.

        Should be zero, or a small one-off from connecting mid-packet. Growing
        steadily means framing trouble - wrong baud, a competing reader on the
        same tty, or a protocol version mismatch."""
        return self.bytes_in - (self.packets * STATE_SIZE) - self.residual

    def __str__(self) -> str:
        return (f"{self.packets} pkts, {self.loss_rate * 100:.3f}% lost "
                f"({self.seq_lost}), {self.junk_bytes} junk B")


class NexusLink:
    """
    Threaded 1 kHz reader for the STM32 link.

    port        serial device, /dev/ttyACM0 on a Pi
    history     if > 0, keep this many packets for drain(); 0 disables it
    on_packet   optional callback run in the reader thread for every packet.
                Keep it short - anything slow here delays the next read.
    """

    def __init__(self,
                 port: str = "/dev/ttyACM0",
                 history: int = 0,
                 on_packet: Optional[Callable[[NexusState], None]] = None) -> None:
        self._port_name = port
        self._on_packet = on_packet

        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        # Assigning a reference is atomic under the GIL, so latest() needs no
        # lock. The history deque does - drain() and the reader both mutate it.
        self._latest: Optional[NexusState] = None
        self._history: Optional[Deque[NexusState]] = deque(maxlen=history) if history else None
        self._hist_lock = threading.Lock()

        self._tx_lock = threading.Lock()
        self._cmd_seq = 0

        self.stats = LinkStats()

    # ---- lifecycle ------------------------------------------------------

    def start(self) -> None:
        self._ser = serial.Serial(self._port_name, timeout=0.005)

        # The tty layer normally waits up to 16 ms to batch small reads, which
        # would turn a 1 ms stream into 16 ms bursts. Not available on every
        # platform or driver, hence the guard - but check that it took, because
        # silently keeping the default is the difference between 1 ms and 16 ms
        # of latency and nothing else will tell you.
        try:
            self._ser.set_low_latency_mode(True)
        except (AttributeError, ValueError, OSError):
            pass

        self._ser.reset_input_buffer()

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="nexus-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "NexusLink":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # ---- reading --------------------------------------------------------

    def latest(self) -> Optional[NexusState]:
        """Newest packet, or None if nothing has arrived yet.

        This is what a control loop wants. It never blocks and never returns
        stale data in preference to fresh."""
        return self._latest

    def drain(self) -> List[NexusState]:
        """Every packet since the last call, oldest first. Requires history > 0."""
        if self._history is None:
            raise RuntimeError("NexusLink was constructed with history=0")
        with self._hist_lock:
            out = list(self._history)
            self._history.clear()
        return out

    def wait_for_fusion(self, timeout: float = 10.0) -> Optional[NexusState]:
        """Block until the estimator reports OK. Returns None on timeout.

        The filter starts with a 30 degree orientation and 1 m/s velocity
        uncertainty and takes ~2 s to converge, so height and velocity are
        meaningless until this returns. Call it once before starting a policy."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pkt = self._latest
            if pkt is not None and pkt.fusion_usable:
                return pkt
            time.sleep(0.005)
        return None

    def _run(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        prev_seq: Optional[int] = None

        while not self._stop.is_set():
            try:
                # read(1) blocks until a byte or the 5 ms timeout, so this loop
                # never spins hot; in_waiting then takes whatever else arrived
                # in the meantime, so a burst costs one syscall rather than one
                # per packet.
                chunk = self._ser.read(1)
                if not chunk:
                    continue
                extra = self._ser.in_waiting
                if extra:
                    chunk += self._ser.read(extra)
            except (serial.SerialException, OSError):
                break

            buf += chunk
            self.stats.bytes_in += len(chunk)

            # A single read can easily carry several packets - at 1 kHz any
            # scheduling hiccup batches them - so keep parsing until dry.
            # find_and_parse trims what it consumed and anything too short to
            # still become a packet, so buf cannot grow without bound.
            while True:
                pkt, buf = NexusState.find_and_parse(buf)
                if pkt is None:
                    break

                prev_seq = self._accept(pkt, prev_seq)

            self.stats.residual = len(buf)

    def _accept(self, pkt: NexusState, prev_seq: Optional[int]) -> int:
        """Record one good packet and its sequence continuity."""
        if prev_seq is not None:
            # seq is uint32 on the C side and increments once per 1 kHz tick,
            # so it wraps roughly every 50 days. Masking makes the wrap a
            # non-event rather than a 4-billion-packet loss report.
            gap = (pkt.seq - prev_seq) & 0xFFFFFFFF
            if gap != 1:
                self.stats.seq_gaps += 1
                self.stats.seq_lost += max(0, gap - 1)

        self.stats.packets += 1
        if self.stats.first_us == 0:
            self.stats.first_us = pkt.timestamp_us
        self.stats.last_us = pkt.timestamp_us

        self._latest = pkt

        if self._history is not None:
            with self._hist_lock:
                self._history.append(pkt)

        if self._on_packet is not None:
            self._on_packet(pkt)

        return pkt.seq

    # ---- writing --------------------------------------------------------

    def send_command(self, target_pos: Sequence[float], flags: int = 0) -> None:
        """Send a position command, in turns, one per joint.

        Send these at ~250 Hz. The STM32 interpolates between consecutive
        commands across its 1 kHz ticks, so sending faster gains nothing and
        sending slower makes each ramp longer. If commands stop, the last
        target is held rather than extrapolated."""
        if self._ser is None:
            raise RuntimeError("link not started")

        with self._tx_lock:
            cmd = NexusCommand(seq=self._cmd_seq, target_pos=list(target_pos), flags=flags)
            self._cmd_seq = (self._cmd_seq + 1) & 0xFFFFFFFF
            self._ser.write(cmd.pack())


# --------------------------------------------------------------------------
# Standalone check: run this on the Pi before wiring anything into zeus_26.
#
#   python nexus_link.py /dev/ttyACM0
#
# A healthy link prints ~1000.0 Hz and 0.000% lost. Anything less means the
# problem is on the Pi - scheduling, the tty layer, or a slow on_packet - and
# is much easier to find here than inside a control loop.
# --------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

    with NexusLink(dev) as link:
        print(f"reading {dev}, ctrl-c to stop\n")
        t0 = time.monotonic()
        last_pkts = 0

        try:
            while True:
                time.sleep(1.0)
                pkt = link.latest()
                now = time.monotonic()

                rate = (link.stats.packets - last_pkts) / (now - t0)
                last_pkts, t0 = link.stats.packets, now

                if pkt is None:
                    print("no packets - is the STM32 running and enumerated?")
                    continue

                fusion = ("OK" if pkt.fusion_usable
                          else ("CONVERGING" if pkt.fused_valid == 1 else "INVALID"))
                faults = ",".join(pkt.faults()) or "none"

                print(f"{rate:7.1f} Hz | seq {pkt.seq:9d} | {link.stats} | "
                      f"fusion {fusion} | h {pkt.height:+.3f} m | faults {faults}")
        except KeyboardInterrupt:
            print(f"\nfinal: {link.stats}")
