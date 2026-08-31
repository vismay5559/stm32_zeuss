# CAN bus bring-up test — one assembled leg

Run every test here, in order, before the leg is allowed to run a trajectory.
Each has a pass criterion you can read off the console. Stop at the first
failure; they are ordered so that an early failure makes the later results
meaningless.

Tests 0–3 run with **motors off** (`LEGTEST_ENABLE_CLOSED_LOOP 0`). Tests 4–5
energise one axis at a time and are the first time anything can move.

Host: PuTTY logging *All session output*, **Always append**, to `putty.log`.

---

## Node map

| joint | node | gearbox | encoder measures | command scaling |
|---|---|---|---|---|
| hip pitch | 1 | 47:1 | **load** side (output shaft) | 1:1 — send output turns |
| hip roll | 2 | 47:1 | load side | **not working, excluded** |
| knee pitch | 3 | 47:1 | **load** side (output shaft) | 1:1 — send output turns |
| ankle pitch | 4 | 9:1 | **MOTOR** side | **x9** — send 9 × output turns |

> **UNRESOLVED.** The brief said "hip pitch, knee pitch, ankle pitch on CAN ids
> 1, 3, 4 respectively" and then "(2 is for knee pitch)". Those contradict. This
> table assumes node 2 is the non-working hip roll. Test 4 confirms it by
> commanding one axis at a time and watching which joint moves; that cannot be
> done earlier because **hip and knee are non-backdrivable** and will not move
> by hand.

## Mechanical limits

| joint | limit | gait range | margin | verdict |
|---|---|---|---|---|
| hip pitch | ±25° | −20.53° .. −6.71° | **4.47°** | tight |
| knee pitch | ±35° | +25.53° .. +30.23° | **4.77°** | tight |
| ankle pitch | ±35° | +6.03° .. +21.41° | 13.59° | ok |

Those margins assume each joint's zero is exactly where its encoder's zero is.
A 5° zeroing error puts hip pitch or knee into its stop on the first cycle.
Test 5 exists because of this.

---

## Test 0 — physical, power off

| # | check | pass |
|---|---|---|
| 0.1 | Resistance CAN_H to CAN_L, everything powered down | **60 Ω** (two terminators in parallel) |
| 0.2 | Termination at the two physical **ends** of the bus, not on a stub | visual |
| 0.3 | Continuity of CAN_H, CAN_L and **ground**, end to end | < 1 Ω |
| 0.4 | Stub length from each ODrive to the trunk | < 30 cm |
| 0.5 | Connectors seated; no strain on wire at the leg's moving joints | visual |

60 Ω is the one number to care about. 120 Ω means a terminator is missing;
40 Ω means someone fitted three.

> A bench run at 15% bus load went **BUS-OFF** mid-gait with a single node
> attached. That fault is still unexplained, and it is why 0.1–0.5 come first.
> Three nodes on a moving leg is a strictly harsher environment than the bench
> that already failed.

## Test 1 — passive scan, nobody transmitting

Power all ODrives. The firmware's startup scan listens for `LEGTEST_SCAN_MS`
(2000 ms) without sending anything.

```
scanning the bus for 2000 ms - not transmitting...
  node 1    ~2000 frames  axis_state 1  axis_error 0x00000000   <-- configured
  node 3    ~2000 frames  axis_state 1  axis_error 0x00000000   <-- configured
  node 4    ~2000 frames  axis_state 1  axis_error 0x00000000   <-- configured
```

| # | check | pass |
|---|---|---|
| 1.1 | Every expected node appears | 1, 3, 4 present |
| 1.2 | `axis_error` on each | `0x00000000` |
| 1.3 | `axis_state` on each | `1` (IDLE) — nothing should be armed yet |
| 1.4 | Any **unexpected** node id appears | investigate before continuing |
| 1.5 | `TEC` / `REC` after the scan | both `0` |

A missing node is powered down, has the wrong `node_id`, or is not publishing
cyclically. Fix it before Test 2 — an absent node cannot be tested later, and
its absence stops being obvious once the console is busy.

## Test 2 — telemetry rate and integrity

The per-second status line carries **cumulative** counters. The rate is the
difference between two consecutive lines, not the number printed.

```
  node 1 hip_pitch : pos= ... state=1 err=0x00000000  hb=100 enc=3238 trq=320
  node 1 hip_pitch : pos= ... state=1 err=0x00000000  hb=200 enc=4268 trq=420
                                                        +100    +1030    +100
```

ODrive config, per axis:

```python
odrv0.axis0.config.can.encoder_msg_rate_ms   = 1     # 1000 Hz
odrv0.axis0.config.can.torque_msg_rate_ms    = 10    #  100 Hz
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 10    #  100 Hz
```

| # | check | pass |
|---|---|---|
| 2.1 | Encoder frames per second, each node | 950 – 1050 |
| 2.2 | Torque frames per second, each node | 95 – 105 |
| 2.3 | Heartbeats per second, each node | 95 – 105 |
| 2.4 | `---- SILENT ----` appears for any node | never |

Encoder at 1 kHz matches the command rate, so every setpoint can be compared
against a fresh measurement. Torque and heartbeat at 100 Hz match `CAPTURE_HZ`,
which is all the capture can record anyway.

> The startup banner prints message rates as a **reminder of what to configure**.
> It is not a readback — the board cannot query an ODrive's config over CAN.
> Only the counter deltas above tell you the real rate.

## Test 3 — two-minute soak, transmitting, motors off

Arm nothing. Let the firmware stream `Set_Input_Pos` at 1 kHz to all three nodes
with `LEGTEST_ENABLE_CLOSED_LOOP 0`, so the drives accept setpoints and act on
none of them. Run **120 seconds**.

| # | check | pass |
|---|---|---|
| 3.1 | `txfail` | `0` for the whole run |
| 3.2 | `qdrop` | `0` for the whole run |
| 3.3 | `TEC` and `REC` | `0` at every status line |
| 3.4 | `[WARNING]`, `[ERROR-PASSIVE]`, `[BUS-OFF]` | never appear |
| 3.5 | Bus utilisation | < 60% |
| 3.6 | Any node goes `---- SILENT ----` | never |

Then repeat 3.1–3.6 **while flexing the harness by hand** through the leg's full
range of motion. An intermittent conductor at a joint passes a static soak and
fails in service; this is the version of the test that finds it.

Expected load, three nodes, CAN-FD 1 Mbit arbitration / 2 Mbit data:

| direction | frames/s |
|---|---|
| STM32 → drives, `Set_Input_Pos` 1 kHz × 3 | 3 000 |
| drives → STM32, encoder 1 kHz × 3 | 3 000 |
| drives → STM32, torque 100 Hz × 3 | 300 |
| drives → STM32, heartbeat 100 Hz × 3 | 300 |
| **total** | **6 600** |

Roughly 33% at the firmware's 50 µs/frame estimate. If 3.5 fails anyway, drop
`encoder_msg_rate_ms` to 2 before touching the command rate — telemetry is for
logging, the 1 kHz command stream is what the control loop needs.

## Test 4 — identity, direction and gear ratio (**first motion**)

Hip and knee are non-backdrivable, so nothing here can be done by hand. Each
axis is armed on its own and given a small commanded move, and you watch which
joint physically responds.

**One node at a time. Hand on the stop button. Everything else in IDLE.**

For each node in turn:

| # | step | pass |
|---|---|---|
| 4.1 | Arm that node only; watch `state=` | `1` → `8` |
| 4.2 | `err=` immediately after arming | stays `0x00000000` |
| 4.3 | Command **+2° of output** from the present position | joint moves |
| 4.4 | **Which** joint moved | the one this document claims for that node |
| 4.5 | No other joint moved | else the map is wrong — **stop and correct it** |
| 4.6 | Direction matches the joint's positive convention | else the encoder is reversed |
| 4.7 | Return to start, disarm, `state=` | back to `1` |

+2° is chosen to be far inside every limit even if the map is wrong: the worst
case is 2° into the wrong joint, which no joint here can be damaged by.

Then check the scaling, which is where the two gearbox conventions separate:

| joint | command | expected Δ`pos` | expected physical move |
|---|---|---|---|
| hip pitch (1) | +0.00556 turns | +0.00556 | **+2° of output** |
| knee pitch (3) | +0.00556 turns | +0.00556 | **+2° of output** |
| ankle pitch (4) | +0.05000 turns | +0.05000 | **+2° of output** |

| # | check | pass |
|---|---|---|
| 4.8 | Hip and knee move 2° of output for 0.00556 turns commanded | ±0.5° |
| 4.9 | Ankle moves **2°**, not 18°, for 0.05 turns commanded | ±0.5° |

4.9 is the test that proves `s_cmd_scale[2] = 9.0f` is right. If the ankle moves
18°, its encoder is **not** on the motor side and the scale must be 1.0 — and
every ankle command would otherwise be 9× too large, into a ±35° stop.

## Test 5 — zero and range

The gait is **absolute**: sample 0 of hip pitch is −18.57°, not "wherever the leg
happens to be". Nothing in the firmware discovers where a joint's mechanical
zero is, so it has to be established and checked.

See `ZEROING.md` for how to set the zero. This test only verifies it.

| # | step | pass |
|---|---|---|
| 5.1 | Place the leg in the defined zero pose | by jig or fixture, not by eye |
| 5.2 | Read `pos=` on every node | each within ±0.5° of `0.0000` |
| 5.3 | Power cycle everything, return to the same pose, read `pos=` again | same values |
| 5.4 | Command each joint to its **positive** gait extreme | reaches it, no stop contact |
| 5.5 | Command each joint to its **negative** gait extreme | reaches it, no stop contact |
| 5.6 | Clearance to the mechanical stop at both extremes | ≥ 3° |

5.3 is the one people skip. If the reading changes after a power cycle, the zero
is not persistent and it has to be re-established every time the robot is
switched on — which is a different operating procedure, not a smaller one.

5.6 is arithmetic against Test 5.2's measured offsets, and it is the last gate
before a trajectory runs. Hip pitch has 4.47° of nominal margin and knee has
4.77°; if the measured offset eats more than that, **the gait cannot be run
as-is** — set `s_zero_offset` to re-centre it, or reduce the amplitude.

---

## Record for each run

Archive the log and note, per node: encoder Hz, torque Hz, heartbeat Hz, peak
`TEC`, peak bus %, `txfail`, `qdrop`, and any `axis_error`. A test whose result
was not written down has to be run again the first time someone asks.
