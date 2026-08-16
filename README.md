# nexus — STM32H7S3L8 real-time controller for a biped robot

Firmware for the low-level control board of a bipedal robot. The STM32 owns
everything that must happen on time; a Raspberry Pi handles planning and
perception and talks to this board over USB.

```
Raspberry Pi                      STM32H7S3L8 (this repo)
────────────                      ───────────────────────
Linux, ROS, planning,      ←→     exact 1 kHz control loop
vision, logging                   IMU, encoders, CAN-FD, contacts
(soft real-time)                  (hard real-time)

      one 422-byte struct every 1 ms — see Appli/App/link_proto.h
```

**Start here:** [What the Pi receives](#what-the-pi-receives) is the contract
between this board and the policy. [Test modes](#test-modes) is how to bring one
subsystem up at a time.

---

## Hardware

| Item | Detail |
|---|---|
| MCU | STM32H7S3L8Hx, Cortex-M7 @ 600 MHz, TFBGA225 |
| Board | NUCLEO-H7S3L8 |
| External flash | Macronix MX25UW25645G, 256 Mbit octal, on XSPI2 |
| IMU | BNO085, SHTP over UART1 @ 3 Mbaud, 400 Hz |
| Encoders | 4 × AS5048A, daisy-chained on SPI1 @ 6.25 MHz, 1 kHz |
| Actuators | 10 × ODrive S1 — 5 per bus on 2 × FDCAN |
| Contacts | 4 × mechanical foot switches (toe/heel, both feet) |
| Host link | USB High Speed (480 Mbit), CDC |

### Verified clock tree

HSE 24 MHz → PLL1 (M2 N100 P2) → **600 MHz** CPU, 300 MHz AHB, 150 MHz APB.

| Function | Source | Result |
|---|---|---|
| Control tick (TIM6) | 300 MHz / 300 / 1000 | 1.0000 kHz |
| Timestamp (TIM2) | 300 MHz / 300, 32-bit | 1 MHz, wraps @ 71.6 min |
| FDCAN | PLL2P = 80 MHz | 1 Mbit nominal / 5 Mbit data |
| SPI1 | PLL1Q = 100 MHz / 16 | 6.25 MHz, mode 1 |
| USART1 | PCLK2 150 MHz, OVER8 | 3.000 Mbaud, 0% error |
| XSPI2 | PLL2S = 400 MHz | see *Known issues* |

---

## How it boots

The MCU has only **64 KB of internal flash** and the application is ~75 KB, so
this is a two-binary execute-in-place (XIP) design:

```
power on
   │
   ▼
Boot  (internal flash, 0x08000000)
   configures MPU, clocks, XSPI2; maps the external flash
   │  jumps to
   ▼
Appli (external flash, 0x70000000)
   the robot firmware, executed in place
```

Both are built from one CubeMX project (`nexus_first.ioc`) with separate
contexts. **They must be flashed separately** — see below.

---

## Layout

```
Appli/App/            ← the actual robot code
   app.c                1 kHz control loop, health LEDs
   health.c/.h          subsystem health + red-LED blink codes
   imu_bno085.c         BNO085 SHTP-over-UART driver
   enc_as5048a.c        AS5048A SPI encoder driver
   act_odrive.c         ODrive CANSimple, software TX queue
   contact.c            foot switch debouncing
   link_usb.c           framing/CRC for the Pi link
   link_proto.h         wire format — keep byte-identical with the Pi
   critical.h           ISR-safe copy helper

   lie_group.c/.h       SE_K(3) Lie group maths (fixed size)
   kinematics.c/.h      leg forward kinematics + Jacobian
   inekf.c/.h           contact-aided right-invariant EKF
   fusion.c/.h          bridges sensors <-> filter; fills the policy block
   gait_ref.c/.h        reference trajectory (GENERATED - see tools/gen_gait.py)

   test_leg_can.c       NEXUS_MODE_LEG_CAN
   test_imu.c           NEXUS_MODE_IMU

pi/                              the Raspberry Pi side, drop into zeus_26
   nexus_proto.py         wire format - must match link_proto.h byte for byte
   nexus_link.py          threaded 1 kHz reader
   zeus_viz.py            live Rerun view - plots, 3D, sim comparison

tools/
   check_proto.py         proves the C and Python layouts agree
   test_pi_link.py        proves the Pi side sustains 1 kHz, no hardware needed
   gen_gait.py            spreadsheet -> gait_ref.c
   plot_run.py            PuTTY log -> tracking plot + gain-tuning metrics

runs/                            archived captures, one CSV per gain setting
                                 (generated - `plot_run.py -l NAME`)

Appli/Core/, Appli/USB_DEVICE/   CubeMX-generated (edit only in USER CODE blocks)
Boot/                            bootloader project, also CubeMX-generated
Drivers/, Middlewares/           ST HAL + USB + external-memory manager (vendored)
sync_from_cubemx.sh              pull generated files back from the CubeMX folder
```

`Appli/Core/Src/main.c` only switches peripherals on and calls `app_init()` /
`app_run()`. All real work lives in `Appli/App/`.

**Adding a source file:** create it in `Appli/App/`, then add it to
`target_sources` in `Appli/CMakeLists.txt`.

---

## Building

Requires STM32CubeCLT (provides `arm-none-eabi-gcc`, CMake, Ninja and
STM32CubeProgrammer).

```bash
cmake --preset Debug
cmake --build build/Debug
```

Outputs, per context:

```
Boot/build/nexus_first_Boot.elf    +  .hex
Appli/build/nexus_first_Appli.elf  +  .hex
```

If the build complains about `cube-cmake` not being found, the CMake cache is
stale (it was created by CubeIDE). Wipe and reconfigure:

```bash
rm -rf build Appli/build Boot/build
cmake --preset Debug && cmake --build build/Debug
```

---

## Flashing

Two separate commands. **The Appli needs an external loader**; the Boot does not.

```bash
# Appli -> external flash at 0x70000000
STM32_Programmer_CLI -c port=SWD mode=UR \
  -el "C:/ST/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr" \
  -d Appli/build/nexus_first_Appli.hex -v

# Boot -> internal flash at 0x08000000
STM32_Programmer_CLI -c port=SWD mode=UR -d Boot/build/nexus_first_Boot.elf -v -rst
```

Notes:

- **Flash the Appli `.hex`, not the `.elf`.** The ELF carries two LOAD segments
  that hold no data (`0x20000000` heap/stack in DTCM, `0x24070000` non-cacheable
  buffers). STM32CubeProgrammer tries to erase at every segment address, the
  external loader only knows how to drive flash at `0x70000000`, and the whole
  download aborts with `Error: failed to download Sector[0]`. The `.hex` records
  only sections that contain data, so it programs cleanly. The Boot is internal
  flash with no external loader attached, so its ELF is fine.
- Use the loader **without** the `-XSPIM1` suffix; this board wires the flash
  to XSPI port 2.
- `mode=UR` (connect under reset) is important for an XIP project — running
  firmware can otherwise block the connection.
- Only reflash Boot when Boot changes. Day-to-day you reflash the Appli only.

---

## Test modes

The full robot loop touches every peripheral at once, which is the worst way
to bring hardware up — when nothing works you cannot tell which of six
subsystems is at fault. So the firmware has single-purpose modes, selected at
build time in `Appli/App/nexus_mode.h`:

```c
#define NEXUS_MODE  NEXUS_MODE_LEG_CAN
```

| Mode | Exercises | State |
|---|---|---|
| `NEXUS_MODE_ROBOT` | Everything — the real 1 kHz loop | builds, untested on hardware |
| `NEXUS_MODE_LEG_CAN` | **CAN-FD only.** One leg, 4 ODrives, reference gait | builds, no ODrives attached yet |
| `NEXUS_MODE_IMU` | **IMU only.** BNO085 over UART | ✅ **working on hardware** |

Change it, rebuild, reflash the Appli. `main()` dispatches on it; unused code
is simply never entered.

### `NEXUS_MODE_LEG_CAN` — single leg over CAN-FD

Talks to four ODrive S1 axes on **FDCAN1** (PB8 rx / PD1 tx) and touches
nothing else. Per joint: 1 TX (`Set_Input_Pos`) and 2 RX
(`Get_Encoder_Estimates`, `Get_Torques`), plus the ODrive heartbeat.

Configuration lives at the top of `Appli/App/test_leg_can.c`:

| Setting | Default | Notes |
|---|---|---|
| `s_node_id[]` | `{1,2,3,4}` | hip_roll, hip_pitch, knee, ankle. Must match `axis0.config.can.node_id` |
| `LEGTEST_ENABLE_CLOSED_LOOP` | **0** | **Safety.** At 0 the axes are never commanded into closed loop, so motors stay unpowered and cannot move — positions are still transmitted, so TX is fully testable |
| `LEGTEST_MOTION_GAIT` | 1 | 1 = play the reference gait, 0 = single-joint sine |
| `LEGTEST_GAIT_SPEED` | **0.25** | Fraction of real time. Start low |
| `LEGTEST_GAIT_ENTRY_MS` | 2000 | Ramp from the measured pose into the trajectory |
| `LEGTEST_USE_CAN_FD` | **0** | Classic CAN 2.0 @ 1 Mbit. Measured: this S1 sends CLASSIC frames and will not acknowledge FD ones |
| `LEGTEST_LISTEN_ONLY` | 0 | Never transmit. Use to verify reception without risking bus-off |
| `LEGTEST_STOP_BUTTON` | 1 | Blue USER button commands IDLE and latches. **Not an e-stop** — it rides the same CAN bus |
| `LEGTEST_GAIT_CYCLES` | 1 | Play this many cycles then hold. 0 loops forever |
| `LEGTEST_VEL_POKE` | 0 | Command a constant velocity instead of positions, to test whether the drive can produce torque at all |
| `s_zero_offset[]` | 0 | Where the gait's zero sits in the drive's coordinates. The arming log computes it for you |
| `LEGTEST_TRACE_RX` | 1 | Decode every received frame onto the console |
| `LEGTEST_TX_DIV` | 1 | 1 = 1 kHz per joint, 4 = 250 Hz per joint |

### The reference gait

`Appli/App/gait_ref.c` is **generated, not hand-written**:

```bash
python tools/gen_gait.py reference_gait_rleg_40ms_250hz.xlsx
```

200 samples, 0.8 s cycle, from a Pinocchio + CasADi trajectory optimisation at
0.40 m/s. Re-run the generator when the trajectory changes — hand-editing 200
rows of floats is how a sign error gets in and stays in.

Two conversions happen in the generator, and both are worth knowing:

- **Degrees → turns is just `/360`.** The spreadsheet is joint angles on the
  output shaft; each ODrive is already configured with its own gear ratio, so no
  reduction factor belongs in the firmware.
- **Columns are reordered into node order.** The spreadsheet runs hipPitch,
  hipRoll, knee, ankle; the CAN nodes run hip_roll(1), hip_pitch(2), knee(3),
  ankle(4). The generator prints the resulting turn ranges so a mix-up is visible
  before anything moves:

```
  slot        column           degrees            turns
  hip_roll    hipRoll_r_deg    -1.67..  +6.86   -0.0047..+0.0191
  hip_pitch   hipP_r_deg      -20.53..  -6.71   -0.0570..-0.0186
  knee        knee_r_deg      +25.53.. +30.23   +0.0709..+0.0840
  ankle       ankle_r_deg      +6.03.. +21.41   +0.0167..+0.0595
```

`gait_sample(phase, out)` interpolates linearly between the 250 Hz samples and
wraps at 1.0, so the 1 kHz loop gets a smooth ramp rather than a staircase.

#### KNOWN DEFECT — the spreadsheet is not continuous

`reference_gait_rleg_40ms_250hz.xlsx` contains **two step discontinuities**, at
rows 99→100 (phase 0.5) and 199→0 (phase 1.0). Every joint jumps about 2° in a
single 4 ms sample while every other step in the trajectory is under 0.15°:

| column | median step | row 99→100 | row 199→0 |
|---|---|---|---|
| hipP_r_deg | 0.134° | **−2.013°** | **+1.961°** |
| hipRoll_r_deg | 0.059° | **+1.798°** | **+1.808°** |
| knee_r_deg | 0.034° | **+1.658°** | **−1.969°** |
| ankle_r_deg | 0.142° | −0.781° | **−1.510°** |

The two halves are not duplicates of one another (they differ by up to 13.8°),
so this is two separate 0.4 s optimiser solutions concatenated without a
periodicity constraint at either join. It is a defect in the source data, not in
the generator or the firmware.

Interpolation does **not** rescue it. It makes position continuous and leaves
velocity discontinuous: the 2° is still delivered inside one row-time, so during
those 16 ms at 0.25× the drive is commanded to move at 0.35 turn/s when the real
gait never exceeds **0.029 turn/s**. Twelve times the peak velocity of the
actual motion, twice per cycle, on every joint at once.

Consequences worth knowing before you read any tracking plot:

- Tracking error has a floor of roughly 2° at those two phases that no gain
  setting can remove. Do not tune against it.
- Velocity feedforward faithfully amplifies it — it feeds forward a velocity
  that exists only because the data is broken, as a kick at a direction
  reversal.
- Any peak-velocity figure computed by differentiating this table is dominated
  by the two joins rather than by the gait.

The real fix is upstream: re-run the optimisation with `q(T) = q(0)` and a
continuous half-step join. Blending the joins in `tools/gen_gait.py` would make
the bench test a clean test signal, but the firmware would then no longer be
playing what the optimiser produced — acceptable for actuator bring-up, not
something to carry into the robot silently.

**Two safety behaviours that are not optional:**

- The leg **ramps from its measured position** into gait sample 0 over
  `LEGTEST_GAIT_ENTRY_MS`, because the first command after arming would
  otherwise be a jump from wherever the leg is resting to phase 0.
- If **any** joint has not reported an encoder estimate over CAN, the gait does
  **not start** — the leg holds station and says so on the console. Ramping from
  an assumed zero would command a jump exactly as large as the assumption is
  wrong.

Start at `LEGTEST_GAIT_SPEED 0.25`. A wrong sign or node mapping then shows as
a slow drift you can watch and kill, rather than a snap you only hear.

Commands go out at **1 kHz to every joint** (`LEGTEST_TX_DIV 1`). Because the
hardware TX FIFO is fixed at three entries and a tick hands over four frames,
they pass through a software queue that `tx_pump()` drains as space frees.
Set `LEGTEST_TX_DIV` to 4 for one joint per tick (250 Hz each) if you want to
halve bus load during early bring-up.

When the queue is full it discards the **oldest** entry. These are position
setpoints: a stale setpoint is worthless, the newest is exactly what the
actuator should receive, and dropping the newest instead would leave the queue
full of seconds-old commands that would replay when the bus recovered.

### If the bus goes quiet

CAN is acknowledged: **every** transmitter needs at least one other node to pull
the ACK slot low. A node whose frames are not acknowledged retries forever, its
transmit error counter climbs, and at 255 the controller takes itself
**bus-off** — which stops *reception* as well. A perfectly good receive path can
therefore look completely dead a second after boot.

The report prints the state that explains it:

```
can: TEC=248 REC=0 txfifo_free=0  [BUS-OFF]
last error: ACK - we transmitted and NOBODY answered
```

**Read the two counters as a pair.** `REC=0` with a high `TEC` means their
frames reach us and ours do not reach them — the fault is on our side of the
wire, not the bus.

This is how the FD mismatch was found. Reception was flawless — 8,731 frames —
while `TEC` climbed to 248 and killed the link. The frame trace showed every
incoming frame decoding as `CLASSIC`, so the drive was never going to
acknowledge our FD frames. One `#define` fixed it.

`LEGTEST_LISTEN_ONLY 1` never transmits, so it can never reach bus-off. Use it
when you need to prove reception works for longer than the error counter allows.

### Bus load

**Measured on hardware, one node:**

```
--- t=5s  tx=1000 rx=2247 (unknown=0)  txfail=0 qdrop=0  bus~39% ---
```

That is 1000 commands out plus ~2250 telemetry frames back — **39% of a 1 Mbit
bus for a single actuator.** A classic 8-byte frame is ~108 bits, so ~3250
frames/s fills roughly a third of the wire on its own.

The consequence is worth being blunt about: **five nodes at these rates does not
fit.** Not "tight" — arithmetically impossible, several times over.

| Nodes, unchanged rates | Bus |
|---|---|
| 1 (measured) | 39% |
| 2 | 78% — already past usable |
| 5 (full robot, per bus) | ~195% — impossible |

Past roughly 50% the latency of lower-priority frames degrades badly; past 70%
it is effectively unbounded.

The drive was streaming encoder estimates at ~1120 Hz and torques at the same
rate, which is what dominates. Both are configured on the ODrive, not here:

```
axis0.config.can.encoder_msg_rate_ms   = 1     # 1 kHz - the estimator needs this
axis0.config.can.torque_msg_rate_ms    = 10    # 100 Hz is plenty for logging
axis0.config.can.heartbeat_msg_rate_ms = 100
```

### Why a frame costs what it does

A classic 8-byte frame is **121 bits**, and only 64 of them are payload:

```
SOF 1 · ID 11 · RTR/IDE/r0 3 · DLC 4 · DATA 64 · CRC 15
      · delimiters 3 · EOF 7 · IFS 3 · ~10 stuff bits
```

At 1 Mbit that is **121 µs per frame**, and 3210 frames/s fills 39% of the
second. The measurement and the arithmetic agree exactly.

**CAN FD would help, by about 2.5x — not 5x.** Only the data field and CRC use
the faster rate; the ID, acknowledgement and end-of-frame always run at the
nominal rate:

```
CLASSIC   |------------ 121 bits at 1 Mbit ------------|   121 us
FD+BRS    |-17 bits-||--- 92 bits at 5 Mbit ---||-12 b-|    47 us
            1 Mbit           data + CRC          1 Mbit
```

| Config | 1 node | 5 nodes/bus |
|---|---|---|
| Classic, 1 ms telemetry (today) | 39% | ~195% |
| Classic, torque at 100 Hz | 27% | ~134% |
| FD, 1 ms telemetry | 15% | ~76% — tight |
| **FD, torque at 100 Hz** | **11%** | **~52%** |

So the full robot needs **both**: FD framing *and* torque telemetry slowed down.
Neither alone is enough. Encoder estimates have to stay at 1 kHz because the
estimator's prediction step runs on them.

That makes getting this drive into genuine FD mode a real prerequisite for the
second leg, not a nice-to-have. It currently sends CLASSIC frames despite being
configured for a 5 Mbit data rate, which is worth resolving with ODrive before
the bus has ten nodes on it.

The test prints a live `bus~NN%` estimate so you can watch this directly.

On the ODrive side, telemetry must be published cyclically:

```
axis0.config.can.encoder_msg_rate_ms   = 1     # 1 kHz - needed by the estimator
axis0.config.can.torque_msg_rate_ms    = 4     # 250 Hz - see bus load below
axis0.config.can.heartbeat_msg_rate_ms = 100
```

Output, once per second:

```
--- t=12s  rx=4821 (unknown=0)  txfail=0 ---
  node 1 hip_roll  : pos= +0.0031 vel=  +0.002 trq= +0.014 state=1 err=0x00000000  hb=12 enc=120 trq=120  cmd=+0.0000
  node 2 hip_pitch : ---- SILENT ----  (hb=0 enc=0 trq=0)
```

Reading it:

- **`qdrop` climbing fast means nobody is on the bus.** CAN needs another node
  to acknowledge every frame; with no transceiver or no powered ODrive nothing
  ACKs, the three hardware slots never free, and the software queue backs up.
  Watching `tx` go from ~3 to the full rate — and `qdrop` stop climbing — is
  your *first* proof of working wiring, before any telemetry appears.
- `bus~NN%` is the estimated bus load. Keep it under ~50%.
- `rx` climbing and `hb`/`enc`/`trq` rising per node = that node is healthy.
- `unknown` counts frames from node IDs you did not configure — a
  misconfigured ODrive shows up here rather than vanishing.
- `SILENT` = nothing heard from that node for 500 ms.

### Enabling closed loop

`Set_Input_Pos` only sets the **target**. Whether the motor acts on it depends
on the axis *state*, which is separate — an axis in `IDLE` (state 1) accepts
the position and does nothing, motor unpowered. To reach
`CLOSED_LOOP_CONTROL` (state 8):

| Method | How |
|---|---|
| odrivetool — best for first bring-up | `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL` |
| Over CAN | `Set_Axis_State` (0x007), payload uint32 `8` — what `LEGTEST_ENABLE_CLOSED_LOOP` sends |
| ODrive auto-start | `axis0.config.startup_closed_loop_control = True` |

The `state=` field in the test output shows this live, so you can confirm
before anything moves.

**Bench safety before enabling closed loop:** leg in a fixture and off the
ground, physical e-stop within reach, low current and velocity limits set in
odrivetool, and bring up one node at a time before running all four. Also
enable ODrive's own watchdog (`axis0.config.enable_watchdog`) — that is the
layer that protects the hardware if this firmware stops.

### Feedforward

`Set_Input_Pos` carries three fields, not one: position (float32 turns), then
`Vel_FF` and `Torque_FF` as int16 at 0.001 units. Without them the drive has to
*discover* every motion from tracking error, which means it is always behind by
whatever error it took to generate the torque.

`LEGTEST_GAIT_VEL_FF` (default 1) sends the trajectory's own derivative.
`gait_sample_vel()` computes it as a central difference across neighbouring
table rows — wrapping rather than clamping at the seam — then interpolates, so
the result is continuous rather than the staircase a forward difference of the
interpolated position would give.

Two scalings are mandatory and both are easy to get wrong:

- **Multiply by `LEGTEST_GAIT_SPEED`.** The phase clock runs at that fraction of
  nominal and the velocity must agree with the position it accompanies. Unscaled
  feedforward at 0.25× asks for four times the motion the setpoint is making.
- **Zero it once the cycles are done.** Holding the final pose freezes the
  *phase*, not the table. The trajectory still has a velocity at that phase; the
  command no longer does. Sending the table's value there drives the joint off a
  stationary setpoint for as long as the test is left running.

The int16 fields saturate rather than wrap. A wrapped int16 turns "too fast
forward" into "full speed backward", which is a command to slam the joint the
wrong way at full velocity; clipping merely asks for less than the trajectory
wanted.

`LEGTEST_GAIT_TORQUE_FF` is **0 and the array stays zero**, because the
spreadsheet has no torque column — it is `time_s, hipP, hipRoll, knee, ankle`,
positions only. The parameter is plumbed so that adding real data later is a
value change rather than a protocol change. Two honest sources: re-export joint
torques from `gait_library_v1.npz` (the Pinocchio/CasADi solve necessarily
computed them, they just were not carried into the xlsx), or compute gravity
compensation on the fly from link mass and measured angle. Inventing a torque
trajectory from a position table would be a guess wearing the costume of a
measurement.

**Feedforward authority is scaled by `vel_gain`.** `Vel_FF` is added to the
velocity setpoint, which is then multiplied by `vel_gain` to become torque — so
with a small `vel_gain` the feedforward contributes almost nothing and appears
to do nothing. Tune `vel_gain` first, then judge whether feedforward helped.

### Tuning gains against captured runs

With `LEGTEST_CAPTURE 1` the board records `cmd`, `pos`, `vel` and `trq` for
joint 0 into a 2048-row RAM buffer at `CAPTURE_HZ` (100 Hz, so 20.5 s of
capacity), **only while the gait is running** so the entry ramp is excluded and
`t=0` is the first sample of real gait. Nothing is transmitted during the run.
When the gait completes it dumps the buffer over UART as CSV, once per boot.

Point PuTTY at a log file — *All session output*, **Always append** — and:

```bash
python tools/plot_run.py                    # plot the newest run
python tools/plot_run.py -l vg1.0_vi5.0     # ...and archive it under that name
python tools/plot_run.py --compare          # overlay everything archived
```

The log path defaults to `DEFAULT_LOG` at the top of the script. Label runs with
the gains that produced them; `try4` is meaningless by the next morning.

Read these two numbers, in this order:

- **`stuck mid-stroke`** — fraction of samples where the command is sweeping
  fast and the encoder is not moving. This is missing torque authority and it is
  what `vel_gain` cures. Drive it to zero *first*.
- **`overshoot`** — how far past the command the joint travels at the ends of
  the stroke. Once stiction is beaten this dominates, and it points at the
  integrator rather than at `vel_gain`.

Sticking at direction *reversals* is reported separately because some of it is
unavoidable in any geared joint, and lumping it in hides the moment the real
problem is solved. `lag` is suppressed entirely while the joint sticks
mid-stroke — a best-fit time shift against a frozen trace is meaningless, and a
confident number there sends you tuning `pos_gain` when the problem is
`vel_gain`.

**Ignore `tracking %`.** It is a peak-to-peak ratio, so a joint thrashing 300%
past the setpoint scores better than one lagging gently. It is printed only
because it appears in older logs.

The loop, one round per gain change — gains are live over USB, no reflash:

```
1. odrivetool:  set vel_gain and vel_integrator_gain, read them back
2. board:       press RESET        (leg clear, stop button in reach)
3. wait ~21 s:  3 s arm + 2 s ramp + 16 s gait, then it dumps
4. host:        python tools/plot_run.py -l vgX_viY
```

If the capture count in the first line of output has not gone up, no new run was
recorded — PuTTY was not logging, or the reset did not take. Do not archive
anything until it does. `odrv0.save_configuration()` only once you are happy;
until then a power cycle silently reverts the gains and you are tuning against a
moving baseline without knowing it.

A worked example, hip_pitch on node 3, where `pos_gain × vel_gain` is the whole
stiffness of the position loop:

| | `vel_gain` 0.167, `vi` 0.333 | `vel_gain` 0.5, `vi` 2.5 |
|---|---|---|
| stiffness | 1.99 Nm/turn | 5.95 Nm/turn |
| RMS error | 12.16° | **2.14°** |
| worst error | 20.30° | 5.01° |
| stuck mid-stroke | seizing for seconds | 0.8% |

At the first setting the proportional term needed 27° of error to make the
~0.15 Nm that breaks the joint loose, and the whole gait is 13.8° — so the
position loop could never move the joint at all, and every motion came from the
integrator slowly winding up at 0.64 Hz. That is what stick-slip looks like in a
log, and it is a gain being an order of magnitude too small rather than anything
mechanical.

### `NEXUS_MODE_IMU` — BNO085 over UART

Exercises **USART1 only** (PA9 tx / PA10 rx, 3 Mbaud) — no CAN, no encoders, no
USB. Working, on hardware:

```
q[w,x,y,z]=+0.4625 +0.2537 -0.1549 +0.8353 |q|=1.000  a[m/s2]= +6.438 -2.910 +7.086  w[rad/s]= -0.014 +0.031 +0.023
  >> 0 err/s   per-report Hz: rotation=105 accel=158 gyro=421
     handshake: advertisement=SEEN reset-notice=seen
```

Data lines are throttled to 5/s because a 115200 console cannot carry 400 — but
the **rate is measured over every sample**, so throttling never hides a rate
problem.

#### Wiring

Per the CEVA datasheet §1.2.3 and the Adafruit pinout:

| STM32 | Breakout | Note |
|---|---|---|
| PA9 (TX) | **SCL** | "UART data IN to sensor" |
| PA10 (RX) | **SDA** | "UART data OUT from sensor" |
| 3V3 | Vin | |
| GND | GND | |
| — | **PS1 → 3V3, PS0 left alone** | selects UART-SHTP |

No reset pin needed for UART. The straps are sampled **at the sensor's reset**,
so after changing them you must fully power-cycle — `-rst` and the black button
restart the STM32 but do not drop the sensor's 3.3 V.

#### Three bugs this bring-up found

All three were in our code, not the wiring, and all three came from the CEVA
datasheet:

**1. Bytes must be 100 µs apart.** §1.2.3.1: *"Bytes sent from the host to the
BNO08X must be separated by at least 100us. Bytes sent from the BNO to the host
have no extra spacing."* We sent frames back-to-back at 3 Mbaud — 3.3 µs per
byte, **thirty times too fast** — so everything after the first byte of every
frame arrived mangled. The rule being one-directional is exactly why the
sensor's own transmissions parsed perfectly while every frame we sent came back
as an SHTP error. This one cost the most time; nothing about the symptoms
pointed at timing.

**2. Wrong acceleration report.** We asked for `0x04` Linear Acceleration, which
has **gravity removed**. The estimator propagates `v += (R·a + g)·dt` and needs
*specific force* — what an accelerometer physically reads. A stationary robot
reported `(0,0,0)` against gravity's `−9.81`, so the filter would have concluded
**free fall** and ramped velocity downward forever. Now `0x01` Calibrated
Acceleration, which includes gravity. The host tests never caught it because
they fed the filter synthetic specific force.

**3. Per-channel sequence numbers.** SHTP counts them per channel; we kept one
global counter, so the control channel opened mid-sequence.

#### Reading the output

- **`|q|` is a free correctness check.** A unit quaternion always has norm 1. If
  it is not ~1.000 the bytes are being misinterpreted — wrong Q-point,
  misaligned report, wrong report id — even when the numbers look plausible.
- **The accelerometer must show gravity.** Held flat and still it reads ~9.81 on
  one axis. Zeros mean bug 2 is back.
- **The counters separate wiring faults from protocol faults**, which otherwise
  look identical:

| Symptom | Meaning |
|---|---|
| `bytes/s = 0`, `ndtr` frozen | Nothing on the wire — TX/RX swapped, no power, no ground |
| bytes but `frames/s = 0` | Wrong baud, or the IMU is in **UART-RVC** instead of **UART-SHTP** |
| frames but only channel 0 | The sensor is *rejecting* what we send — it answers with an SHTP error list |
| `advertisement=not seen` | The sensor never finished booting. Power-cycle. |

The startup **listens for one second before transmitting anything**. Datasheet
§5.2.1 says the sensor announces itself unprompted after reset, so silence in
that window means the part is not running — a question no test that transmits
first can answer.

#### Report rates

Measured, with all three reports requested at 400 Hz:

| Report | Rate | |
|---|---|---|
| Gyro `0x02` | **421 Hz** | drives the filter's orientation |
| Accelerometer `0x01` | **158 Hz** | drives velocity — the binding constraint |
| Rotation vector `0x05` | 105 Hz | deliberately slowed to 100 Hz |

Datasheet §6.9 warns that *"all sensors cannot operate at their maximum rate
simultaneously"*, and the accelerometer is what the sensor starves. **This is
not yet resolved** — see *Known issues*. `IMU_ENABLE_QUAT 0` drops the sensor's
quaternion entirely to test whether the fusion engine is what costs the
accelerometer its budget; the Pi still gets `quat` from our own estimator, which
is fused with leg kinematics and is the better estimate anyway.

## What the Pi receives

One **422-byte packet every millisecond** (422 KB/s, well under 1% of USB HS).
`Appli/App/link_proto.h` and `pi/nexus_proto.py` describe the same bytes;
`tools/check_proto.py` compares every field offset and both struct sizes, so a
mismatch is caught rather than debugged.

### The policy block

Bytes 12 to 219 are **52 contiguous float32** holding exactly what the RL
observation needs, in the order it expects. The Pi slices it in place and copies
nothing:

```python
obs = np.frombuffer(raw, "<f4", count=52, offset=12)
```

| # | Field | Count | Units | Comes from |
|---|---|---|---|---|
| 0 | `pelvis_z` | 1 | m | estimator — height above stance ground |
| 1 | `quat` | 4 | w,x,y,z | estimator (observation uses x,y = indices 1,2) |
| 2 | `gyro` | 3 | rad/s, **body** | BNO085 |
| 3 | `vel_hdg` | 3 | m/s, **heading** | estimator — lateral, forward, vertical |
| 4 | `joint_pos` | 10 | rad, output side | ODrive |
| 5 | `joint_vel` | 10 | rad/s, output side | ODrive |
| 6 | `spring_angle` | 4 | rad | after-spring encoders — **deflection** |
| 7 | `ref_angle` | 10 | rad | gait library — **reserved, reads 0** |
| 8 | `contact` | 4 | 0.0 / 1.0 | foot switches: L toe, L heel, R toe, R heel |
| 9 | `foot_z` | 2 | m, world | forward kinematics — **[0] right, [1] left** |
| 10 | `phase` | 1 | 0..1 | gait clock — **reserved, reads 0** |

**Everything is a raw SI quantity.** The STM32 applies no policy scaling — no
target-height subtraction, no `/10`, no clipping, no sin/cos, no normalisation.
All of that belongs on the Pi, so the observation transform can change without
reflashing the robot.

Four things worth understanding:

- **`spring_angle` is deflection, not a joint angle.** The encoder sits *after*
  the series spring, so it reads how far the spring has wound up. Torque is
  deflection × spring constant, computed on the Pi where the constant is tunable.
- **`vel_hdg` is in the heading frame, not the world.** Forward means where the
  robot faces. Yaw is the one part of the pose the filter cannot observe, so it
  drifts — but the same drifting yaw defines both the velocity and the frame, so
  the error cancels.
- **`foot_z` is computed every tick, including during swing.** A gait policy
  cares most about the foot that is off the ground.
- **`ref_angle` and `phase` read zero** until the gait library runs on the
  STM32. The Pi can tell because `phase` never advances.

### The rest of the packet

Diagnostics and raw values the policy does not need but a human debugging it
does: `imu_quat` (the sensor's own 9-axis fusion, independent of ours),
`imu_accel` / `imu_gyro`, `act_torque`, `act_error` / `act_state` / `act_flags`,
`fused_pos` and `fused_vel` (world frame, before the heading rotation), the
estimated IMU biases, `contact_ticks`, and `health` — the same bitmask the red
LED blinks.

And **`fused_valid`**, which matters more than the rest:

| Value | Meaning |
|---|---|
| `INVALID` | no contact, IMU stale, or the filter diverged |
| `CONVERGING` | running, uncertainty still large |
| `OK` | velocity uncertainty low for 500 consecutive ticks |

**Check `fusion_usable` before using height or velocity.** The filter starts
with 30° of orientation uncertainty and 1 m/s of velocity uncertainty; for the
first ~2 seconds those numbers are meaningless.

### Rates

There is one stream, not several. Every field ships in the same packet every
millisecond — but not every field is *freshly measured* every time:

| Field | Genuinely new at | Note |
|---|---|---|
| `joint_pos/vel`, `spring_angle`, `contact` | **1 kHz** | new value in every packet |
| `gyro`, `imu_accel` | ~400 Hz gyro, ~160-220 Hz accel | bytes repeat between updates; watch `imu_seq` |
| `quat` (ours), `pelvis_z`, `vel_hdg`, `foot_z` | 1 kHz corrected, IMU-rate propagated | contact update every tick, prediction only on a new IMU sample |
| `imu_quat` (sensor's) | 100 Hz | deliberately slowed, see the IMU section |

So actuator and spring-encoder data really does reach the Pi at 1 kHz. The wire
and the kernel buffer never drop it — **the only way to lose it is to read too
slowly.**

### Receiving it at 1 kHz

`pi/nexus_link.py` is the reader. A background thread drains the port so
reception stays at 1 kHz no matter what the control loop is doing; the loop
then takes the newest packet and ignores the rest.

```python
from nexus_link import NexusLink

with NexusLink('/dev/ttyACM0') as link:
    if link.wait_for_fusion(timeout=10) is None:
        raise RuntimeError('estimator never converged')

    while True:                       # policy at 250 Hz
        pkt = link.latest()           # never blocks, never stale
        if pkt is None or not pkt.fusion_usable:
            continue
        action = policy(build_obs(pkt))
        link.send_command(action_to_turns(action))
```

**A 250 Hz loop that reads one packet per step falls three packets further
behind every step, forever.** Discarding the older three is not data loss — the
newest packet is the only one that is not already out of date. Pass
`history=N` and call `drain()` if you also want every packet for logging.

`link.stats` reports `loss_rate`, `seq_gaps` and `junk_bytes`; all three should
be zero on a healthy link. Run `python pi/nexus_link.py /dev/ttyACM0` on the Pi
before wiring anything into zeus_26 — a problem is far easier to find there than
inside a control loop.

`tools/test_pi_link.py` proves the Python side sustains 1 kHz without hardware:
it feeds `NexusLink` a synthetic 1 kHz stream through a fake port and checks
every packet arrives, in order, with no gaps. It also measures parse load, which
matters more than it looks — the original pure-Python CRC cost 55% of a desktop
core at 1 kHz and would not have kept up on a Pi at all. `crc16` now calls
`binascii.crc_hqx`, the same algorithm in C and ~300× faster (2.8% of a core);
`_crc16_slow` is kept as the reference the test checks it against.

### Layout rules

- **Every 4-byte field sits on a 4-byte boundary.** The struct is packed, so
  otherwise the M7 emits byte-wise access for every misaligned float and numpy
  cannot view the buffer in place on the Pi. Fields are grouped by size rather
  than by topic for this reason.
- **The policy block is contiguous and its offset is checked.**
  `tools/check_proto.py` fails if the block moves or gains a gap, because the
  zero-copy slice above would then read the wrong bytes silently.
- **Protocol version is 3.** v1 sent raw encoder counts; v2 added the health
  byte and alignment; v3 added the policy block and split the contacts. Mismatched
  versions reject each other rather than silently misparsing.

## Watching it live — Rerun

`pi/zeus_viz.py` turns the 1 kHz packet stream into live plots and a 3D view.
It is a consumer of the existing link, not new plumbing.

### The one thing to understand

Rerun splits into an **SDK** and a **Viewer**, and they do not have to be on the
same machine. That split is the whole answer to "the Pi has no screen":

```
STM32 ──USB──► Raspberry Pi ──network──► your laptop
1 kHz          zeus_viz.py               Rerun Viewer
sensing        (the SDK)                 (the window)
               headless                  where you look
```

The Pi logs. The laptop looks. Neither needs the other to exist for the robot
to run — **the viewer is never in the control path.**

### Wireless

```bash
# laptop — starts an empty viewer and waits
pip install rerun-sdk
rerun

# pi — point it at the laptop's IP
python pi/zeus_viz.py /dev/ttyACM0 --connect 192.168.1.50
```

Port **9876**, plain TCP. Wi-Fi is fine: decimated to 200 Hz the stream is a few
hundred KB/s, which is nothing for 802.11n. Expect 5–50 ms of latency and some
jitter — irrelevant for watching, and the reason this is *only* for watching.

> **Never put anything that commands the robot on Wi-Fi.** The policy runs on
> the Pi over USB for a reason. A dropped viewer connection must be a blank
> screen, never a stalled leg.

### Wired

Same command — the Pi just reaches the laptop over Ethernet or a USB-Ethernet
gadget instead. Lower latency and no dropouts, worth it if you are watching
something fast. Nothing in the script changes.

### Record now, look later

```bash
# pi
python pi/zeus_viz.py /dev/ttyACM0 --save run.rrd

# laptop
scp pi@raspberrypi:run.rrd .  &&  rerun run.rrd
```

**Start here.** No network at all, and recorded sessions scrub — you can find
the 40 ms where something went wrong and step through it, which live viewing
cannot do. Most interesting failures are over before you look up.

### Laptop only, no Pi

```bash
python pi/zeus_viz.py COM7 --spawn
```

STM32 plugged straight into the laptop. **This works today with one actuator** —
you do not need the Pi to exist to develop against real data.

### What it shows

| Path | |
|---|---|
| `estimator/height`, `estimator/vel/*` | what the policy consumes |
| `joints/pos/*` vs `joints/ref/*` | commanded against measured, same plot |
| `springs/*`, `contact/*`, `feet/*_z` | leg state |
| `gait/phase` | where in the cycle |
| `imu/gyro/*`, `imu/accel/*` | raw, for cross-checking the estimator |
| `world/pelvis` | 3D pose from the fused quaternion |

Scalars are decimated to 200 Hz (`DECIMATE` in the file). Set it to 1 when
chasing impact transients; the eye cannot use 1 kHz but a plot can.

### Simulation beside reality

`log_sim()` takes joint angles from MuJoCo and logs them under `sim/…` on the
same timeline. Drop `sim/joints/3` and `joints/pos/3` into one plot and the
sim-to-real gap is right there.

**MuJoCo is not part of the robot** — it is the simulator the policy was trained
in, running on a desktop. This comparison is optional. It is also the single
most useful thing here, because a policy that works in simulation and not on
hardware shows its divergence on that plot long before the robot shows it by
falling over.

---

## State estimation

A contact-aided right-invariant EKF, ported from the Python in
`vismay5559/zeus_26` (`zeus_sensor_fusion`) and following Hartley et al. 2019.
It fuses the IMU with leg forward kinematics while feet are on the ground to
estimate body orientation, velocity, height and IMU biases.

All fusion runs **on the STM32**. The Pi receives the fused state and runs only
the RL policy.

| File | Contents |
|---|---|
| `lie_group.c` | SO(3) exponential, Gamma0-3, fixed-stride matrix helpers |
| `kinematics.c` | Leg FK (0.30 m thigh/shank, 0.05 m foot) + 3x4 Jacobian |
| `inekf.c` | Predict, contact update, contact add/remove |

State is `X` in SE_{N+2}(3) with `R, v, p` and one world position per contact,
IMU bias `theta` in R^6, and a 21x21 right-invariant error covariance.

### Deliberate differences from the Python

**Fixed-size, no allocator.** The Python reshapes `X` and `P` with numpy every
time a foot lands or lifts. Everything here is declared at its two-contact
maximum with an active flag per slot; error-state matrices keep a constant
stride so indices never move, and inactive contact blocks are zeroed so they
are inert in every product.

**`float`, not `double`.** Singles are ~2x faster on this FPU and halve memory
traffic, and covariance propagation is the hot path. Joseph-form updates plus
explicit symmetrisation every step keep it stable. `inekf_real_t` is the one
switch if that ever stops holding.

**Central differences for the Jacobian.** The Python uses forward differences
with `eps=1e-6`, which is fine at float64 but broken at float32: FK output is
order 0.5 m, so a 1e-6 rad step moves it ~1e-7 m - right at float resolution.
Central differences with `eps=1e-3` are O(h^2), so a step clear of the noise
floor is still more accurate.

### Checked against the paper

Verified equation by equation against Hartley et al. 2019
(arXiv:1904.09251). Two defects were found and fixed that the Python has:

**`Phi(phi,phi)` must be IDENTITY** (eq 58), not `Gamma0^T`. `Gamma0^T` belongs
to the *left*-invariant `Phi^l` (eq 55), where the error lives in the body
frame. This is not cosmetic - the identity block is exactly why the
right-invariant error dynamics are independent of the state estimate, which is
the property the whole method rests on. With `Gamma0^T` there, the filter
degrades into the state-dependent linearisation the paper compares against and
beats.

**`Q_bar` must use the full adjoint** (eq 28: `Ad*Cov(w)*Ad^T`). `Ad` (eq 63)
carries `(v)_x R`, `(p)_x R` and `(d_k)_x R` in its first block column, so gyro
noise reaches velocity, position and every contact with cross-covariances
between them. A diagonal `Q_bar` drops all of it. Those terms scale with `|v|`,
`|p|`, `|d|` - negligible at the origin, growing as the robot walks away, which
is when the covariance most needs to be right.

`H` is written as `[0 0 -I I 0 0]` exactly as eq 20, with the correction `+K z`
per eq 29, so the code reads directly against the paper.

Known approximations, both as the paper allows:

- `Psi1`/`Psi2` (eq 56/57) use only the leading `(a)_x Gamma2/3(-w dt)` term
  and drop the higher-order corrections. Those vanish as `w*dt -> 0`, and at
  400 Hz `w*dt` is small.
- `Q_d ~ Phi Q_bar Phi^T dt` (eq 61) - the same approximation the paper used
  for all of its own results.

### Two bugs found in the Python during the port

**The measurement is not used in the update.** `update_contact` computes
`Xinv @ b` and never touches `B_p_BC`, so the innovation is
`R^T(d_k - p)` - the prediction alone, with the encoder measurement missing
entirely. The C version uses the right-invariant innovation
`z = R*B_p_BC + p - d_k`, which is zero exactly when the state agrees with
forward kinematics.

**`Phi` is built from the already-propagated state.** `R`, `v`, `p` are numpy
*views* into `self._X`; writing the propagated values back silently updates
them, so `state_transition_right` receives the new state and then propagates it
again internally. `Phi` linearises about the state at t_k, so the C version
keeps the pre-propagation values.

Both are worth fixing in the Python too if it stays in use.

### Verification

Checked on the host against independent references, not against the Python:

- `Gamma1` against a numerically integrated `exp(phi*s) ds` (1e-4)
- FK against hand-computable poses; Jacobian against analytic cross products
- Free fall for 1 s gives exactly -9.81 m/s and -4.905 m
- **An injected 20 cm position error decays to 1.1 mm under contact updates** -
  this is what validates the innovation and `H` sign convention; with either
  flipped the error would diverge instead
- Covariance stays symmetric and positive on the diagonal through 200 cycles
- **The observability structure of section 5.4 is reproduced**: over 4000 steps
  roll and pitch covariance shrink ~50x while yaw, being unobservable, does not
  converge at all. This only emerges with the identity `Phi` block, so the test
  fails against the pre-correction code
- The `phi`-position cross-covariance is non-zero once the body has moved off
  the origin, which a diagonal `Q_bar` could never produce

**The filter is not yet wired into the control loop.** `inekf.c` builds and is
tested, but nothing calls it, so the `fused_*` fields of `nexus_state_t` are
still transmitted as zeros. Wiring it up needs joint angles from the encoders
and ODrives, neither of which has produced real data yet.

## Diagnostics

### Serial console

`printf` is routed to the **ST-LINK virtual COM port** in both projects.
Open it at **115200 8N1, no flow control**. Boot prints its external-memory
init trace; the Appli prints a status line every 2 seconds:

```
loop max 234 us | overruns 0 | can drop 0/0 | health 0x01 watching 0x21 blink 1
```

| Field | Meaning |
|---|---|
| `loop max` | Longest single cycle, of a 1000 µs budget |
| `overruns` | Ticks missed because a cycle ran long. Should stay 0 |
| `can drop` | Frames dropped per bus because the bus could not keep up |
| `health` | Bitmask of faulted subsystems — see below |
| `watching` | Which subsystems are armed (`HEALTH_EXPECTED_NOW`) |
| `blink` | What the red LED is currently blinking |

### LEDs

| LED | Meaning |
|---|---|
| **LD1 green** | 1 Hz heartbeat — the 1 kHz loop is alive and on time |
| **LD2 yellow** | Boot→Appli marker. Off = handover fine. **Stuck on = the Appli never started** |
| **LD3 red** | Blink code for the first faulted subsystem |

```
1 blink = IMU stale             4 blinks = CAN bus 2 silent
2 blinks = encoders invalid     5 blinks = Pi link stale
3 blinks = CAN bus 1 silent     6 blinks = loop overrun
```

Red also goes solid from `Error_Handler()` or a hard fault.

**"Healthy" means data is flowing, not that the peripheral initialised.** Every
peripheral initialises fine with nothing plugged in, which tells you nothing —
so an unconnected sensor is deliberately a fault. `HEALTH_EXPECTED_NOW` in
`Appli/App/health.h` gates which subsystems may complain; add each flag as you
wire that hardware, and red going dark is your proof it works.

---

## Current state

| Subsystem | State |
|---|---|
| Boot chain, XIP execution | ✅ working |
| 1 kHz loop | ✅ ~230 µs of a 1000 µs budget, zero overruns |
| Health LEDs, serial console | ✅ working |
| **IMU (BNO085)** | ✅ **working on hardware** — quaternions, gravity, gyro |
| Pi link, protocol v3 | ✅ C/Python verified, 1 kHz proven in test |
| State estimator | ✅ wired in, ⚠️ never run on real sensor data |
| Encoders (SPI) | ⚠️ initialises, no hardware attached |
| CAN / ODrive | ⚠️ initialises, no hardware attached |

The IMU is the first external interface actually proven end to end. Encoders and
CAN still have never moved a byte of real data.

### Known issues

Only genuinely open items live here. Bugs that were found and fixed but that
CubeMX can silently undo are listed under
[Working with CubeMX](#working-with-cubemx) instead — they are hazards, not
outstanding work.

**Accelerometer runs at ~158 Hz, not 400.** The gyro reaches 421 Hz but the
accelerometer — which drives the estimator's velocity and height — is starved by
the sensor's own scheduling (datasheet §6.9: sensors cannot all run at maximum
simultaneously). Dropping the quaternion from 400 Hz to 100 Hz made it *worse*
(216 → 158) and cut total throughput from 972 to 684 reports/s, which rules out
simple bandwidth saturation. Untested next steps, in order: `IMU_ENABLE_QUAT 0`,
requesting 800 Hz to exploit the `≤ 2.1 × requested` rule, and the Game Rotation
Vector (`0x08`, 6-axis, much cheaper — and better on a robot full of motor
magnets). 158 Hz is usable but below where it should be.

**Estimator calibration constants are placeholders.** The `sign` and `offset`
per joint in `fusion.c` are `+1` and `0`. Until they are measured on the real
leg, forward kinematics is offset by however wrong they are — **and the filter
will trust it completely.** This is the main thing hardware unblocks.

**`ref_angle` and `phase` read zero.** The gait library does not run on the
STM32 yet; only the leg test plays the trajectory. Space is reserved in the
packet so the Pi side can be written against the final layout now.

**External flash runs in 1-line mode, not octal.** SFDP init fails at step 11
(re-reading the SFDP header through the freshly configured octal mode) with
`EXTMEM_DRIVER_NOR_SFDP_MEMTYPE_CHECK`. 1-line works and is the current setting
(`NEXUS_EXTMEM_LINES` in `Boot/Core/Src/extmem_manager.c`), at roughly 1/8th the
instruction-fetch bandwidth. The loop runs in ~230 µs of its 1000 µs budget
*with* the estimator, so this costs nothing measurable today. If that margin
shrinks, move the hot path into ITCM (64 KB, 0% used) before spending time on
octal mode.

**CAN bandwidth needs attention before all 10 actuators run.** At 5 nodes per
bus with commands plus two telemetry messages each at 1 kHz, the bus sits near
75% load. Raising the *nominal* bitrate helps far more than the data bitrate,
because 8-byte frames are dominated by the arbitration phase. Also verify
whether your ODrive firmware supports CAN FD at all. Deferred deliberately —
4 nodes at 45% is fine, so this only blocks the second leg.

**No watchdog on the STM32.** A deliberate choice: ODrive's own watchdog is the
hardware-protecting layer, so a stalled STM32 stops the motors without needing
one here. Revisit only if something has to fail safe that the ODrives cannot
see.

### Resolved

For reference, since these were live problems during bring-up: the MPU/DMA
cache-coherency fault, the `MX_XSPI2_Init()` bus hang, the USB voltage detector
killing Boot, the 3-deep FDCAN TX FIFO silently dropping frames to nodes 4 and
5, the `imu_service()` infinite loop, the `Q̄` stack overflow in the estimator,
the USB TX buffer being overwritten mid-transfer, the pure-Python CRC that
could not sustain 1 kHz on a Pi, and the three BNO085 protocol bugs above. Each
has a guard, a counter, or a test so it cannot come back unnoticed.

---

## Working with CubeMX

CubeMX generates into its own project folder, which is not this repo. Generate
there, then pull the generated files back:

```bash
./sync_from_cubemx.sh
git diff --stat -- . ':!Drivers' ':!Middlewares'   # review before building
```

The script never touches `Appli/App/`, and warns if MPU region 2 came back
wrong. **Always review the diff before building** — a stale CubeMX folder can
silently revert fixes.

### Two fixes CubeMX will undo

Both of these are *fixed*. Both are also things CubeMX regenerates back to a
broken state, so check them after every generation.

**MPU region 2 must cover `0x24070000`, size 8 KB.** Every DMA buffer lives in
the `noncacheable_buffer` section there. If the MPU and the linker disagree,
the CPU reads stale cache while DMA writes real RAM — intermittent corruption
that looks exactly like bad wiring. The setting lives in the **Boot** context
of the .ioc (CORTEX_M7_BOOT → MPU Region 2).

*Guard:* `check_noncacheable_region()` in `app_init()` compares the linker
symbols against the MPU base at boot and halts with the red LED on if they
disagree, so this fails loudly instead of corrupting data.

**`MX_XSPI2_Init()` must stay commented out in the Appli**
(`Appli/Core/Src/main.c`). The Appli executes *from* XSPI2, so re-initialising
it destroys the memory mapping and the next instruction fetch hangs the bus —
no LED, no serial output, nothing.

*Guard:* none possible; the hang happens before any code could report it. This
one is on you to check in the diff.
