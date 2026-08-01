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

           one fixed struct over USB — see Appli/App/link_proto.h
```

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
  -d Appli/build/nexus_first_Appli.elf -v

# Boot -> internal flash at 0x08000000
STM32_Programmer_CLI -c port=SWD mode=UR -d Boot/build/nexus_first_Boot.elf -v -rst
```

Notes:

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

| Mode | Exercises |
|---|---|
| `NEXUS_MODE_ROBOT` | Everything — the real 1 kHz loop |
| `NEXUS_MODE_LEG_CAN` | **CAN-FD only.** One leg, 4 ODrives |
| `NEXUS_MODE_IMU` | **IMU only.** BNO085 over UART |

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
| `LEGTEST_AMPLITUDE_TURNS` | 0.05 | Only matters with closed loop on |
| `LEGTEST_FREQ_HZ` | 0.25 | Slow enough to watch |
| `LEGTEST_USE_CAN_FD` | 1 | Set to 0 for classic CAN 2.0 @ 1 Mbit if your ODrive firmware does not do FD |
| `LEGTEST_TX_DIV` | 1 | 1 = 1 kHz per joint, 4 = 250 Hz per joint |

Commands go out at **1 kHz to every joint** (`LEGTEST_TX_DIV 1`). Because the
hardware TX FIFO is fixed at three entries and a tick hands over four frames,
they pass through a software queue that `tx_pump()` drains as space frees.
Set `LEGTEST_TX_DIV` to 4 for one joint per tick (250 Hz each) if you want to
halve bus load during early bring-up.

When the queue is full it discards the **oldest** entry. These are position
setpoints: a stale setpoint is worthless, the newest is exactly what the
actuator should receive, and dropping the newest instead would leave the queue
full of seconds-old commands that would replay when the bus recovered.

### Bus load

With `set_input_pos`, `encoder` and `torque` all at 1 kHz, each node needs
3 frames/ms. Small 8-byte frames are dominated by the **arbitration phase**,
which runs at the *nominal* bitrate — so raising the nominal rate helps far
more than raising the data rate.

| | frames/ms | 1M nominal | 2M nominal |
|---|---|---|---|
| 4 nodes (this leg test) | 12 | 60% — tight | 42% — ok |
| 5 nodes (full robot, per bus) | 15 | **76% — too high** | 52% — tight |

Past roughly 50% the latency of lower-priority frames degrades badly; past 70%
it is effectively unbounded. **Four nodes at full rate works today. Five will
not**, so before the second leg goes on, either raise the nominal bitrate
(both here and in ODrive) or drop torque telemetry to ~250 Hz.

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

### `NEXUS_MODE_IMU` - BNO085 over UART

Exercises **USART1 only** (PA9 tx / PA10 rx, 3 Mbaud) - no CAN, no encoders,
no USB. Asks the BNO085 for rotation vector, linear acceleration and gyro at
400 Hz over SHTP, then prints them.

```
q[w,x,y,z]=+0.9998 +0.0121 -0.0043 +0.0155 |q|=1.000  a[m/s2]= +0.021 -0.014 +0.008  w[rad/s]= +0.001 -0.002 +0.000
  >> 400 samples/s (want 400), 400 frames/s, 17600 bytes/s   OK
```

Data lines are throttled to 5/s because a 115200 console cannot carry 400 -
but the **rate is measured over every sample**, so throttling never hides a
rate problem.

Two things worth understanding in that output:

- **`|q|` is a free correctness check.** A unit quaternion always has norm 1.
  If it is not ~1.000 the bytes are being misinterpreted - wrong Q-point,
  misaligned report, wrong report id - even when the numbers look plausible.
- **`bytes/s` and `frames/s` separate wiring faults from protocol faults**,
  which otherwise look identical:

| Symptom | Meaning |
|---|---|
| `bytes/s = 0` | Nothing on the wire — TX/RX swapped, no power, no common ground |
| bytes but `frames/s = 0` | Wrong baud, or the IMU is in **UART-RVC** mode instead of **UART-SHTP** |
| frames but `samples/s = 0` | Framing fine, but `SET_FEATURE` did not take |
| `samples/s` well under 400 | Running, but not at the requested rate |

Wiring: STM32 **PA9 → IMU RX**, STM32 **PA10 → IMU TX**, common ground, and the
BNO085 must be strapped for **UART-SHTP** mode (PS0/PS1 select the interface —
not I2C, not SPI, and not UART-RVC, which is a different fixed-100 Hz protocol
this driver does not speak).

## What the Pi receives

One 326-byte packet every millisecond (326 KB/s, well under 1% of USB HS).
`Appli/App/link_proto.h` and `pi/nexus_proto.py` describe the same bytes;
`tools/check_proto.py` compares every field offset and both struct sizes, so a
mismatch is caught rather than debugged.

| What | Field | Units |
|---|---|---|
| Actuator position / velocity | `act_pos[10]`, `act_vel[10]` | turns, turns/s |
| Actuator torque | `act_torque[10]` | Nm |
| After-spring joint angles | `enc_angle[4]` | **radians** |
| IMU orientation / accel / rate | `imu_quat[4]`, `imu_accel[3]`, `imu_gyro[3]` | quaternion, m/s^2, rad/s |
| **Torso height** | `fused_pos[2]` | m |
| **Torso velocity** | `fused_vel[3]` | m/s, world |

Also carried, because the Pi needs them to know whether to trust the rest:
`fused_quat`, the estimated IMU biases, `fused_valid` (the estimator refuses to
claim `OK` until it has converged), `contacts` / `contact_ticks`, per-actuator
`act_error` / `act_state` / `act_flags`, and `health` - the same bitmask the red
LED blinks.

Using it from zeus_26:

```python
from nexus_proto import NexusState, FUSION_OK

pkt, buf = NexusState.find_and_parse(buf)
if pkt and pkt.fusion_usable:
    height = pkt.height          # fused_pos[2]
    vel    = pkt.velocity        # fused_vel
    if pkt.faults():
        print("STM32 reports:", pkt.faults())
```

**Check `fusion_usable` before using height or velocity.** The filter starts
with 30 degrees of orientation uncertainty and 1 m/s of velocity uncertainty;
until it converges those numbers are meaningless.

Three deliberate choices:

- **Encoder angles are sent in radians, not raw counts.** The STM32 needs them
  in radians for forward kinematics anyway, so converting on the Pi would
  duplicate the scale and any zero offsets in two places that can drift apart.
- **Every 4-byte field sits on a 4-byte boundary.** The struct is packed, so
  otherwise the M7 emits byte-wise access for every misaligned float and numpy
  cannot view the buffer in place on the Pi. Fields are grouped by size rather
  than by topic for this reason.
- **Protocol version is 2.** v1 sent raw encoder counts, had no health byte and
  was misaligned. A v1 Pi and a v2 STM32 reject each other on the version check
  rather than silently misparsing.

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

**Working:** boot chain, XIP execution, 1 kHz loop at ~230 µs of a 1000 µs
budget (77% headroom), zero overruns, health/LED diagnostics, serial console.

**Not yet verified:** every external interface. The IMU, encoders, CAN buses
and USB link all *initialise* cleanly, but no hardware is attached yet, so none
of them has moved a single byte of real data.

### Known issues

**External flash runs in 1-line mode, not octal.** SFDP init fails at step 11
(re-reading the SFDP header through the freshly configured octal mode) with
`EXTMEM_DRIVER_NOR_SFDP_MEMTYPE_CHECK`. 1-line works and is the current
setting (`NEXUS_EXTMEM_LINES` in `Boot/Core/Src/extmem_manager.c`), at roughly
1/8th the instruction-fetch bandwidth. Fine today; worth fixing before the
state estimator lands, or move the hot path into ITCM (64 KB, unused).

**`MX_XSPI2_Init()` is disabled in the Appli** (`Appli/Core/Src/main.c`). The
Appli executes *from* XSPI2, so re-initialising it destroys the memory mapping
and the next instruction fetch hangs the bus. CubeMX regenerates this call —
comment it out again after any regeneration.

**MPU region 2 must cover `0x24070000`, size 8 KB.** Every DMA buffer lives in
the `noncacheable_buffer` section there. If the MPU and linker disagree, the
CPU reads stale cache while DMA writes real RAM — intermittent corruption that
looks exactly like bad wiring. `app_init()` checks this at boot and halts with
red on if it is wrong. The setting lives in the **Boot** context of the .ioc
(CORTEX_M7_BOOT → MPU Region 2).

**CAN bandwidth needs attention before all 10 actuators run.** At 5 nodes per
bus with commands plus two telemetry messages each at 1 kHz, the bus sits near
75% load. Raising the *nominal* bitrate helps far more than the data bitrate,
because 8-byte frames are dominated by the arbitration phase. Also verify
whether your ODrive firmware supports CAN FD at all.

**No watchdog on the STM32 yet.** Enable ODrive's own watchdog as the
hardware-protecting layer.

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
