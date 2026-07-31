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
| `NEXUS_MODE_IMU` | *(planned)* IMU only |

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
axis0.config.can.encoder_msg_rate_ms   = 1
axis0.config.can.torque_msg_rate_ms    = 1
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
