# What the STM32 sends the ODrives while the leg runs

Exact wire contents of `NEXUS_MODE_LEG_CAN`. Everything here is what the
firmware in this repo actually does — see `Appli/App/test_leg_can.c`.

**Control mode is POSITION, not torque.** The STM32 streams position setpoints
with velocity feedforward and the ODrive closes the position, velocity and
current loops itself. If you are looking for a torque-control loop with
`TAU_FF_SCALE`, that is a different program and it does not exist in this repo
yet — see the note at the end.

---

## Addressing

CANSimple: `arbitration_id = (node_id << 5) | cmd_id`, 11-bit standard id,
CAN-FD with BRS, 1 Mbit arbitration / **2 Mbit** data. See the README on why
not 5.

| frame | cmd | node 1 hip | node 3 knee | node 4 ankle |
|---|---|---|---|---|
| Heartbeat (rx) | `0x001` | `0x021` | `0x061` | `0x081` |
| Set_Axis_State (tx) | `0x007` | `0x027` | `0x067` | `0x087` |
| Get_Encoder_Estimates (rx) | `0x009` | `0x029` | `0x069` | `0x089` |
| Set_Controller_Mode (tx) | `0x00B` | `0x02B` | `0x06B` | `0x08B` |
| Set_Input_Pos (tx) | `0x00C` | `0x02C` | `0x06C` | `0x08C` |
| Get_Torques (rx) | `0x01C` | `0x03C` | `0x07C` | `0x09C` |

All multi-byte fields are **little-endian**.

---

## Sequence for one run

### 1. Scan — 2000 ms, transmitting nothing

Listen only. Every node must be seen, `axis_error == 0`, `axis_state == 1`.
The firmware refuses to arm if a configured node stayed silent: commanding a
node that never answered is how you discover a wiring fault by driving a joint
you cannot see.

### 2. Arm — once, per node

**`Set_Controller_Mode`** — `0x00B`, 8 bytes:

| bytes | field | value |
|---|---|---|
| 0..3 | `control_mode` | `3` = POSITION_CONTROL |
| 4..7 | `input_mode` | `1` = PASSTHROUGH |

```
06 B    03 00 00 00  01 00 00 00
```

PASSTHROUGH is deliberate. POS_FILTER would add its own lag on top of a
trajectory that is already time-parameterised, and TRAP_TRAJ would re-plan a
trapezoid every millisecond and never finish one.

**`Set_Axis_State`** — `0x007`, 4 bytes: `uint32 = 8` (CLOSED_LOOP_CONTROL).

```
06 7    08 00 00 00
```

Confirm `state=8` in the heartbeat before commanding motion.

### 3. Entry ramp — `LEGTEST_GAIT_ENTRY_MS`, default 2000 ms

Straight line from the joint's **measured** position to gait sample 0, at 1 kHz.
Not from an assumed zero — the first command after arming would otherwise be a
step from wherever the leg is resting to phase 0, which on an assembled leg is a
kick of unknown size.

Velocity feedforward during the ramp is the line's own constant slope,
`(target − start) / ramp_seconds`.

### 4. Gait — 1 kHz per node, `Set_Input_Pos`

`0x00C`, 8 bytes:

| bytes | field | type | units |
|---|---|---|---|
| 0..3 | position | `float32` LE | **turns** |
| 4..5 | `Vel_FF` | `int16` LE | **0.001 turns/s** |
| 6..7 | `Torque_FF` | `int16` LE | **0.001 Nm** |

The two int16 fields **saturate**, they do not wrap. A wrapped int16 turns "too
fast forward" into "full speed backward" — a command to slam the joint the
wrong way at full velocity. Clipping only ever asks for less than the trajectory
wanted.

`Torque_FF` is **always 0**. The bytes are documented because they are on the
wire; nothing in this system writes them.

### 5. End of run

`Set_Axis_State` = `1` (IDLE) to every node, then transmission stops
(`LEGTEST_GAIT_IDLE_AFTER 1`). The queue is flushed *before* the idle frames are
enqueued and drained *before* returning, so the idle is on the wire rather than
sitting behind a console dump.

> **IDLE de-energises the motor and the joint goes limp.** On an assembled leg
> it will fall the instant the last cycle completes. Set
> `LEGTEST_GAIT_IDLE_AFTER 0` before running the assembled leg, or make sure the
> fixture catches it.

The blue button does the same thing at any moment, and latches until reset.

---

## Units — the part that bites

The position field is **turns of whatever the ODrive's encoder measures**. That
is not the same shaft on every joint of this leg.

| joint | node | gearbox | encoder on | command = | 1° of output = |
|---|---|---|---|---|---|
| hip pitch | 1 | 47:1 | **load** | `deg / 360` | 0.00278 turns |
| knee pitch | 3 | 47:1 | **load** | `deg / 360` | 0.00278 turns |
| ankle pitch | 4 | 9:1 | **MOTOR** | `deg / 360 × 9` | **0.02500 turns** |

**The ankle is 9× everything else.** Its encoder is on the motor, before the
9:1, so a command of 0.0278 turns moves the output by 1°, not 9°.

This applies to **`Vel_FF` as well as position.** Feedforward velocity is in
turns/s of the same shaft, so the ankle's feedforward is also ×9. Scaling the
position and forgetting the feedforward gives a drive fighting itself.

### Per-joint command ranges for this gait

| joint | output range | commanded turns | mechanical limit | margin |
|---|---|---|---|---|
| hip pitch | −20.53° .. −6.71° | −0.0570 .. −0.0186 | ±25° | **4.47°** |
| knee pitch | +25.53° .. +30.23° | +0.0709 .. +0.0840 | ±35° | **4.77°** |
| ankle pitch | +6.03° .. +21.41° | **+0.1507 .. +0.5353** | ±35° | 13.59° |

The ankle's commanded turns look alarming next to the others and are correct —
that is 54° to 193° of *motor* shaft producing 6° to 21° of *output*.

Margins assume the encoder zero coincides with the joint's mechanical zero. It
does not, until someone measures it. See `ZEROING.md`, then Test 5 in
`CAN_BUS_TEST.md`.

---

## What we receive

| frame | bytes | contents |
|---|---|---|
| `Get_Encoder_Estimates` `0x009` | 0..3 / 4..7 | `float32` position (turns), `float32` velocity (turns/s) |
| `Get_Torques` `0x01C` | 0..3 / 4..7 | `float32` target torque, `float32` estimated torque (Nm) |
| `Heartbeat` `0x001` | 0..3 / 4 / 5 / 6 | `uint32` axis_error, `uint8` axis_state, `uint8` procedure_result, `uint8` trajectory_done |

Required rates, per axis:

```python
odrv0.axis0.config.can.encoder_msg_rate_ms   = 1     # 1000 Hz
odrv0.axis0.config.can.torque_msg_rate_ms    = 10    #  100 Hz
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 10    #  100 Hz
```

Encoder at 1 kHz matches the command rate, so every setpoint can be compared
against a measurement taken in the same millisecond. Torque and heartbeat at
100 Hz match `CAPTURE_HZ`, which is the fastest the capture records anyway.

---

## Bus budget

| direction | frames/s |
|---|---|
| STM32 → drives, `Set_Input_Pos` 1 kHz × 3 | 3 000 |
| drives → STM32, encoder 1 kHz × 3 | 3 000 |
| drives → STM32, torque 100 Hz × 3 | 300 |
| drives → STM32, heartbeat 100 Hz × 3 | 300 |
| **total** | **6 600** |

Roughly 33% at the firmware's 50 µs/frame estimate. The TX path is a software
ring drained by `tx_pump()` because the hardware FIFO holds three entries and a
tick hands over more than that. When the ring is full it discards the **oldest**
entry: these are position setpoints, a stale one is worthless, and dropping the
newest would leave the queue replaying seconds-old commands after a recovery.

---

## Arbitrary parameter access (SDO)

Predefined CANSimple messages cover setpoints and gains. Anything else — any
parameter reachable from odrivetool or the GUI — goes through `RxSdo`.

| | id | payload |
|---|---|---|
| `RxSdo` (us -> drive) | `0x004` | `opcode(1) | endpoint_id(2, LE) | pad(1) | value(4)` |
| `TxSdo` (drive -> us) | `0x005` | `0(1) | endpoint_id(2, LE) | pad(1) | value(4)` |

`opcode` is `0x00` to read, `0x01` to write. A read sends 4 bytes and the reply
arrives on `0x005`. A function call (`save_configuration`) is a write with no
value — 4 bytes.

Writing `spi_encoder0.config.max_error_rate = 0.1` on node 1:

```
0x024   01 A1 02 00  CD CC CC 3D
        ^  ^^^^^ ^   ^^^^^^^^^^^
        |  ep    pad float32 LE
        write
```

### Endpoint numbers are version-specific — this is the dangerous part

Endpoint IDs are assigned by position in the firmware's parameter tree. They
move with **every** firmware and hardware revision. Writing a stale number does
not fail — it lands on whatever parameter now occupies that slot and silently
corrupts it.

The numbers hard-coded in `test_leg_can.c` came from `flat_endpoints.json` for
**fw 0.6.12 / hw 5.2.0**:

| endpoint | id | type |
|---|---|---|
| `spi_encoder0.config.max_error_rate` | 673 | float |
| `save_configuration` | 718 | function |

Get the file matching your drives from the firmware release page, then verify
with `python -c "import json;d=json.load(open('flat_endpoints.json'));print(d['fw_version'],d['hw_version'],d['endpoints']['<path>'])"`.

So `odrv_check_version()` interrogates every drive with `Get_Version` (`0x000`)
before writing anything, and skips any drive whose reply does not match exactly:

```
Get_Version reply: [0] reserved  [1] hw_product_line  [2] hw_version
                   [3] hw_variant  [4] fw_major  [5] fw_minor
                   [6] fw_revision  [7] fw_unreleased
```

A mismatch prints both versions and refuses the write. If your drives are on a
different build, update the six `EP_JSON_*` defines and the endpoint IDs
together — never one without the other.

### What this is currently used for

`spi_encoder0.config.max_error_rate` is raised from its default to `0.1`,
applied at boot to every drive that answers the scan. It is the fraction of SPI
transactions the drive tolerates coming back corrupt before it declares the
encoder estimate missing and disarms.

The knee dropped out mid-gait with `nan` in its position stream and
`err=0x00000008`, ending a run 1.6 s in. Loosening this stops a handful of bad
reads from killing a run. **It does not fix the link.** A connection at 10%
error rate is broken; this only buys a usable trace while the harness is
investigated. Two load-side AS5047Ps have now failed this way.

The write is deliberately **not** persisted — `LEGTEST_SDO_SAVE` is 0, so it is
re-applied every boot and stays visible in the log rather than hidden in a
drive's saved config. Set it to 1 to call `save_configuration`; that needs a
power cycle to re-init the encoder.

The firmware reads the value back after writing and prints `!! DID NOT TAKE` if
the drive rejected it, so a config that only applies on reboot is visible
immediately instead of being assumed.

---

## Defining the joint zero over CAN

`Set_Absolute_Position` (`0x019`), 4 bytes, float32 turns. Sent once from
`legtest_init()` while `LEGTEST_ZERO_ABSOLUTE_AT_BOOT` is 1, telling each drive
that where it is standing right now is zero.

    put_f32(data, 0.0f);
    can_send(node, ODRV_CMD_SET_ABS_POS, data, 4u);

`s_define_zero[]` picks which joints get it - currently `{ 1, 1, 0 }`, so hip
and knee are zeroed and the ankle is left alone.

**Put the hip and knee at mechanical zero before powering the STM32.** The
firmware prints that reminder at boot, because this command does not find zero,
it declares it. Whatever pose the leg is in when the board starts becomes the
origin of an absolute trajectory.

There is no acknowledgement frame - `0x019` is host->drive only - so the
firmware waits for fresh encoder telemetry and checks the drive now reports
within 1e-4 turns of zero. A joint that does not is reported and
`s_arm_blocked` is set, so a silent failure cannot become an absolute gait
played from an unknown origin:

    absolute joint reference setup
      Put HIP and KNEE at mechanical 0 deg before powering the STM32.
      hip_pitch CAN 0x19: Set_Absolute_Position(0.0 turns)
      hip_pitch reference: pos_estimate=+0.000000 turns  [OK]
      knee      CAN 0x19: Set_Absolute_Position(0.0 turns)
      knee      reference: pos_estimate=+0.000000 turns  [OK]
    absolute reference setup complete: HIP=0, KNEE=0

Nothing is saved to the drive, so this is re-established on every boot rather
than persisting - which is the right way round for a bench test, and the reason
the pose at power-on matters every single time.

---

## Velocity feedforward is per joint

`Set_Input_Pos` byte 4-5 is `Vel_FF`, int16, and the drive interprets it as
`input_vel = Vel_FF / input_vel_scale` with `input_vel_scale` defaulting to
1000. The SENDER therefore multiplies by 1000 so the drive's division lands on
the intended value - dividing here would send 0 and the feedforward would
vanish.

The firmware sends every joint the same quantity: trajectory velocity, scaled
by `GAIT_SPEED` and by `s_cmd_scale`, times 1000. Despite that, the right
amount differs per joint, measured at `GAIT_SPEED 1.0`:

| joint | RMS, FF 1.0 | RMS, FF 0.0 |
|---|---|---|
| hip_pitch | **0.13 deg** | 1.53 |
| knee | 0.58 | **0.46** |
| ankle | 1.88 | **0.26** |

The hip lags badly without it; the ankle overshoots 22% and arrives early with
it. `s_vel_ff[]` in `test_leg_can.c` scales it per joint.

Since the firmware treats them identically, the difference is on the DRIVE side.
`axis0.config.can.input_vel_scale` is per axis (endpoint 292) - if one drive's
is not 1000, its feedforward is multiplied by however far off it is. Read it on
each drive before accepting a per-joint scale as a real mechanical result.

---

## What we do not send

- **No torque commands.** `Set_Input_Torque` (`0x00E`) is never used.
- **No trajectory-mode commands.** No `Set_Traj_Vel_Limit` or friends;
  PASSTHROUGH means the ODrive does no planning of its own.
- **No `Set_Absolute_Position` during a run.** It is now sent ONCE at boot,
  before the timers start, to define the joint zero - see below. It is a
  homing command, not a control mode, and the control loop never issues it.
- **No persistent config writes.** Gains are pushed at arming and
  `max_error_rate` at boot, but both are runtime writes that die with the power.
  `save_configuration` is never called unless `LEGTEST_SDO_SAVE` is set, so what
  is saved in each drive stays what you saved from odrivetool.

### Note on the torque-control deployment plan

The deployment checklist describes a different architecture: `control_step_1kHz`
computing PD + feedforward **torque**, with `CTRL_LIM` in Nm, `TAU_FF_SCALE`,
and a generated `leg_feedforward.h` from `export_stm32_leg.py`. None of that
exists in this repo, and it is not what the firmware documented above does.

Those are two different control strategies and only one can be on the board:

| | this firmware | the checklist |
|---|---|---|
| STM32 sends | position + velocity feedforward | torque |
| loop closed by | ODrive (pos, vel, current) | STM32 (pos, vel) + ODrive (current) |
| gains live in | ODrive config, tuned and saved | STM32 firmware |
| tuning done so far | `pos_gain` 20, `vel_gain` 1.0, `vi` 5.0 | would not apply |

Decide which before writing any more code. Switching later means re-tuning from
scratch, because the ODrive-side gains that took this joint from 12.16° to
0.75° RMS error stop being used the moment the STM32 starts sending torque.
