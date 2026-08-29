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
CAN-FD with BRS, 1 Mbit arbitration / 5 Mbit data.

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

`Torque_FF` is **always 0** at present: the trajectory spreadsheet has no torque
column. The field is plumbed so adding gravity compensation later is a value
change rather than a protocol change.

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
does not, until someone measures it. See Test 6 in `CAN_BUS_TEST.md`.

---

## What we receive

| frame | bytes | contents |
|---|---|---|
| `Get_Encoder_Estimates` `0x009` | 0..3 / 4..7 | `float32` position (turns), `float32` velocity (turns/s) |
| `Get_Torques` `0x01C` | 0..3 / 4..7 | `float32` target torque, `float32` estimated torque (Nm) |
| `Heartbeat` `0x001` | 0..3 / 4 / 5 / 6 | `uint32` axis_error, `uint8` axis_state, `uint8` procedure_result, `uint8` trajectory_done |

Required rates, per axis:

```python
odrv0.axis0.config.can.encoder_msg_rate_ms   = 1
odrv0.axis0.config.can.torque_msg_rate_ms    = 1
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
```

Encoder position at 1 kHz is what the capture logs against the command; the
heartbeat is what proves the axis is still armed and error-free.

---

## Bus budget

| direction | frames/s |
|---|---|
| STM32 → drives, `Set_Input_Pos` 1 kHz × 3 | 3 000 |
| drives → STM32, encoder + torque 1 kHz, heartbeat 10 Hz, × 3 | ~6 030 |
| **total** | **~9 030** |

Measured ~15% with one node; budget ~45% with three. The TX path is a software
ring drained by `tx_pump()` because the hardware FIFO holds three entries and a
tick hands over more than that. When the ring is full it discards the **oldest**
entry: these are position setpoints, a stale one is worthless, and dropping the
newest would leave the queue replaying seconds-old commands after a recovery.

---

## What we do not send

- **No torque commands.** `Set_Input_Torque` (`0x00E`) is never used.
- **No trajectory-mode commands.** No `Set_Traj_Vel_Limit` or friends;
  PASSTHROUGH means the ODrive does no planning of its own.
- **No `Set_Absolute_Position`.** That is a homing command, not a control mode.
- **No config writes.** Gains, limits and message rates are set in odrivetool
  and persist in the drive. The firmware never changes them, so what you tuned
  is what runs.

### Note on the torque-control deployment plan

The deployment checklist describes a different architecture: `control_step_1kHz`
computing PD + feedforward **torque**, with `CTRL_LIM` in Nm, `TAU_FF_SCALE`,
and a generated `leg_feedforward.h` from `export_stm32_leg.py`. None of that
exists in this repo, and it is not what the firmware documented above does.

Those are two different control strategies and only one can be on the board:

| | this firmware | the checklist |
|---|---|---|
| STM32 sends | position + vel feedforward | torque |
| loop closed by | ODrive (pos, vel, current) | STM32 (pos, vel) + ODrive (current) |
| gains live in | ODrive config, tuned and saved | STM32 firmware |
| tuning done so far | `pos_gain` 20, `vel_gain` 1.0, `vi` 5.0 | would not apply |

Decide which before writing any more code. Switching later means re-tuning from
scratch, because the ODrive-side gains that took this joint from 12.16° to
0.75° RMS error stop being used the moment the STM32 starts sending torque.
