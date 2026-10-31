# Handoff — `stm32_zeuss`, as of 2026-10-31

Written for an agent or engineer picking this up cold, in either this repo or
the Pi workspace. **Read [`AGENTS.md`](AGENTS.md) first** for the build
commands, the five things that will bite you, and the conventions. This file is
only *where things stand right now and what to do next*, which AGENTS.md
deliberately does not carry.

`HEAD` at writing: **`92339c6`**. CI green, all 14 host suites pass, firmware
builds.

---

## The one-paragraph version

Zeus is a **10-DOF biped**: five joints a leg — hip pitch, hip roll, knee
pitch, ankle pitch, **ankle roll** — driven by ODrive S1 over two CAN-FD buses.
There is **no waist**. An STM32H7 runs a 1 kHz loop doing all sensing, the
state estimate, the stored gait and every safety decision, and streams a
478-byte state packet to a Raspberry Pi 5 every millisecond over USB. The Pi
runs a trained RL policy that answers with a **residual** at 250 Hz. The board
drives each joint to `stored gait + residual`, so **a policy sending zeros
walks the stored gait**.

The firmware is now aligned with the policy: protocol v10, the policy's gait
table, its spring model, its command rate and its feedforward. **It has never
been flashed in this configuration and no joint has ever moved under it.**

## Where the work actually is

| | state |
|---|---|
| USB link | **done and measured**: 1000.0 Hz at the board, 100% delivered, worst gap 1.28 ms over 60 s |
| Protocol v10 | **done**: 478/52/130 bytes, 50-float policy block, `check_proto.py` agrees with the workspace field-for-field |
| Kinematics | **done**: 7-number joint vector per leg, matched to Pinocchio 2.7.0 |
| Gait + feedforward | **done**: position, velocity and torque tables, spring pre-deflection, commands at 500 Hz |
| Estimator | **done in simulation**: velocity rms 0.019 m/s, tilt rms 0.247°, height drift 0.015 m over 4.49 m |
| **Start sequencing** | **NOT DONE** — handoff item 4 |
| **Joint envelope** | **NOT DONE** — handoff item 6, still `±10` turns |
| **Gains** | **NOT DONE** — bench leg's values, wrong by ~3x and in unconfirmed units |
| **Calibration** | **NOT DONE** — and it blocks everything, see below |

The authoritative list is
`../zeus_26_mukesh_deploy/firmware_handoff/FIRMWARE_CHANGES.md`. Items 1, 2, 3,
5 (partly) and 7 are done. **Items 4 and 6 are not.**

## Three things that block the first walk

**1. `ROBOT_CONFIG_CALIBRATED` is 0** (`Appli/App/robot_config.h`). While it is
0 the estimator refuses to report `NEXUS_FUSION_OK` however well its covariance
converges — deliberately, because a converged filter on guessed geometry is
confidently wrong. And `rl_policy_node` **will not engage** until `fused_valid`
reads OK. So the policy cannot arm at all today.

Setting it to 1 honestly requires measuring, on the robot:

- every **sign and offset** in `g_leg_joints[2][5]` — all ten are `+1.0, 0.0`,
  which is another way of writing "not measured". Put the robot in the URDF's
  zero pose, read each drive, store the negated reading as the offset; then
  move each joint by hand in its URDF-positive direction (pitch about +Y, roll
  about +X) and flip the sign if the reading went the other way.
- every **spring encoder zero and sign** in `g_spring_enc[4]` — all
  `{0u, 1.0f}`, same meaning. Unload the leg, read the raw counts off the
  console, store them; then hold the motor and push the limb positive — the
  deflection must read positive.

**2. The firmware is built as `NEXUS_MODE_LEG_CAN`** (`Appli/App/nexus_mode.h`
line 40), the single-leg bench test. It only talks to bus 0 and most of the
packet reads as deliberately absent. **Set `NEXUS_MODE_ROBOT` and reflash**
before expecting anything from the right leg.

**3. CAN was last seen dead** — a bus scan found nothing, `TEC=0 REC=0`. `TEC=0`
is the informative half: the board never attempted a transmission that failed,
so it is not detecting a bus fault, there is simply nothing answering. That
predates the ISO1042B transceivers being fitted and the per-drive config pass,
so **retest before believing it**. All ten drives must answer, FD with BRS,
1 Mbit arbitration / 2.5 Mbit data, sample point 87.5%, and **node 5 on each
bus** is new (ankle roll) and may never have been configured.

## Item 4 — the start sequence, and why it needs a gantry

`s_phase` free-runs from boot, so arming happens at an arbitrary point in the
stride and jumps straight to `ref`. Both have to change:

1. hold `s_phase` at 0 until the drives are armed,
2. on arm, ramp each joint from where it is to `ref(0)`, then hold,
3. release the clock on a start condition.

The policy never trained a transition from standing. From rest at the reference
pose it survived from phase 0.95 to 0.30 and **fell within 1.5 s from 0.40,
0.50, 0.60, 0.75 and 0.90**. `rl_policy_node` will not engage outside that
window or with any joint more than 5° from `ref`.

**`ref(0)` has the left foot in the air** — single support — so the robot
cannot stand there unaided. The first run needs a gantry, with the robot
lowered as the clock releases. Start pose in degrees:

| | hip pitch | hip roll | knee | ankle pitch | ankle roll |
|---|---:|---:|---:|---:|---:|
| left | +6.26 | −6.19 | +19.00 | −26.18 | +7.13 |
| right | −17.01 | +6.76 | +18.14 | −1.22 | −6.27 |

## Item 6 — the joint envelope

`SAFETY_POS_MIN_TURNS` / `MAX_TURNS` are `±10` turns, placeholders for all ten
joints. `FIRMWARE_CHANGES.md` §6 has the per-joint reach of the gait plus the
largest possible residual. Clamp to the mechanical stop where that is tighter.
Ankle roll runs into the model's ±25° limit by design at full residual.

## Gains — read this before typing any in

The policy trained with KP/KD on the **output** shaft. The conversion to the
ODrive's own units is written out above `g_drive_gains[]` in `robot_config.c`,
and the short version is:

```
pos_gain = KP / KD          always — a ratio of encoder units, the gearbox cancels
vel_gain = KD · 2π / N      encoder on the load
vel_gain = KD · 2π / N²     encoder on the MOTOR  (both ankles)
```

| class | KP | KD | `pos_gain` | `vel_gain` |
|---|---:|---:|---:|---:|
| hip pitch, knee (47:1) | 150 | 12 | 12.5 | 1.6042 |
| hip roll (26:1) | 60 | 4 | 15.0 | 0.9666 |
| ankle pitch, ankle roll (15:1) | 30 | 1.2 | 25.0 | 0.0335 |

`vel_integrator_gain` is **0** — training had no integrator, so the `50.0f` in
the table is not a smaller version of it, it is a term that did not exist.

**The one open question.** This assumes hip pitch, knee and hip roll report
position at the load and the ankles at the motor, per `docs/ODRIVE_COMMANDS.md`.
`robot_config.h` claims every drive reports output-shaft position with the ratio
already applied, which contradicts it for the ankles, and **hip roll's encoder
placement has never been measured at all**. The drives themselves are the
authority. A `vel_gain` wrong by N² is a factor of 225.

Torque is settled: `Torque_FF` is **always motor torque**, so what goes on the
wire is `g_gait_tau / N`.

## Still unmeasured, and what each one costs

| quantity | where | why it matters |
|---|---|---|
| joint signs and offsets | `g_leg_joints` | blocks arming; a wrong sign drives a leg into the floor |
| spring zeros and signs | `g_spring_enc` | corrupts `joint_pos` and the torque estimate by up to ±π |
| spring rate and travel | `ROBOT_SPRING_*` | 3437.75 Nm/rad and ±1.7° are the *policy's* numbers, not this robot's |
| foot switch debounce | `contact.c`, 3/8 ticks | feeds the contact constraint, so it sets the velocity estimate too |
| real torque/velocity limits | the URDF's 25 Nm / 20 rad/s | placeholders; a policy trained with more torque than exists fails instantly |
| rotor inertia, joint friction | absent from the URDF | reflected rotor inertia scales by N², so 225× at the ankles |
| mass and CoM | never weighed | sets the pendulum frequency, which sets the gait timing |
| command latency | never measured | Pi → USB → board → CAN → motion. Policies are very sensitive to action delay |

The **hip load encoder reads garbage** and the prime suspect is mechanical:
`docs/PINOUT.md` has the magnet checklist (6 × 2.5 mm diametric, centred within
0.5 mm, airgap 0.5–3 mm, **non-ferromagnetic holder** — a steel screw nearby
warps the field and the chip reports no error at all).

## Things that will waste your time

AGENTS.md has the five big ones (USBREGEN cleared, OTG DMA off, no synchronous
printf in the loop, XSPI2 not re-initialised, DMA buffers non-cacheable). These
are newer:

- **`tools/model/mukesh.urdf` is vendored**, and `gen_kinematics.py` reads that
  copy, not the workspace's. Refresh all four steps together or CI fails —
  `tools/model/README.md` has them. **Run
  `tools/model/check_drift.sh` after pulling the workspace**: it is the one
  model check CI cannot do, because the workspace is a private repo owned by
  another personal account and a fine-grained token cannot be scoped to one.
- **Commands go out at 500 Hz**, not 1 kHz (`ODRV_CMD_DIVIDER` in
  `act_odrive.h`). Tests counting frames on the wire use the `cmd_tick()`
  helper; tests measuring elapsed milliseconds use raw `ticks()`.
- **`ref_angle` in the packet is the pure gait**, deliberately not the
  pre-deflected value the drives get. `rl_policy_node` validates it to 1°.
- **The gait table must not be smoothed.** The knee steps 0.475° across the
  stride wrap. The policy trained on that exact table.
- **`test_leg_can.c` holds the user's uncommitted bench edits. Stage only your
  own changes.** Its `s_cmd_scale[]` still reads `9.0f` and needs `15.0f` — the
  ankles are 15:1 — but that is the user's file to change.
- **`tools/sim/` is a KINEMATIC walk, not physics.** Nothing in it balances or
  falls over; it tests whether the estimator can track a known motion. The
  "falls at 1.5 s" figures come from the *policy's* MuJoCo sim in the
  workspace, which is a different program.

## Commands

```bash
for d in /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.{gnu-tools-for-stm32,ninja,cmake,cubeprogrammer}.*/tools/bin; do PATH="$d:$PATH"; done; export PATH

bash tools/hosttest/run.sh            # 14 suites, pure host C, no hardware
python3 tools/check_proto.py          # C struct vs the workspace's Python
cmake --build Appli/build

# the walk sim and the kinematics reference need Pinocchio (~/kinvenv here)
env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py --check
PY=~/kinvenv/bin/python EVAL_PY=python3 URDF=tools/model/mukesh.urdf \
  env -u PYTHONPATH tools/sim/run.sh --check

ST_LOADER=$(ls /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr)
STM32_Programmer_CLI -c port=SWD mode=UR -el "$ST_LOADER" -d Appli/build/nexus_first_Appli.hex -v -rst
```

Console is the ST-LINK virtual COM port, 115200 8N1. `link_usb_diag()` prints
the USB supply rail, control bits and device state there — readable exactly when
the host can tell you nothing.

## What I would do next, in order

1. **Retest CAN** with all ten drives powered. Nothing below matters until they
   answer. Configure node 5 on both buses.
2. **Switch to `NEXUS_MODE_ROBOT`** and reflash.
3. **Settle the torque/encoder-shaft question** at the drives — hang a known
   mass on a known arm, compare reported torque with `m·g·r`.
4. **Measure signs and offsets**, then spring zeros, then set
   `ROBOT_CONFIG_CALIBRATED` to 1. About half a day, and it unblocks arming.
5. **Item 6** — the per-joint envelope. Cheap, and it is what stops a wrong
   sign becoming a broken leg.
6. **Item 4** — the start sequence. Needs the gantry.
7. **Gains**, on the assembled robot with weight on the legs, in the order
   `zeus tune` ranks: `vel_gain` until `stuck` is 0, then `pos_gain` until
   `lag_ms` < 15, then leave the integrator at 0.
8. **Then** a first walk attempt, on the gantry, with the policy sending zeros.
