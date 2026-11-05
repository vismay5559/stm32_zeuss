# Handoff — `stm32_zeuss`, as of 2026-11-06

Written for an agent or engineer picking this up cold. **Read
[`AGENTS.md`](AGENTS.md) first** for the build commands, the five things that
will bite you, and the conventions. This file is only *where things stand and
what to do next*, which AGENTS.md deliberately does not carry.

CI green, all 14 host suites pass, firmware builds, walk simulation passes.

---

## The robot this firmware is built for

**8 actuated joints, four a leg** — hip pitch, hip roll, knee pitch, ankle
pitch — on ODrive S1 over two CAN-FD buses, four nodes each. **No waist, and no
ankle roll.** Protocol **v9**, 426-byte state packet at 1 kHz, 44-byte command
at ~250 Hz.

> **This repo was briefly taken to 10 joints** (ankle roll on node 5, protocol
> v10) for the RL policy in `zeus_26_mukesh_deploy`, and **reverted on
> 2026-11-06** because the robot went back to the eight-actuator design. If you
> need any of that work it is in git history — `03285b8` through `6feb3b9`, and
> `200d598` in particular has the torque feedforward, the spring pre-deflection
> and the 500 Hz command rate. **Do not take facts from those commits without
> checking them against the current tree.**

What survived the revert, because it is true at any joint count: the **gear
ratios** (47:1 hip pitch and knee, 26:1 hip roll, **15:1 ankle**), the
**spring travel and stiffness**, the **IMU on USART2**, the vendored model and
drift check, and this file.

## Where the work actually is

| | state |
|---|---|
| USB link | **done and measured**: 1000.0 Hz at the board, 100% delivered, worst gap 1.28 ms over 60 s |
| Protocol v9 | **done**: `check_proto.py` agrees C against Python, 8 joints |
| Kinematics | **done**: 6-number joint vector per leg, matched to Pinocchio 2.7.0 |
| Estimator | **done in simulation**: velocity rms 0.018 m/s, tilt rms 0.246°, height drift 0.014 m over 4.38 m |
| CAN | **wired, never confirmed** — see below |
| IMU | **wired, never confirmed** |
| Spring encoders | **not wired yet** |
| **Calibration** | **NOT DONE**, and it gates the estimator |
| **Gains** | bench leg's values, and the ankle's predate the 15:1 gearbox |

## Three things to confirm before anything moves

**1. `ROBOT_CONFIG_CALIBRATED` is 0** (`Appli/App/robot_config.h`). While it is
0 the estimator will not report `NEXUS_FUSION_OK` however well its covariance
converges — deliberately, because a converged filter on guessed geometry is
confidently wrong.

It needs, measured on the robot:

- every **sign and offset** in `g_leg_joints[2][4]` — all eight are
  `+1.0, 0.0`, which is another way of writing "not measured". Put the robot in
  the URDF's zero pose, read each drive, store the negated reading as the
  offset; then move each joint by hand in its URDF-positive direction (pitch
  about +Y, roll about +X) and flip the sign if the reading went the other way.
- every **spring encoder zero and sign** in `g_spring_enc[4]` — all
  `{0u, 1.0f}`, same meaning. Those four are not wired yet.

**2. The firmware is built as `NEXUS_MODE_LEG_CAN`** (`Appli/App/nexus_mode.h`
line 40), the single-leg bench test. It only talks to bus 0 and most of the
packet reads as deliberately absent. **Set `NEXUS_MODE_ROBOT` and reflash**
before expecting anything from the right leg.

**3. CAN has never been confirmed working.** The last scan found nothing,
`TEC=0 REC=0` — and `TEC=0` is the informative half: the board never attempted
a transmission that failed, so it was not detecting a bus fault, there was
simply nothing answering. That predates the ISO1042B transceivers being fitted,
so **retest**. Expect **exactly 8 drives**, nodes 1–4 on each bus; anything
answering on node 5 is a drive still configured from the ten-joint attempt.

## Gains — read before typing any in

The ankle's `17.0 / 0.3 / 1.5` was measured on a **9:1** ankle and this robot's
is **15:1**, so it is worse than untuned. Hip roll has never been tuned at all
and starts where hip pitch did.

The open question is **which shaft each drive's numbers refer to**:
position and velocity are turns of whatever shaft that drive's encoder sits on,
which is not the same shaft on every joint, while torque is always **motor**
torque. So `pos_gain` is a ratio of encoder units and the gearbox cancels, but
`vel_gain` does not — it is `KD·2π/N` on a load-side encoder and `KD·2π/N²` on
a motor-side one, a factor of 225 apart at the ankle.

`robot_config.h` claims every drive reports output-shaft position with the
ratio already applied; `docs/ODRIVE_COMMANDS.md` says the ankle is motor-side.
**The drives are the authority.** Settle it with a known mass on a known lever
arm, comparing the drive's reported torque against `m·g·r`.

## Still unmeasured, and what each one costs

| quantity | where | why it matters |
|---|---|---|
| joint signs and offsets | `g_leg_joints` | gates the estimator; a wrong sign drives a leg into the floor |
| spring zeros and signs | `g_spring_enc` | corrupts `joint_pos` and the torque estimate by up to ±π |
| spring rate and travel | `ROBOT_SPRING_*` | 3437.75 Nm/rad and ±1.7° are modelled, not weighed |
| foot switch debounce | `contact.c`, 3/8 ticks | feeds the contact constraint, so it sets the velocity estimate too |
| real torque/velocity limits | the URDF's 25 Nm / 20 rad/s | placeholders |
| rotor inertia, joint friction | absent from the URDF | reflected rotor inertia scales by N², so 225× at the ankle |
| mass and CoM | never weighed | sets the pendulum frequency, which sets the gait timing |
| command latency | never measured | Pi → USB → board → CAN → motion |

The **hip load encoder reads garbage** and the prime suspect is mechanical:
`docs/PINOUT.md` has the magnet checklist (6 × 2.5 mm diametric, centred within
0.5 mm, airgap 0.5–3 mm, **non-ferromagnetic holder** — a steel screw nearby
warps the field and the chip reports no error at all).

## Things that will waste your time

AGENTS.md has the five big ones. These are newer:

- **`tools/model/zeus.urdf` is vendored**, and `gen_kinematics.py` reads that
  copy, not the workspace's. Refresh all four steps together or CI fails —
  `tools/model/README.md` has them. **Run `tools/model/check_drift.sh`** after
  pulling the workspace; it compares all four hashes and is the one model check
  CI may not be able to do.
- **`tools/sim/` is a KINEMATIC walk, not physics.** Nothing in it balances or
  falls over; it tests whether the estimator can track a known motion.
- **`test_leg_can.c` holds the user's uncommitted bench edits. Stage only your
  own changes.** Its `s_cmd_scale[]` still reads `9.0f` and needs `15.0f`, but
  that is the user's file to change.
- **The spring guard is 0.0297 rad (±1.7°), not 0.35.** Anything feeding a
  larger deflection — a test fixture, the walk simulation — is simulating a
  spring this robot does not have, and the board will correctly reject it.

## Commands

```bash
for d in /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.{gnu-tools-for-stm32,ninja,cmake,cubeprogrammer}.*/tools/bin; do PATH="$d:$PATH"; done; export PATH

bash tools/hosttest/run.sh            # 14 suites, pure host C, no hardware
python3 tools/check_proto.py          # C struct against the Python copy
cmake --build Appli/build

env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py --check
PY=~/kinvenv/bin/python EVAL_PY=python3 URDF=tools/model/zeus.urdf \
  env -u PYTHONPATH tools/sim/run.sh --check
tools/model/check_drift.sh

ST_LOADER=$(ls /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr)
STM32_Programmer_CLI -c port=SWD mode=UR -el "$ST_LOADER" -d Appli/build/nexus_first_Appli.hex -v -rst
```

Console is the ST-LINK virtual COM port, 115200 8N1. `link_usb_diag()` prints
the USB supply rail, control bits and device state there — readable exactly when
the host can tell you nothing.

## What I would do next, in order

1. **Confirm CAN and the IMU without arming anything** — the drives answering
   and the IMU framing, with every axis still idle. That is the next job and it
   needs no calibration and no motion.
2. **Wire the four spring encoders**, then check each one reads and sweeps
   smoothly by hand before trusting any of them.
3. **Settle the torque/encoder-shaft question** at the drives.
4. **Measure signs and offsets**, then spring zeros, then set
   `ROBOT_CONFIG_CALIBRATED` to 1.
5. **Per-joint `SAFETY_POS_MIN/MAX_TURNS`** — still `±10` turns, and it is what
   stops a wrong sign becoming a broken leg.
6. **Gains**, on the assembled robot with weight on the legs, in the order
   `zeus tune` ranks: `vel_gain` until `stuck` is 0, then `pos_gain` until
   `lag_ms` < 15.
