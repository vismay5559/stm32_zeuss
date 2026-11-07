# Zeus firmware — orientation for an agent picking this up cold

This is the **firmware** half of Zeus, a bipedal robot. It runs on an
STM32H7S3L8H6 (NUCLEO-H7S3L8, board MB1737) and talks to a Raspberry Pi 5 over
one USB cable.

This file is **self-contained**: the conventions, the traps, and enough of the
system to work on it. **[`HANDOFF.md`](HANDOFF.md) is the current state** — what
is done, what is not, and what to do next. Read that second.

If a fact here contradicts the code, **the code wins and this file is a bug.**

**Push directly to `main`. Do not create branches.**

---

## The robot

**8 actuated joints, four a leg**: hip pitch, hip roll, knee pitch, ankle
pitch. **No waist and no ankle roll.** Driven by **ODrive S1** over **CAN-FD**,
two buses — FDCAN1 is the left leg, FDCAN2 the right, four nodes each.

- **Joint index = `bus * 4 + (node - 1)`**, the single source of truth for every
  per-joint array in both packets. Written down once in `link_proto.h` as
  `NEXUS_J_*` and once in `pi/nexus_proto.py` as `JOINT_NAMES`;
  `tools/check_proto.py` fails if they disagree.
- **Gear ratios**: 47:1 hip pitch and knee, 26:1 hip roll, **15:1 ankle**. The
  ankle's encoder is on the **motor** side of its gearbox, so its position,
  velocity and torque are all in motor units. Everything else reads the load.
  (It said 9:1 here until 2026-10-31. It is 15:1.)
- **Series-elastic** hip pitch and knee pitch: the drive reports the motor side,
  an **AS5047P** reports the spring's deflection, and the real joint angle is
  their sum. Four encoders share SPI1 with **one chip select each**
  (PF1 / PD15 / PD14 / PF5). Spring is **3437.75 Nm/rad**, travel **±1.7°**.
- **BNO085 IMU** on the torso, SHTP over **USART2** (PD5/PD6) at 3 Mbaud. Not
  USART1: PA9 carries I2C_SDA to the TCPP03-M20 USB-C controller through solder
  bridge SB35, closed by default.
- **2 foot switches**, one per foot, at the centre of the sole.
- The estimator's joint vector is **six per leg** (`ZEUS_KIN_NQ`): four drives
  plus the two spring deflections. Five actuators a leg would be ankle roll,
  which this robot does not have.

**Every pin into the STM32** is in [`docs/PINOUT.md`](docs/PINOUT.md). That is
the soldering reference.

## The Pi side

**[`zeus_8dof_deploy`](https://github.com/Maverick280306/zeus_8dof_deploy)** is
the live ROS 2 workspace. It holds the RL policy (`z1_softland_tol05`), the
URDF, and `zeus_link`, which is the only thing that opens the serial port.

> **Two older workspaces are retired. Do not read them for current facts and do
> not change them**: `vismay5559/zeus_26` (the pre-policy workspace) and
> `Maverick280306/zeus_26_mukesh_deploy` (a ten-joint robot with ankle roll,
> protocol v10, that this project no longer builds).

**The STM32 does the robot. The Pi does the policy.** The board owns all
real-time sensing, the state estimate, the stored gait and every safety
decision. The policy answers with a **residual**:

```
drive target = stored gait (ref_angle) + residual
```

So a policy sending zeros walks the stored gait, and a policy that stops
answering stops the robot.

## The link

USB CDC. The board is the device, the Pi is the host.

- USB ID **`0483:5740`**. `0483:374x` is the **ST-LINK** — a different chip, the
  debugger and text console, not the data link.
- **Protocol v9.** State packet **426 bytes, board → Pi, every 1 ms**. Command
  **44 bytes**, Pi → board, ~250 Hz. Gains message 106 bytes.
- The state packet opens with a contiguous **44-float32 policy block** at byte
  offset 12, so the policy takes one array with no per-field parsing. This is
  exactly the observation the policy trains on — `zeus_env.py`'s own docstring
  says "44 raw SI values, same order".
- **The version byte is load-bearing.** A board and a workspace at different
  protocol versions refuse to talk and say so rather than misreading bytes.
  Flash and deploy together.
- **One reader per port.** Two processes on one tty do not each get a copy; the
  port opens `exclusive=True` so the second fails with a clear message.

## What the board sends the drives

Per drive, every **2 ms (500 Hz)**, ODrive CANSimple `Set_Input_Pos`:

| bytes | field | units | filled from |
|---|---|---|---|
| 0–3 | `Input_Pos` | turns of **that drive's encoder shaft** | `ref_angle + residual − tau/K`, after safety and the interpolator |
| 4–5 | `Vel_FF` | int16, 0.001 turns/s | `g_gait_vel` at phase |
| 6–7 | `Torque_FF` | int16, 0.001 **motor** Nm | `g_gait_tau` at phase **÷ gear ratio** |

**500 Hz, not 1 kHz**, because that is `DT_PHYS` in the policy's `zeus_env.py`
(`ODRV_CMD_DIVIDER` in `act_odrive.h`). The loop is still 1 kHz; only this
message is halved.

Hip pitch and knee are commanded to **`ref − tau/K`** — the drive holds the
motor side and the limb sits behind the spring, so without it the limb trails
the reference for the whole of stance. `ref_angle` **in the packet stays the
pure gait**, because the policy reads it as an observation and refuses to
engage if it differs from its own copy by more than 1°.

Both int16 fields are **clamped before the cast**: an out-of-range float cast to
int16 in C wraps sign, turning maximum forward torque into maximum reverse.

## Repo map

| path | what |
|---|---|
| `Boot/` | internal flash. Sets up XSPI2, maps the external flash, jumps. Rarely changes |
| `Appli/App/` | **everything worth reading.** Every header opens with a plain description; `Appli/App/README.md` explains the vocabulary in non-specialist language |
| `pi/` | the Python side, droppable onto any machine with pyserial. `nexus_proto.py` is **canonical**; `bench_monitor.py` is the no-ROS terminal view |
| `tools/hosttest/` | 15 suites of pure host C, no hardware |
| `tools/model/` | the **vendored** URDF and gait library, their sha256s, and `check_drift.sh` |
| `docs/` | PINOUT, the ODrive command reference, the CAN and zeroing procedures |

Key files in `Appli/App/`:

| file | what |
|---|---|
| `link_proto.h` | **the protocol.** Both packet structs, every constant, byte offsets |
| `app.c` | the 1 kHz robot loop, `build_reference()`, the spring pre-deflection |
| `robot_config.h/.c` | what CAD cannot know: signs, offsets, gearing, the spring, the per-joint envelope |
| `safety.c` | what is allowed to move, and why not |
| `fusion.c`, `inekf.c`, `lie_group.c` | contact-aided right-invariant EKF (Hartley et al. 2019) |
| `zeus_kinematics.c` + `_model.h` | forward kinematics and the exact Jacobian. **`_model.h` is GENERATED** |
| `gait_ref.c/.h` | the stored gait. **GENERATED** |
| `act_odrive.c` | CAN to the drives, the interpolator, the feedforward |
| `nexus_mode.h` | **which program the firmware is**: `NEXUS_MODE_ROBOT`, `_LEG_CAN`, `_LEG_TORQUE`, `_IMU` |
| `test_leg_can.c` | the single-leg bench test. **The user leaves uncommitted edits here** |

## Build, test, flash

```bash
for d in /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.{gnu-tools-for-stm32,ninja,cmake,cubeprogrammer}.*/tools/bin; do PATH="$d:$PATH"; done; export PATH

bash tools/hosttest/run.sh          # 15 suites, pure host C, no hardware needed
python3 tools/check_proto.py        # the C struct against the Python one, field by field
cmake --build Appli/build

ST_LOADER=$(ls /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr)
STM32_Programmer_CLI -c port=SWD mode=UR -el "$ST_LOADER" -d Appli/build/nexus_first_Appli.hex -v -rst
```

Only the Appli needs the external loader. Console is the ST-LINK virtual COM
port at 115200 8N1.

**Check `Voltage` in the programmer's output before anything else.** `0.00V`
means the STM32 has no power and there is no point retrying — the ST-LINK will
still report its own serial and firmware quite happily.

### Building a variant without committing one

`NEXUS_MODE`, `NEXUS_LOOP_STATS` and `HEALTH_EXPECTED_MASK` are **environment
variables at configure time**. Build into a side directory so the committed
build is never the odd one:

```bash
NEXUS_MODE=NEXUS_MODE_IMU NEXUS_LOOP_STATS=1 \
HEALTH_EXPECTED_MASK='(HEALTH_TIMING|HEALTH_IMU|HEALTH_CAN1|HEALTH_CAN2)' \
cmake -S Appli -B Appli/build-bench \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build Appli/build-bench
```

`build-*/` is gitignored. **Keep `NEXUS_MODE_ROBOT` committed**: a board flashed
with a test mode answers no USB packets, which from the Pi is indistinguishable
from a dead link. `HEALTH_EXPECTED_MASK` narrows what must be healthy before the
robot may arm — useful while subsystems are unwired, and a failsafe that is not
watching otherwise.

### Pinocchio

Installs **only at pinned versions**: `pin==2.7.0` with `numpy<2`, in
`tools/requirements-kinematics.txt`. Every 3.x/4.x wheel dies on import with
`undefined symbol: EIGENPY_ARRAY_APIPyArray_RUNTIME_VERSION`. There is a working
venv at `~/kinvenv`.

```bash
env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py --check
PY=~/kinvenv/bin/python EVAL_PY=python3 URDF=tools/model/zeus.urdf \
  env -u PYTHONPATH tools/sim/run.sh --check
```

`env -u PYTHONPATH` matters when ROS is sourced: ROS's eigenpy breaks
Pinocchio's.

## Generated files — do not hand-edit

| file | generator | source |
|---|---|---|
| `Appli/App/zeus_kinematics_model.h`, `tools/hosttest/zeus_kinematics_ref.h` | `tools/gen_kinematics.py` | `tools/model/zeus.urdf` |
| `Appli/App/gait_ref.c/.h` | `tools/gen_gait_from_library.py` | `tools/model/gait_library.npz` |

Both sources are **vendored** in `tools/model/` with their sha256 pinned beside
them, because this repo is private and the generators then need nothing
external — a regeneration here and one in CI read the same bytes. CI checks the
generated files against both, unconditionally.

`tools/model/check_drift.sh` compares the vendored copies against the
workspace's, which is the one model check CI may not be able to do. Run it after
pulling the workspace. `tools/model/README.md` has the refresh procedure — all
four steps together, or CI fails.

**`tools/gen_gait.py` and `tools/gen_gait_sheet_select.py` are LEGACY**: they
built the old 1.26 s `final_humangait.xlsx` gait. The live gait comes from the
policy's library at a **1.857507 s** stride. Do not regenerate with them.

## The five things that will bite you

Each has a comment in the code explaining it. **Do not undo them.**

1. **`USBREGEN` must be CLEARED.** `VDD33USB` is supplied externally on this
   board. Setting it — which CubeMX and every ST example do — leaves the USB
   transceiver unpowered, `USB33RDY` stuck at 0, and the host seeing nothing at
   all. Indistinguishable from a dead cable. See `usbd_conf.c`.
2. **The OTG core's DMA is off on purpose.** D-cache is on and the control
   endpoint buffers are cached, so the core would DMA each SETUP packet into a
   line the CPU reads stale. Symptom: attaches, then `descriptor read error -32`.
3. **Never `printf` synchronously from the 1 kHz loop.** The console UART is
   polled — 87 µs per character. `console.c` makes it asynchronous; a burst that
   outruns the wire is dropped and counted, never waited for.
4. **XSPI2 must not be re-initialised in Appli.** The code executes through it;
   re-initialising hangs the bus forever with no fault and no message.
5. **DMA buffers must be non-cacheable** — `NEXUS_DMA_BUFFER` in
   `dma_buffer.h`, which must agree with MPU region 2 in Boot. `app.c` checks
   that agreement at startup rather than trusting it.

When the link will not enumerate, `link_usb_diag()` prints the supply rail, the
control bits and the USB device state on the ST-LINK console — readable exactly
when the host can tell you nothing.

## CubeMX

It regenerates `main.c`, `main.h` and the MSP files, and will lose anything
outside `USER CODE` blocks. Several deliberate deviations from what it generates
are documented in place — read the comment before "fixing" something that looks
wrong.

Pin names are CubeMX's and do not always match what is wired. `L_TOE`/`R_TOE`
are the **single foot switch on each foot**; `L_HEEL`/`R_HEEL` are unconnected.
Renaming needs the `.ioc`, which is a GUI action, so the real mapping is written
out in `contact_init()` instead.

## Conventions

- **Comments explain _why_, and often name the bug they prevent.** Keep them
  when editing around them; delete one only if it has become false.
- **Tests state their verdict first.** Cases are hand-built with the expected
  answer written down before the maths runs, and the comment says what a failure
  would mean on the real robot. A metric that is quietly wrong sends someone to
  tune the wrong gain on a real leg.
- **Derive, do not hardcode.** Several bugs here were a literal that was right
  for one robot: a test perturbing `q[7]` while asserting on the joint that
  index used to be, an envelope sized for a ten-turn placeholder, a `10` where
  the joint count belonged. If a number can be computed from the protocol or the
  tables, compute it.
- **Protocol changes bump `NEXUS_PROTO_VERSION`** and must land in
  `link_proto.h`, `pi/nexus_proto.py` (**canonical**), the copy in
  `zeus_8dof_deploy/zeus_link/`, and `NexusState.msg` **together**.
  `check_proto.py` enforces the first two.
- **Stage only your own changes.** The user keeps bench edits uncommitted in
  `Appli/App/test_leg_can.c`. Its `s_cmd_scale[]` still reads `9.0f` and needs
  `15.0f`, but that is the user's file to change.
- **Verify the way CI does**: grep the whole tree after a rename, run the host
  tests from the repo root, and check CI with `gh run list` after pushing.

## Where things stand

See **[`HANDOFF.md`](HANDOFF.md)** — kept separate so this file can stay true
while the state moves. The short version at the time of writing: the link, the
protocol, the kinematics, the gait and the feedforward are done and verified
against the policy bundle itself; the start sequence, the calibration tables and
the drive gains are not; CAN has never been confirmed working end to end.
