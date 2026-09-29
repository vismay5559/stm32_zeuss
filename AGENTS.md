# Zeus firmware — orientation for an agent picking this up cold

This is the **firmware** half of Zeus, a bipedal robot. It runs on an
STM32H7S3L8H6 (NUCLEO-H7S3L8, board MB1737) and talks to a Raspberry Pi 5 over
one USB cable.

**The full orientation document lives in the other repo:
[`zeus_26/AGENTS.md`](https://github.com/vismay5559/zeus_26/blob/main/AGENTS.md).**
Read that first — it covers the hardware, the protocol, both repo maps, the
working conventions and the current state of the robot. This file is only the
firmware-specific quick start, so the two do not drift.

Push directly to `main`. Do not create branches.

## Two projects

- **`Boot/`** — internal flash. Sets up XSPI2, maps the external flash, jumps.
  Rarely changes.
- **`Appli/`** — executes in place from external flash at `0x70000000`.
  Everything worth reading is in **`Appli/App/`**, and every header there opens
  with a plain description of what it is for. `Appli/App/README.md` explains the
  vocabulary in non-specialist language.

`Appli/App/nexus_mode.h` decides **which program the firmware is**:
`NEXUS_MODE_ROBOT`, `_LEG_CAN` (single-leg bench test), `_LEG_TORQUE`, `_IMU`.

## Build, test, flash

```bash
for d in /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.{gnu-tools-for-stm32,ninja,cmake,cubeprogrammer}.*/tools/bin; do PATH="$d:$PATH"; done; export PATH

bash tools/hosttest/run.sh          # 14 suites, pure host C, no hardware needed
python3 tools/check_proto.py        # the C struct against the Python one, field by field
cmake --build Appli/build

ST_LOADER=$(ls /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr)
STM32_Programmer_CLI -c port=SWD mode=UR -el "$ST_LOADER" -d Appli/build/nexus_first_Appli.hex -v -rst
```

Only the Appli needs the external loader. Console is the ST-LINK virtual COM
port at 115200 8N1.

## The five things that will bite you

Each has a comment in the code explaining it. Do not undo them.

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

## Generated files — do not hand-edit

`Appli/App/zeus_kinematics_model.h` and `tools/hosttest/zeus_kinematics_ref.h`
come from `tools/gen_kinematics.py`, which reads `zeus_26`'s URDF. CI checks the
URDF sha256 embedded in the model header against that URDF, so the firmware
cannot silently be walking an older robot.

Pinocchio's current wheels fail to import (eigenpy built against a mismatched
numpy ABI). `gen_kinematics.py --model-only` regenerates the firmware tables
with numpy alone; only the host test's reference and the walk simulation need
Pinocchio, so those may only be runnable in CI.

## CubeMX

It regenerates `main.c`, `main.h` and the MSP files, and will lose anything
outside `USER CODE` blocks. Several deliberate deviations from what it generates
are documented in place — read the comment before "fixing" something that looks
wrong.

Pin names are CubeMX's and do not always match what is wired. `L_TOE`/`R_TOE`
are the **single foot switch on each foot**; `L_HEEL`/`R_HEEL` are unconnected.

## Conventions

- Comments explain **why**, and often name the bug they prevent. Keep them.
- Tests state their verdict first: cases are hand-built with the expected answer
  written down before the maths runs.
- Protocol changes bump `NEXUS_PROTO_VERSION` and must land in `link_proto.h`,
  `pi/nexus_proto.py` (**canonical**), the copy in `zeus_26`, and
  `NexusState.msg` together. `check_proto.py` enforces it.
- **The user keeps uncommitted bench edits in `Appli/App/test_leg_can.c`.**
  Stage only your own changes.
