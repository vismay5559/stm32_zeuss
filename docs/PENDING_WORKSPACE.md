# Pending changes for the Pi workspace

Facts the firmware has established that **`zeus_26_mukesh_deploy`** has not
picked up yet. Nothing here breaks the link — the wire protocol is v10 on both
sides and `tools/check_proto.py` passes — and nothing here is urgent. Every
item is a comment or a document, and every one of them is what somebody reads
before touching the robot.

Delete each line as it is done, and this file when it is empty.

> The old Pi workspace `vismay5559/zeus_26` is **retired**. It is not the
> deployment target any more and should not be updated. This file used to be
> called `PENDING_ZEUS_26.md` and was about that repo.

---

## 1. The spring encoders are not daisy chained any more

They were two chains of two, one per leg. They are now four sensors sharing
SPI1 with **one chip select each** — PF1 left hip, PD15 left knee, PD14 right
hip, PF5 right knee. Two places still describe the old wiring.

**`AGENTS.md`**, line 36, in "The hardware, concretely":

```diff
 - **Series-elastic** hip pitch and knee pitch: the drive reports the motor side,
   an **AS5047P** encoder reports the spring's deflection, and the real joint
-  angle is their sum. Two SPI daisy chains, one per leg, one chip-select each.
+  angle is their sum. Four encoders share SPI1 with **one chip select each**
+  (PF1 / PD15 / PD14 / PF5) — see stm32_zeuss/docs/PINOUT.md.
```

**`zeus_link/zeus_link/nexus_proto.py`**, line 60, above `ENC_L_HIP_PITCH`:

```diff
 # Spring encoder order in spring_angle[] and the `enc_valid` bitmask, as
-# NEXUS_ENC_* in link_proto.h: two AS5047P daisy chains, one per leg.
+# NEXUS_ENC_* in link_proto.h: four AS5047P on one SPI bus, a chip select each.
```

Why it matters: the chain's word ORDER carried meaning, so the old comment
tells a reader that swapping two wires silently swaps two springs in the data.
With a select each it cannot happen, and someone chasing a spring that reads
wrong should not be sent looking for it.

## 2. Three things missing from `AGENTS.md`'s hardware section

```diff
+- **BNO085 IMU**, on the torso, SHTP over **USART2** (PD5/PD6) at 3 Mbaud. It
+  moved off USART1: PA9 is not a free pin on this board, it carries I2C_SDA to
+  the TCPP03-M20 USB-C controller through solder bridge SB35, closed by
+  default. PA8 is the matching SCL through SB31.
+- **Gear ratios**: 47:1 hip pitch and knee, 26:1 hip roll, **15:1 both
+  ankles**. The ankles' encoders are on the MOTOR side of the gearbox, so
+  their position, velocity and torque are all in motor units.
+- **Every pin into the STM32** — CAN, encoders, IMU, foot switches, supply
+  rails and the wiring traps — is in `stm32_zeuss/docs/PINOUT.md`. That is the
+  soldering reference.
```

## 3. The ankle gear ratio in this workspace is wrong

`zeus_bringup/config/gains.yaml` says "the 9:1 ankle's much smaller velocity
gain". **The ankles are 15:1**, which is what the policy assumed; 9:1 came from
the first bench leg. Anything derived from 9 is 15/9 = 1.67x out.

The same file's gain values are the bench leg's and are not the trained ones.
See the derivation above `g_drive_gains[]` in `stm32_zeuss/robot_config.c`:
`pos_gain` is `KP/KD` whichever shaft the encoder is on, but `vel_gain` carries
motor Nm against encoder turns/s, so it is `KD * 2pi / N` — and `KD * 2pi / N^2`
for the ankles, whose encoders sit before the gearbox. The handoff's 75.4 /
25.1 / 7.54 become **1.6042 / 0.9666 / 0.0335**.

## 4. Commands now go out at 500 Hz, not 1 kHz

`ODRV_CMD_DIVIDER` in `act_odrive.h`. This changes nothing the Pi sends — the
command stream is still 250 Hz and the policy still runs at 50 Hz — but any
document here that describes the board addressing the drives at 1 kHz is now
wrong. It is 500 Hz because that is the rate `zeus_env.py` trained at
(`DT_PHYS` 0.002), and it takes a five-drive bus from 73% to 56%.

---

## Not changes — things to confirm once the robot moves

- **`zeus_control_interface/README.md`** is the policy contract and its
  observation table is pinned to the protocol by
  `zeus_link/test/test_policy_readme.py`. If the observation changes, that test
  is what says the README no longer matches.
- **The simulator still needs numbers the URDF does not carry**: spring damping,
  rotor inertia (`armature`), joint friction, and real torque/velocity limits in
  place of the 25 Nm / 20 rad/s placeholders. Spring stiffness it now has —
  3437.75 Nm/rad, travel ±1.7°, which the firmware's
  `ROBOT_SPRING_MAX_DEFLECTION_RAD` has been set to match.
- **The firmware clips the residual at ±0.1 turn (±0.628 rad)**
  (`SAFETY_MAX_RESIDUAL_TURNS` in `safety.h`) and faults beyond it. The
  simulator must clip identically, or the policy learns actions the robot
  refuses. `ACT_SCALE` is well inside this, so it should never bind.
- **The gait table is not perfectly periodic.** The knee steps 0.475° across
  the stride wrap. It is deliberately not smoothed — the policy trained on that
  exact table and `rl_policy_node` rejects a `ref_angle` more than 1° from its
  own copy — but once the torque feedforward is in, that is a once-per-stride
  transient worth knowing about.
