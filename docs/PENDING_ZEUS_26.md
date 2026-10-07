# Pending changes for the zeus_26 workspace

Changes the firmware has made that the Pi workspace has **not** picked up yet.
`zeus_26` is deliberately being left alone until the software lead has added the
RL policy; this file is the list so nothing is lost in the meantime.

Nothing here is urgent and nothing here breaks the link: every item is a comment
or a document. The wire protocol is **unchanged at v9** — the encoder rewiring
changed no packet field, no size and no version, so a board flashed with the new
firmware talks to today's workspace without complaint.

Delete each line as it is done, and this file when it is empty.

---

## 1. Two stale comments about the encoder wiring

The spring encoders were two daisy chains of two, one per leg. They are now four
sensors on one shared SPI bus with **one chip select each** (firmware commit
"Spring encoders: a chip select each, not two daisy chains"). Two comments in
`zeus_26` still describe the old arrangement.

**`AGENTS.md`**, in "The hardware, concretely":

```diff
 - **Series-elastic** hip pitch and knee pitch: the drive reports the motor side,
   an **AS5047P** encoder reports the spring's deflection, and the real joint
-  angle is their sum. Two SPI daisy chains, one per leg, one chip-select each.
+  angle is their sum. Four encoders share SPI1 with **one chip select each**
+  (PF1 / PD15 / PD14 / PF5) — see the wiring table in the firmware README.
```

**`zeus_link/zeus_link/nexus_proto.py`**, above `ENC_L_HIP_PITCH`:

```diff
 # Spring encoder order in spring_angle[] and the `enc_valid` bitmask, as
-# NEXUS_ENC_* in link_proto.h: two AS5047P daisy chains, one per leg.
+# NEXUS_ENC_* in link_proto.h: four AS5047P on one SPI bus, a chip select each.
```

Note that `nexus_proto.py` is a **copy**. The canonical file is
`stm32_zeuss/pi/nexus_proto.py`, which already has this change; the workspace
copy is that file with a six-line banner on top. Copy it over rather than
editing by hand, or the two drift.

## 2. Nothing else

The protocol, the joint map, the message definitions and the URDF are all
current. `check_proto.py` and `test_msg_matches_proto.py` pass against the
workspace as it stands today.

---

## When the policy lands, check these too

Not firmware changes — things worth confirming once there is a policy to run.

- **`zeus_control_interface/README.md`** is the policy contract and its
  observation table is pinned to the protocol by
  `zeus_link/test/test_policy_readme.py`. If the lead changes the observation,
  that test is what tells him the README no longer matches.
- **The simulator still needs numbers the URDF does not carry**: spring
  stiffness and damping for the four spring joints, rotor inertia (`armature`),
  joint friction, and real torque/velocity limits in place of the 25 Nm /
  20 rad/s placeholders. A policy trained against the URDF as exported is
  trained against limp springs and a frictionless robot.
- **The firmware clips the policy's residual at ±0.1 turn (±0.628 rad)**
  (`SAFETY_MAX_RESIDUAL_TURNS` in `safety.h`) and faults beyond it. The
  simulator must clip identically, or the policy learns actions the robot
  refuses.
