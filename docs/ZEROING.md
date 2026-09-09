# Joint zero — what it is, how to set it, and whether it survives a power cycle

The gait is **absolute**. Sample 0 of hip pitch is −18.57°, not "wherever the
leg happens to be". That number only means anything if the drive and the robot
agree on where 0° is, so this has to be established deliberately and verified.

---

## Two different things are called "calibration"

They are independent and confusing them wastes a day.

| | what it finds | when it is needed |
|---|---|---|
| **Motor / encoder calibration** | the electrical offset between rotor magnets and encoder count, so commutation works | once per motor+encoder pairing; persists if `pre_calibrated` is set |
| **Position zero** | where `pos_estimate == 0` sits on the *robot* | once per mechanical assembly — **this document** |

A drive can be perfectly commutated and still have no idea that its zero is 30°
away from the robot's zero. That is the failure this is about.

---

## Does the zero survive a power cycle?

Entirely determined by the encoder, and **it is not the same answer for every
joint on this leg**.

### Hip pitch and knee — encoder on the load side, 47:1

> **Current as of this writing.** Hip and knee read load-side encoders; the
> ankle reads the motor side of its 9:1, so the section below applies to it.
> `s_cmd_scale[] = { 1, 1, 9 }` in `test_leg_can.c` follows that split — the
> drive's position units follow its encoder, not its gearbox.

Output travel is ±25° and ±35°, both far inside **one turn of the output
shaft**. If the load-side encoder is absolute, one encoder reading maps to
exactly one joint angle, with no ambiguity.

**Set the zero once. It persists.** That is the answer you were hoping for.

### Ankle — encoder on the MOTOR side, 9:1

This one does not work the same way, and it is worth being precise about why.

```
joint range          ±35°  =  70° of output
motor turns for that  70° × 9  =  630°  =  1.75 turns
```

A **single-turn** absolute encoder repeats every 360° of motor shaft, which is
40° of output. So a single reading is consistent with several different joint
angles, 40° apart, and 1.75 turns of travel means the joint genuinely visits
more than one of them.

**A single-turn absolute encoder on the motor side cannot determine this
joint's output angle on its own.** One of these has to be true:

1. the encoder is **multi-turn** and counts revolutions across power cycles, or
2. the drive is **homed** at every power-up — driven to a hard stop or an index
   mark whose output angle is known, or
3. the joint is never moved while powered down, and the turn count is trusted
   from the last session. Fragile: one hand-push with the power off and the
   robot silently believes something false.

Find out which before relying on the ankle's zero. (1) and (2) are operating
procedures; (3) is a bug waiting for a bad day.

---

## Setting the zero

### 0. Agree what the zero pose *is*

The gait came out of a Pinocchio/CasADi model. Its `q = 0` is whatever pose that
model calls zero, and the leg has to be put in **that** pose — not "looks
straight to me". Check it against the URDF the trajectory was generated from.

This matters more than it sounds: hip pitch has **4.47°** of margin to its
mechanical stop and knee has **4.77°**. A zero that is 5° out is not a tracking
error, it is contact with the stop on the first cycle.

### 1. Put the leg in that pose

By jig or fixture. Hip and knee are non-backdrivable, so this means either
energising them and commanding the pose, or assembling to the pose. It cannot be
done by pushing the leg around.

### 2. Zero each axis

In odrivetool, per axis, with the leg held in the zero pose:

```python
odrv0.axis0.set_abs_pos(0)          # define here as zero
odrv0.save_configuration()
```

The exact call varies with ODrive firmware version — check what your S1 build
exposes. Over CAN the equivalent is `Set_Absolute_Position` (`0x019`).

**The leg firmware now sends this itself**, once, from `legtest_init()` while
`LEGTEST_ZERO_ABSOLUTE_AT_BOOT` is 1, for the joints listed in
`s_define_zero[]`. It is still a setup operation and the control loop still
never issues it: it happens before the timers start, and the result is not
saved to the drive, so it is re-established every boot rather than persisting.

That changes what this document is for. The procedure below makes a zero that
SURVIVES a power cycle, which is what the robot needs. The firmware's
boot-time version is the bench equivalent - it declares the current pose to be
zero, so **the leg has to be in the zero pose before the STM32 is powered**,
every single time. It does not find zero, and it cannot tell you the pose was
wrong: it will happily call a 20-degree error the origin.

### 3. Verify it persisted — do not skip this

```
1. read pos_estimate on every axis          -> should be ~0
2. power everything down completely
3. power back up, leg still in the zero pose
4. read pos_estimate again                  -> must still be ~0
```

If step 4 does not match step 1, **the zero is not persistent** and this becomes
a per-power-up procedure. That is a different operating model, not a smaller
one, and the leg must not run a trajectory until it is resolved.

This is Test 5.3 in `CAN_BUS_TEST.md`.

### 4. Check the range actually fits

With the zero set, the gait extremes are known absolutely:

| joint | gait range | mechanical limit | clearance |
|---|---|---|---|
| hip pitch | −20.53° .. −6.71° | ±25° | 4.47° |
| knee pitch | +25.53° .. +30.23° | ±35° | 4.77° |
| ankle pitch | +6.03° .. +21.41° | ±35° | 13.59° |

Command each joint slowly to both extremes and confirm it arrives without
touching a stop, with ≥ 3° to spare. If the measured zero eats the margin, the
gait cannot run as-is — use `s_zero_offset[]` in `test_leg_can.c` to re-centre
it inside the available travel, or reduce the amplitude.

---

## What `s_zero_offset[]` is for

It is a per-joint trim, in **output-shaft turns**, added to the gait value before
the gear scaling:

```c
static float joint_cmd(int j, float out_turns)
{
    return (out_turns + s_zero_offset[j]) * s_cmd_scale[j];
}
```

It exists to absorb a disagreement between the drive's zero and the gait's zero
that you have measured and decided to live with. It is **not** a substitute for
zeroing the drives properly — a trim carried in firmware is invisible to
odrivetool, to anyone reading `pos_estimate`, and to whoever debugs this next.

The firmware prints the value it would take, on every run, from the pose it
measures at arming:

```
  hip_pitch: at -0.0511, gait starts at -0.0516 -> ramp -0.0005 turns (-0.2 deg)
     to play the gait around where the leg is now, set s_zero_offset[0] = +0.0005
```
