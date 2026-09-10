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

### Hip pitch and knee — load-side encoder, done

Both read load-side absolute encoders. Output travel is ±30°, far inside **one
turn of the output shaft**, so one encoder reading maps to exactly one joint
angle and there is nothing to disambiguate.

**Set once, saved in the drive, survives power cycles.** This is configured and
working — the firmware does not touch it, and must not.

#### How ODrive actually holds the reference

Worth understanding, because the mechanism is not obvious and the earlier
version of this document got it wrong.

Every position command is interpreted against `<axis>.pos_estimate`. From the
[control docs](https://docs.odriverobotics.com/v/latest/manual/control.html#position-reference-frame):

> *"The ODrive interprets all position commands with respect to
> `<axis>.pos_estimate`. That means when the user commands a position setpoint
> of 0.123, the ODrive tries to move the axis such that `<axis>.pos_estimate`
> becomes 0.123."*

`pos_estimate` is not the raw encoder angle. It is the encoder angle plus a
stored offset:

| parameter | what it does |
|---|---|
| `pos_vel_mapper.config.offset` | shifts where axis zero sits relative to the encoder's own zero |
| `pos_vel_mapper.config.offset_valid` | enables it |
| `pos_vel_mapper.config.approx_init_pos` | for multi-turn axes: the range the axis is guaranteed to start in |
| `pos_vel_mapper.config.approx_init_pos_valid` | enables that guarantee |

`set_abs_pos(x)` computes the offset that makes the current encoder reading
report as `x`, and writes it there. **`save_configuration()` persists it**, and
after that the docs are explicit: the position estimate *"will be immediately
available without needing a homing procedure."*

So the drive does the work at power-up. It reads its absolute encoder, adds the
saved offset, and `pos_estimate` is already correct before the STM32 has said
anything.

#### What that means for the firmware

**`test_leg_can.c` must not send `Set_Absolute_Position` to these two.** That
command redefines zero as wherever the leg happens to be standing, which would
silently overwrite a calibrated reference with an arbitrary one. `s_define_zero`
is `{ 0, 0, 1 }` — the ankle only — for exactly this reason.

Instead the firmware **drives to** zero: the run opens with a `RUN_GOTO_ZERO`
phase that ramps both joints to `pos_estimate = 0` before anything else happens.
Declaring zero and driving to zero are opposite operations, and only one of them
is right once the drive holds a real reference frame.

#### Setting it, once

```python
# leg held at the pose you are calling zero
odrv0.axis0.set_abs_pos(0)
odrv0.save_configuration()
```

Then verify, and do not skip this:

```
1. read pos_estimate               -> ~0
2. power everything down completely
3. power back up, leg untouched
4. read pos_estimate again         -> must still be ~0
```

If step 4 does not match step 1 the offset did not persist, and you have a
per-power-up procedure rather than an absolute zero.

### Ankle — motor-side encoder, 9:1, cannot persist

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

**What the firmware does today is (3), made explicit.** `s_define_zero` is
`{ 0, 0, 1 }`, so `legtest_init()` sends `Set_Absolute_Position(0.0)` to the
ankle and to nothing else:

```
absolute joint reference setup
  Hip and knee keep the reference frame saved in their drives:
  pos_vel_mapper offset, persisted by save_configuration.
  Only the ANKLE is declared here - put it at mechanical 0 deg
  before powering the STM32, because a motor-side encoder on a
  9:1 cannot hold a zero across a power cycle.
  ankle     CAN 0x19: Set_Absolute_Position(0.0 turns)
  ankle     reference: pos_estimate=+0.000000 turns  [OK]
```

**This declares zero, it does not find it.** Whatever pose the ankle is in when
the board powers up becomes its origin, and the readback only proves the drive
accepted the command - not that the joint was anywhere near where you meant.
Place it by hand, every boot, until it gets a load-side encoder.

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
