#ifndef LINK_PROTO_H
#define LINK_PROTO_H

/*
 * Wire format for the STM32 <-> Raspberry Pi USB CDC link.
 *
 * The STM32 owns all real-time sensing and state estimation; the Pi receives
 * the finished state and runs only the RL policy.
 *
 * THE PYTHON SIDE OF THIS FILE IS pi/nexus_proto.py IN THIS REPO. The two are
 * checked against each other by tools/check_proto.py, which compares every
 * field offset and the total size. Change one, change the other, run the check.
 *
 * Layout rule: every 4-byte field sits at a 4-byte-aligned offset. The struct
 * is packed, so without that rule the compiler emits byte-by-byte access for
 * every misaligned float on the M7, and numpy cannot view the buffer directly
 * on the Pi.
 *
 * ---------------------------------------------------------------------------
 * v3 introduces the POLICY BLOCK: one contiguous run of float32 holding exactly
 * what the RL observation needs, in the order the policy expects, so the Pi can
 * slice it in place rather than reassembling it field by field.
 *
 * Everything in that block is a RAW PHYSICAL QUANTITY IN SI UNITS. The STM32
 * applies no policy scaling - no target-height subtraction, no clipping, no
 * sin/cos, no normalisation. All of that belongs on the Pi, so the observation
 * transform can change without reflashing the robot.
 * ---------------------------------------------------------------------------
 */

#include <stdint.h>

#define NEXUS_SYNC              0xA5A5u
#define NEXUS_PROTO_VERSION     8u      /* v8: act_target, and gains from the Pi      */

#define NEXUS_MSG_STATE         0x01u
#define NEXUS_MSG_COMMAND       0x02u
#define NEXUS_MSG_GAINS         0x03u   /* Pi -> board, rare: drive gains     */

#define NEXUS_NUM_JOINTS        8       /* see the joint map below            */
#define NEXUS_NUM_ENCODERS      4       /* AS5047P, after-spring (SEA) joints */
#define NEXUS_NUM_CONTACTS      4       /* mechanical foot switches           */

/*
 * THE JOINT MAP - the one definition of which index is which joint.
 *
 * Every per-joint array in both packets uses it: joint_pos, joint_vel,
 * ref_angle, act_torque, act_error, act_state, act_flags, and the command's
 * residual. The packet carries no names, so a reader that labels these by any
 * other table is labelling them wrong.
 *
 *     index = bus * 4 + (node - 1)      bus 0 = FDCAN1, bus 1 = FDCAN2
 *
 *   idx  bus  node   joint
 *    0    0    1     left_hip_pitch
 *    1    0    2     left_hip_roll
 *    2    0    3     left_knee_pitch
 *    3    0    4     left_ankle_pitch
 *    4    1    1     right_hip_pitch
 *    5    1    2     right_hip_roll
 *    6    1    3     right_knee_pitch
 *    7    1    4     right_ankle_pitch
 *
 * ---------------------------------------------------------------------------
 * TWO LEGS, NO WAIST - a temporary build.
 *
 * The robot has ten actuators: these eight plus waist roll (bus 0 node 5) and
 * waist pitch (bus 1 node 5). This firmware is for bringing the two legs up
 * without them: node 5 on each bus is neither commanded nor expected, and the
 * waist is bolted at its zero pose. The estimator's kinematics still run the
 * waist joints - they are part of the chain from the IMU to each leg - and
 * simply hold them at zero (fusion.c).
 *
 * Putting the waist back: NEXUS_NUM_JOINTS 10, ODRV_NODES_PER_BUS 5, the map
 * above back to bus * 5, the two NEXUS_J_WAIST_* indices, the waist entries in
 * robot_config.c, and the version bump. The tag `waist-10-actuators` marks the
 * last commit that had them.
 * ---------------------------------------------------------------------------
 *
 * Confirmed against the wiring. The Pi side names them in the same order in
 * nexus_proto.py JOINT_NAMES; change one, change the other.
 */
#define NEXUS_J_L_HIP_PITCH     0
#define NEXUS_J_L_HIP_ROLL      1
#define NEXUS_J_L_KNEE_PITCH    2
#define NEXUS_J_L_ANKLE_PITCH   3
#define NEXUS_J_R_HIP_PITCH     4
#define NEXUS_J_R_HIP_ROLL      5
#define NEXUS_J_R_KNEE_PITCH    6
#define NEXUS_J_R_ANKLE_PITCH   7

/*
 * Foot switch order, used by contact[] in the policy block and by the
 * `contacts` bitmask. Two switches per foot: toe and heel.
 */
#define NEXUS_CONTACT_L_TOE     0
#define NEXUS_CONTACT_L_HEEL    1
#define NEXUS_CONTACT_R_TOE     2
#define NEXUS_CONTACT_R_HEEL    3

/*
 * Spring encoder order, used by spring_angle[] and the `enc_valid` bitmask
 * (bit i = index i). Two AS5047P daisy chains, one per leg; enc_as5047p.h
 * has the wiring.
 */
#define NEXUS_ENC_L_HIP_PITCH   0
#define NEXUS_ENC_L_KNEE_PITCH  1
#define NEXUS_ENC_R_HIP_PITCH   2
#define NEXUS_ENC_R_KNEE_PITCH  3

/* Bitmask positions in `contacts`, same order, plus the derived per-foot bits
   the STM32 computes by OR-ing each foot's two switches. */
#define NEXUS_CONTACT_L_TOE_BIT   (1u << 0)
#define NEXUS_CONTACT_L_HEEL_BIT  (1u << 1)
#define NEXUS_CONTACT_R_TOE_BIT   (1u << 2)
#define NEXUS_CONTACT_R_HEEL_BIT  (1u << 3)
#define NEXUS_CONTACT_L_FOOT      (1u << 4)
#define NEXUS_CONTACT_R_FOOT      (1u << 5)

/* fused_valid values. */
#define NEXUS_FUSION_INVALID    0u      /* estimator not running              */
#define NEXUS_FUSION_CONVERGING 1u      /* running, covariance still large    */
#define NEXUS_FUSION_OK         2u      /* converged, safe to use             */

/* act_flags bits, per joint. */
#define NEXUS_ACT_TELEM_FRESH   (1u << 0)
#define NEXUS_ACT_HB_FRESH      (1u << 1)

/* cmd flags bits. */
#define NEXUS_CMD_ENABLE        (1u << 0)

/* safety_state values - see safety.h. */
#define NEXUS_SAFETY_BOOT       0u
#define NEXUS_SAFETY_IDLE       1u
#define NEXUS_SAFETY_ARMED      2u
#define NEXUS_SAFETY_FAULT      3u

/* fk_valid bits, matching foot_z's order. */
#define NEXUS_FK_RIGHT_VALID    (1u << 0)
#define NEXUS_FK_LEFT_VALID     (1u << 1)

/*
 * stream_flags - which optional parts of the packet are actually being
 * produced.
 *
 * GAIT_LIVE: ref_angle and phase carry the reference the STM32 is playing.
 * Robot mode sets it on every packet. It exists so a reader never has to infer
 * a live gait from phase moving, which cannot tell a gait parked at phase 0
 * from one that is not running.
 */
#define NEXUS_STREAM_GAIT_LIVE  (1u << 0)   /* ref_angle and phase are real */

/*
 * LEG_TEST: this packet comes from the single-leg bench test (NEXUS_MODE_LEG_CAN,
 * leg_stream.c), not the robot loop. Only the joints that test drives are real -
 * joint_pos, joint_vel, ref_angle (the target it sent) and act_* at their
 * joint-map indices. The estimator, IMU, spring encoders and foot switches are
 * not running: quaternions read identity, foot_z NaN, fused_valid INVALID.
 * Commands sent back are ignored.
 */
#define NEXUS_STREAM_LEG_TEST   (1u << 1)

typedef struct __attribute__((packed))
{
    /* ---- header ------------------------------------------------ 0 */
    uint16_t sync;                           /*   0 */
    uint8_t  msg_id;                         /*   2 */
    uint8_t  version;                        /*   3 */
    uint32_t seq;                            /*   4 increments every 1 kHz tick */
    uint32_t timestamp_us;                   /*   8 free-running 1 MHz counter  */

    /* ====================== POLICY BLOCK ======================== 12
     *
     * 46 contiguous float32. Slice this straight into the observation.
     * SI units, raw. See pi/nexus_proto.py: NexusState.policy_block().
     */
    float pelvis_z;                          /*  12 m, height above stance ground
                                                    from the estimator          */
    float quat[4];                           /*  16 w,x,y,z body->world, fused.
                                                    Observation uses x and y,
                                                    i.e. quat[1] and quat[2].   */
    float gyro[3];                           /*  32 rad/s, BODY frame           */
    float vel_hdg[3];                        /*  44 m/s, HEADING frame:
                                                    [0] lateral
                                                    [1] forward
                                                    [2] vertical                */
    float joint_pos[NEXUS_NUM_JOINTS];       /*  56 rad, OUTPUT side            */
    float joint_vel[NEXUS_NUM_JOINTS];       /*  88 rad/s, OUTPUT side          */
    float spring_angle[NEXUS_NUM_ENCODERS];  /* 120 rad, SPRING DEFLECTION -
                                                    what the after-spring
                                                    encoders actually measure,
                                                    not an absolute joint angle */
    float ref_angle[NEXUS_NUM_JOINTS];       /* 136 rad, OUTPUT side: the stored
                                                    gait at `phase`, the value
                                                    residual[] is added to.     */
    float contact[NEXUS_NUM_CONTACTS];       /* 168 0.0 / 1.0, debounced, in
                                                    NEXUS_CONTACT_* order       */
    float foot_z[2];                         /* 184 m, world. [0] right,
                                                    [1] left. Forward kinematics
                                                    through the fused pose.     */
    float phase;                             /* 192 0..1 gait clock. 0 = start
                                                    of stance, 1 = end of the
                                                    full leg trajectory.        */
    /* ==================== end policy block ====================== 196 */

    /* ---- IMU, raw from the BNO085 ----------------------------- 196 */
    float    imu_quat[4];                    /* 196 w,x,y,z, sensor's own 9-axis
                                                    fusion. Independent of the
                                                    estimator's quat[] above.   */
    float    imu_accel[3];                   /* 212 m/s^2, specific force,
                                                    INCLUDES gravity            */
    float    imu_gyro[3];                    /* 224 rad/s, raw                  */
    uint32_t imu_seq;                        /* 236 lets the Pi spot staleness  */

    /* ---- actuator diagnostics --------------------------------- 240 */
    float    act_torque[NEXUS_NUM_JOINTS];   /* 240 Nm, estimate                */
    uint32_t act_error[NEXUS_NUM_JOINTS];    /* 272 raw ODrive axis_error       */

    /*
     * WHAT THE DRIVE WAS TOLD, this tick, rad on the output side - the
     * interpolated target act_odrive actually put on the wire, after the
     * reference, the policy's residual, the safety envelope and the slew
     * limit have all had their say.
     *
     * ref_angle + residual is what the Pi ASKED for; this is what the joint
     * was ACTUALLY commanded, and it is the only honest thing to plot a
     * measured joint_pos against when tuning a drive's gains. NaN while
     * nothing is being driven, so a plot shows a gap rather than a flat line
     * that looks like a held command.
     */
    float    act_target[NEXUS_NUM_JOINTS];   /* 304 rad, OUTPUT side, or NaN    */

    /* ---- estimator internals ---------------------------------- 336 */
    float    fused_pos[3];                   /* 336 m, world. [2] duplicates
                                                    pelvis_z; [0] and [1] drift
                                                    and are for logging only.   */
    float    fused_vel[3];                   /* 348 m/s, WORLD frame, before the
                                                    heading rotation            */
    float    fused_gyro_bias[3];             /* 360 rad/s, estimated            */
    float    fused_accel_bias[3];            /* 372 m/s^2, estimated            */

    /* ---- diagnostics ------------------------------------------ 384 *
     *
     * These used to go out only on the serial console, in a line that cost
     * ~9.5 ms of a 1 ms control loop every two seconds - a diagnostic that
     * caused nine of the missed ticks it was reporting. The Pi is already
     * reading this packet at 1 kHz, so they belong here, where they can be
     * plotted against everything else that happened at the same moment.
     */
    uint32_t overruns;                       /* 384 ticks missed, cumulative    */
    uint32_t usb_dropped;                    /* 388 state packets skipped       */
    uint16_t can_dropped[2];                 /* 392 TX frames dropped, per bus  */
    uint16_t loop_us_max;                    /* 396 worst cycle since last sent */
    uint16_t enc_stalls;                     /* 398 SPI transfers abandoned     */
    uint8_t  can_bus_off[2];                 /* 400 bus-off events, saturating  */
    uint8_t  stream_flags;                   /* 402 NEXUS_STREAM_*              */
    uint8_t  reserved0;                      /* 403 keeps the next field even   */

    /* ---- 2-byte fields ---------------------------------------- 404 */
    uint16_t contact_ticks[2];               /* 404 ticks each foot held state  */

    /* ---- 1-byte fields ---------------------------------------- 408 */
    uint8_t  act_state[NEXUS_NUM_JOINTS];    /* 408 raw ODrive axis_state       */
    uint8_t  act_flags[NEXUS_NUM_JOINTS];    /* 416 per-joint freshness         */
    uint8_t  enc_valid;                      /* 424 bit per encoder             */
    uint8_t  contacts;                       /* 425 switch + derived foot bits  */
    uint8_t  fused_valid;                    /* 426 NEXUS_FUSION_*              */
    uint8_t  health;                         /* 427 health.h bitmask            */

    /*
     * Which foot_z entries are real measurements: bit 0 = foot_z[0] (right),
     * bit 1 = foot_z[1] (left). An invalid entry is sent as NaN as well, so a
     * reader can use either signal - but never treat a foot_z as a height
     * without checking one of them. It used to be sent as 0.0, which reads as
     * "exactly on the ground".
     */
    uint8_t  fk_valid;                       /* 428 bit per foot                */

    /* NEXUS_SAFETY_* - whether the board is allowed to be driving, and why
       not. Lets the Pi see a fault it caused, and see that a stand-down or a
       re-arm handshake was actually acted on. */
    uint8_t  safety_state;                   /* 429                             */

    /*
     * The seq of the last gains message this board APPLIED (low byte). The Pi
     * sends gains, watches this change, and knows they landed. It does not
     * change when a gains message is refused - which is what happens if the
     * robot is armed, since retuning a drive mid-stride is not something to
     * allow by accident.
     */
    uint8_t  gains_seq;                      /* 430                             */
    uint8_t  reserved1;                      /* 431 keeps the crc even          */

    uint16_t crc;                            /* 432 CRC16-CCITT over 0..431     */
} nexus_state_t;                             /* 434 total                       */

typedef struct __attribute__((packed))
{
    uint16_t sync;                           /*  0 */
    uint8_t  msg_id;                         /*  2 */
    uint8_t  version;                        /*  3 */
    uint32_t seq;                            /*  4 */
    /*
     * A RESIDUAL, not a target. Output-shaft turns, added to the reference
     * the STM32 is already playing:
     *
     *     drive_target[j] = ref_angle[j] + residual[j]
     *
     * Send zeros and the robot walks the stored gait unaided. The policy's job
     * is only the correction on top.
     *
     * The drives only move while commands keep arriving. A Pi that stops
     * sending does NOT fall back to the stored gait: the last accepted target
     * is held, and after 200 ms without a valid command the link fault takes
     * the actuators away (safety.c) until the Pi sends ENABLE off, then on.
     *
     * The reference this is added to comes back in the state packet as
     * ref_angle[] (radians) with phase, so the policy can see exactly what it
     * is correcting and when in the stride it is.
     *
     * Bounded by SAFETY_MAX_RESIDUAL_TURNS on its own account, and the sum is
     * bounded again by the joint envelope and the slew limit. A residual is a
     * nudge; anything large enough to be a trajectory of its own is refused.
     *
     * This field was target_pos[] in v5, same offset and size but an absolute
     * command. The version bump is the guard: link_usb.c rejects any frame
     * whose version is not NEXUS_PROTO_VERSION, so a v5 Pi cannot have its
     * absolute angles silently added to the gait.
     */
    float    residual[NEXUS_NUM_JOINTS];     /*  8 turns, added to ref_angle    */
    uint16_t flags;                          /* 40 */
    uint16_t crc;                            /* 42 */
} nexus_cmd_t;                               /* 44 total                        */

/*
 * DRIVE GAINS, Pi -> board. Sent rarely, by hand or by a tuning script; the
 * board applies them to every drive over CAN and re-applies them on every arm,
 * so a drive that reboots comes back with the gains that were being tuned
 * rather than whatever is in its own memory.
 *
 * Refused unless the robot is disarmed - see gains_seq above. The defaults
 * live in robot_config.h, so a board with no Pi attached still arms with known
 * gains.
 *
 * The ODrive's own loops: pos_gain is (turn/s)/turn, vel_gain is Nm/(turn/s),
 * vel_integrator_gain is Nm/turn. A negative value means "leave this drive's
 * saved value alone", the same convention the bench leg test uses.
 */
typedef struct __attribute__((packed))
{
    uint16_t sync;                           /*   0 */
    uint8_t  msg_id;                         /*   2 NEXUS_MSG_GAINS */
    uint8_t  version;                        /*   3 */
    uint32_t seq;                            /*   4 echoed in state as gains_seq */
    float    pos_gain[NEXUS_NUM_JOINTS];     /*   8 */
    float    vel_gain[NEXUS_NUM_JOINTS];     /*  40 */
    float    vel_int_gain[NEXUS_NUM_JOINTS]; /*  72 */
    uint16_t crc;                            /* 104 CRC16-CCITT over 0..103     */
} nexus_gains_t;                             /* 106 total                       */

/* The longest message the board can receive, for the reader's buffer. */
#define NEXUS_RX_MAX  (sizeof(nexus_gains_t) > sizeof(nexus_cmd_t) \
                       ? sizeof(nexus_gains_t) : sizeof(nexus_cmd_t))

/*
 * Work out the check number for a message.
 *
 * This condenses a whole message down to one short number. The sender
 * calculates it and sends it along; the receiver calculates it again from
 * what arrived. If the two disagree, something was damaged on the way and the
 * message is thrown away.
 *
 * Both the board and the Pi run the identical calculation, which is what lets
 * either side spot a corrupted message. It says nothing about whether the
 * contents were sensible - only that they arrived as they were sent.
 */
uint16_t nexus_crc16(const uint8_t *data, uint32_t len);

#endif /* LINK_PROTO_H */
