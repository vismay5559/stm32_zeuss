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
#define NEXUS_PROTO_VERSION     5u      /* v5: diagnostics block               */

#define NEXUS_MSG_STATE         0x01u
#define NEXUS_MSG_COMMAND       0x02u

#define NEXUS_NUM_JOINTS        10      /* 5 per leg, ODrive order            */
#define NEXUS_NUM_ENCODERS      4       /* AS5048A, after-spring (SEA) joints */
#define NEXUS_NUM_CONTACTS      4       /* mechanical foot switches           */

/*
 * Foot switch order, used by contact[] in the policy block and by the
 * `contacts` bitmask. Two switches per foot: toe and heel.
 */
#define NEXUS_CONTACT_L_TOE     0
#define NEXUS_CONTACT_L_HEEL    1
#define NEXUS_CONTACT_R_TOE     2
#define NEXUS_CONTACT_R_HEEL    3

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
 * ref_angle and phase are reserved space that the STM32 fills with zeros
 * because the gait library does not run here yet. The old comment said the Pi
 * could detect this "because phase never advances", which is
 * indistinguishable from a gait legitimately parked at phase 0. A bit that
 * says so is not a guess.
 */
#define NEXUS_STREAM_GAIT_LIVE  (1u << 0)   /* ref_angle and phase are real */

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
     * 52 contiguous float32. Slice this straight into the observation.
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
    float joint_vel[NEXUS_NUM_JOINTS];       /*  96 rad/s, OUTPUT side          */
    float spring_angle[NEXUS_NUM_ENCODERS];  /* 136 rad, SPRING DEFLECTION -
                                                    what the after-spring
                                                    encoders actually measure,
                                                    not an absolute joint angle */
    float ref_angle[NEXUS_NUM_JOINTS];       /* 152 rad, reference from the gait
                                                    library. Zero until the
                                                    library runs on the STM32.  */
    float contact[NEXUS_NUM_CONTACTS];       /* 192 0.0 / 1.0, debounced, in
                                                    NEXUS_CONTACT_* order       */
    float foot_z[2];                         /* 208 m, world. [0] right,
                                                    [1] left. Forward kinematics
                                                    through the fused pose.     */
    float phase;                             /* 216 0..1 gait clock. 0 = start
                                                    of stance, 1 = end of the
                                                    full leg trajectory.        */
    /* ==================== end policy block ====================== 220 */

    /* ---- IMU, raw from the BNO085 ----------------------------- 220 */
    float    imu_quat[4];                    /* 220 w,x,y,z, sensor's own 9-axis
                                                    fusion. Independent of the
                                                    estimator's quat[] above.   */
    float    imu_accel[3];                   /* 236 m/s^2, specific force,
                                                    INCLUDES gravity            */
    float    imu_gyro[3];                    /* 248 rad/s, raw                  */
    uint32_t imu_seq;                        /* 260 lets the Pi spot staleness  */

    /* ---- actuator diagnostics --------------------------------- 264 */
    float    act_torque[NEXUS_NUM_JOINTS];   /* 264 Nm, estimate                */
    uint32_t act_error[NEXUS_NUM_JOINTS];    /* 304 raw ODrive axis_error       */

    /* ---- estimator internals ---------------------------------- 344 */
    float    fused_pos[3];                   /* 344 m, world. [2] duplicates
                                                    pelvis_z; [0] and [1] drift
                                                    and are for logging only.   */
    float    fused_vel[3];                   /* 356 m/s, WORLD frame, before the
                                                    heading rotation            */
    float    fused_gyro_bias[3];             /* 368 rad/s, estimated            */
    float    fused_accel_bias[3];            /* 380 m/s^2, estimated            */

    /* ---- diagnostics ------------------------------------------ 392 *
     *
     * These used to go out only on the serial console, in a line that cost
     * ~9.5 ms of a 1 ms control loop every two seconds - a diagnostic that
     * caused nine of the missed ticks it was reporting. The Pi is already
     * reading this packet at 1 kHz, so they belong here, where they can be
     * plotted against everything else that happened at the same moment.
     */
    uint32_t overruns;                       /* 392 ticks missed, cumulative    */
    uint32_t usb_dropped;                    /* 396 state packets skipped       */
    uint16_t can_dropped[2];                 /* 400 TX frames dropped, per bus  */
    uint16_t loop_us_max;                    /* 404 worst cycle since last sent */
    uint16_t enc_stalls;                     /* 406 SPI transfers abandoned     */
    uint8_t  can_bus_off[2];                 /* 408 bus-off events, saturating  */
    uint8_t  stream_flags;                   /* 410 NEXUS_STREAM_*              */
    uint8_t  reserved0;                      /* 411 keeps the next field even   */

    /* ---- 2-byte fields ---------------------------------------- 412 */
    uint16_t contact_ticks[2];               /* 412 ticks each foot held state  */

    /* ---- 1-byte fields ---------------------------------------- 396 */
    uint8_t  act_state[NEXUS_NUM_JOINTS];    /* 416 raw ODrive axis_state       */
    uint8_t  act_flags[NEXUS_NUM_JOINTS];    /* 426 per-joint freshness         */
    uint8_t  enc_valid;                      /* 436 bit per encoder             */
    uint8_t  contacts;                       /* 437 switch + derived foot bits  */
    uint8_t  fused_valid;                    /* 438 NEXUS_FUSION_*              */
    uint8_t  health;                         /* 439 health.h bitmask            */

    /*
     * Which foot_z entries are real measurements: bit 0 = foot_z[0] (right),
     * bit 1 = foot_z[1] (left). An invalid entry is sent as NaN as well, so a
     * reader can use either signal - but never treat a foot_z as a height
     * without checking one of them. It used to be sent as 0.0, which reads as
     * "exactly on the ground".
     */
    uint8_t  fk_valid;                       /* 440 bit per foot                */

    /* NEXUS_SAFETY_* - whether the board is allowed to be driving, and why
       not. Lets the Pi see a fault it caused, and see that a stand-down or a
       re-arm handshake was actually acted on. */
    uint8_t  safety_state;                   /* 441                             */

    uint16_t crc;                            /* 442 CRC16-CCITT over 0..441     */
} nexus_state_t;                             /* 444 total                       */

typedef struct __attribute__((packed))
{
    uint16_t sync;                           /*  0 */
    uint8_t  msg_id;                         /*  2 */
    uint8_t  version;                        /*  3 */
    uint32_t seq;                            /*  4 */
    float    target_pos[NEXUS_NUM_JOINTS];   /*  8 turns                        */
    uint16_t flags;                          /* 48 */
    uint16_t crc;                            /* 50 */
} nexus_cmd_t;                               /* 52 total                        */

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
