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
 * on the Pi. Fields are therefore grouped by size - 4-byte first, then 2, then
 * 1 - rather than by topic.
 */

#include <stdint.h>

#define NEXUS_SYNC              0xA5A5u
#define NEXUS_PROTO_VERSION     2u      /* v2: enc in radians, health, aligned */

#define NEXUS_MSG_STATE         0x01u
#define NEXUS_MSG_COMMAND       0x02u

#define NEXUS_NUM_JOINTS        10      /* 5 per leg, ODrive order            */
#define NEXUS_NUM_ENCODERS      4       /* AS5048A, after-spring (SEA) joints */

/* Contact bitmask positions, matching zeus_26 switch naming. */
#define NEXUS_CONTACT_L_TOE     (1u << 0)
#define NEXUS_CONTACT_L_HEEL    (1u << 1)
#define NEXUS_CONTACT_R_TOE     (1u << 2)
#define NEXUS_CONTACT_R_HEEL    (1u << 3)
/* Derived per-foot contact, computed on the STM32. */
#define NEXUS_CONTACT_L_FOOT    (1u << 4)
#define NEXUS_CONTACT_R_FOOT    (1u << 5)

/* fused_valid values. */
#define NEXUS_FUSION_INVALID    0u      /* estimator not running              */
#define NEXUS_FUSION_CONVERGING 1u      /* running, covariance still large    */
#define NEXUS_FUSION_OK         2u      /* converged, safe to use             */

/* act_flags bits, per joint. */
#define NEXUS_ACT_TELEM_FRESH   (1u << 0)
#define NEXUS_ACT_HB_FRESH      (1u << 1)

/* cmd flags bits. */
#define NEXUS_CMD_ENABLE        (1u << 0)

typedef struct __attribute__((packed))
{
    /* ---- header ------------------------------------------------ 0 */
    uint16_t sync;                           /*   0 */
    uint8_t  msg_id;                         /*   2 */
    uint8_t  version;                        /*   3 */
    uint32_t seq;                            /*   4 increments every 1 kHz tick */
    uint32_t timestamp_us;                   /*   8 free-running 1 MHz counter  */

    /* ---- IMU, raw from the BNO085 ------------------------------ 12 */
    float    imu_quat[4];                    /*  12 w, x, y, z                  */
    float    imu_accel[3];                   /*  28 m/s^2, gravity removed      */
    float    imu_gyro[3];                    /*  40 rad/s                       */
    uint32_t imu_seq;                        /*  52 lets the Pi spot staleness  */

    /* ---- after-spring joint angles, AS5048A -------------------- 56 */
    float    enc_angle[NEXUS_NUM_ENCODERS];  /*  56 radians, zero-referenced    */

    /* ---- actuators, from ODrive over CAN ----------------------- 72 */
    float    act_pos[NEXUS_NUM_JOINTS];      /*  72 turns                       */
    float    act_vel[NEXUS_NUM_JOINTS];      /* 112 turns/s                     */
    float    act_torque[NEXUS_NUM_JOINTS];   /* 152 Nm, estimate                */
    uint32_t act_error[NEXUS_NUM_JOINTS];    /* 192 raw ODrive axis_error       */

    /* ---- fused state of the lower torso ----------------------- 232 */
    float    fused_quat[4];                  /* 232 w,x,y,z; body -> world      */
    float    fused_pos[3];                   /* 248 m, world; [2] IS THE HEIGHT */
    float    fused_vel[3];                   /* 260 m/s, world frame            */
    float    fused_gyro_bias[3];             /* 272 rad/s, estimated            */
    float    fused_accel_bias[3];            /* 284 m/s^2, estimated            */

    /* ---- 2-byte fields ---------------------------------------- 296 */
    uint16_t contact_ticks[2];               /* 296 ticks each foot held state  */

    /* ---- 1-byte fields ---------------------------------------- 300 */
    uint8_t  act_state[NEXUS_NUM_JOINTS];    /* 300 raw ODrive axis_state       */
    uint8_t  act_flags[NEXUS_NUM_JOINTS];    /* 310 per-joint freshness         */
    uint8_t  enc_valid;                      /* 320 bit per encoder             */
    uint8_t  contacts;                       /* 321 switch + derived foot bits  */
    uint8_t  fused_valid;                    /* 322 NEXUS_FUSION_*              */
    uint8_t  health;                         /* 323 health.h bitmask            */

    uint16_t crc;                            /* 324 CRC16-CCITT over 0..323     */
} nexus_state_t;                             /* 326 total                       */

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

uint16_t nexus_crc16(const uint8_t *data, uint32_t len);

#endif /* LINK_PROTO_H */
