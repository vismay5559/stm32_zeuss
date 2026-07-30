#ifndef LINK_PROTO_H
#define LINK_PROTO_H

/*
 * Wire format for the STM32 <-> Raspberry Pi USB CDC link.
 * This file is shared verbatim with the Pi side - keep the two copies identical.
 */

#include <stdint.h>

#define NEXUS_SYNC              0xA5A5u
#define NEXUS_PROTO_VERSION     1u

#define NEXUS_MSG_STATE         0x01u
#define NEXUS_MSG_COMMAND       0x02u

#define NEXUS_NUM_JOINTS        10
#define NEXUS_NUM_ENCODERS      4

/* Contact bitmask positions, matching zeus_26 switch naming. */
#define NEXUS_CONTACT_L_TOE     (1u << 0)
#define NEXUS_CONTACT_L_HEEL    (1u << 1)
#define NEXUS_CONTACT_R_TOE     (1u << 2)
#define NEXUS_CONTACT_R_HEEL    (1u << 3)
/* Derived per-foot contact, computed on the STM32. */
#define NEXUS_CONTACT_L_FOOT    (1u << 4)
#define NEXUS_CONTACT_R_FOOT    (1u << 5)

/* fused_valid values. */
#define NEXUS_FUSION_INVALID    0u
#define NEXUS_FUSION_CONVERGING 1u
#define NEXUS_FUSION_OK         2u

/* act_flags bits, per joint. */
#define NEXUS_ACT_TELEM_FRESH   (1u << 0)
#define NEXUS_ACT_HB_FRESH      (1u << 1)

/* cmd flags bits. */
#define NEXUS_CMD_ENABLE        (1u << 0)

typedef struct __attribute__((packed))
{
    uint16_t sync;
    uint8_t  msg_id;
    uint8_t  version;
    uint32_t seq;
    uint32_t timestamp_us;

    float    imu_quat[4];        /* w, x, y, z */
    float    imu_accel[3];       /* m/s^2, gravity removed */
    float    imu_gyro[3];        /* rad/s */
    uint32_t imu_seq;            /* increments per IMU sample, lets the Pi spot staleness */

    uint16_t enc_angle[NEXUS_NUM_ENCODERS];  /* raw 14-bit AS5048A counts */
    uint8_t  enc_valid;                      /* bit per encoder */
    uint8_t  contacts;                       /* switch bits + derived per-foot bits */
    uint16_t contact_ticks[2];               /* 1 kHz ticks each foot has held state */

    float    act_pos[NEXUS_NUM_JOINTS];      /* turns */
    float    act_vel[NEXUS_NUM_JOINTS];      /* turns/s */
    float    act_torque[NEXUS_NUM_JOINTS];   /* Nm */
    uint32_t act_error[NEXUS_NUM_JOINTS];    /* raw ODrive axis_error */
    uint8_t  act_state[NEXUS_NUM_JOINTS];    /* raw ODrive axis_state */
    uint8_t  act_flags[NEXUS_NUM_JOINTS];

    /* Filled by the on-board estimator. Zero until fusion is implemented. */
    float    fused_quat[4];   /* w, x, y, z; body -> world */
    float    fused_pos[3];    /* m, world frame; [2] is height above ground */
    float    fused_vel[3];    /* m/s, world frame */
    float    fused_gyro_bias[3];
    float    fused_accel_bias[3];
    uint8_t  fused_valid;

    uint16_t crc;
} nexus_state_t;

typedef struct __attribute__((packed))
{
    uint16_t sync;
    uint8_t  msg_id;
    uint8_t  version;
    uint32_t seq;
    float    target_pos[NEXUS_NUM_JOINTS];   /* turns */
    uint16_t flags;
    uint16_t crc;
} nexus_cmd_t;

uint16_t nexus_crc16(const uint8_t *data, uint32_t len);

#endif /* LINK_PROTO_H */
