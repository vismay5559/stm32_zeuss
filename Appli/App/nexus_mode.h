#ifndef NEXUS_MODE_H
#define NEXUS_MODE_H

/*
 * Build-time selection of what this firmware does.
 *
 * The full robot loop touches every peripheral at once, which is the worst
 * possible way to bring hardware up: when nothing works you cannot tell which
 * of six subsystems is at fault. These modes each exercise exactly one
 * interface, so a failure has only one possible cause.
 *
 * Change NEXUS_MODE below, rebuild, reflash the Appli. Nothing else moves -
 * main() dispatches on this and the unused code is simply not entered.
 */

#define NEXUS_MODE_ROBOT     0   /* full 1 kHz loop, all subsystems           */
#define NEXUS_MODE_LEG_CAN   1   /* CAN-FD only: one leg, 4 ODrives           */
#define NEXUS_MODE_IMU       2   /* IMU only: print quaternion/accel/gyro     */

#ifndef NEXUS_MODE
#define NEXUS_MODE  NEXUS_MODE_IMU
#endif

#endif /* NEXUS_MODE_H */
