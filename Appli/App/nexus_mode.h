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
#define NEXUS_MODE_LEG_TORQUE 3  /* one leg, STM32-side PD -> Set_Input_Torque */

/*
 * The default is the ROBOT loop, deliberately.
 *
 * A test mode has to be asked for, because the failure it causes is silent: a
 * board flashed with LEG_CAN answers no USB packets and drives one leg from a
 * canned trajectory, which from the Pi's side is indistinguishable from a dead
 * link. Select a test mode at configure time rather than by editing this file:
 *
 *     cmake --preset Release -DNEXUS_MODE=NEXUS_MODE_LEG_CAN
 *
 * That way the choice appears in the build log and in the .elf, instead of
 * living in an uncommitted local edit.
 */
#ifndef NEXUS_MODE
#define NEXUS_MODE  NEXUS_MODE_ROBOT
#endif

/* Short name for the boot banner and for the mode byte the Pi receives. */
#if   (NEXUS_MODE == NEXUS_MODE_ROBOT)
#define NEXUS_MODE_NAME  "ROBOT"
#elif (NEXUS_MODE == NEXUS_MODE_LEG_CAN)
#define NEXUS_MODE_NAME  "LEG_CAN"
#elif (NEXUS_MODE == NEXUS_MODE_IMU)
#define NEXUS_MODE_NAME  "IMU"
#else
#error "NEXUS_MODE is not one of NEXUS_MODE_ROBOT / _LEG_CAN / _IMU"
#endif

#endif /* NEXUS_MODE_H */
