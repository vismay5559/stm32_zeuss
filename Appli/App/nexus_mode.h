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

#define NEXUS_MODE_ROBOT      0  /* full 1 kHz loop, all subsystems            */
#define NEXUS_MODE_LEG_CAN    1  /* CAN-FD only: one leg, ODrive position mode */
#define NEXUS_MODE_IMU        2  /* IMU only: print quaternion/accel/gyro      */
#define NEXUS_MODE_LEG_TORQUE 3  /* one leg, STM32-side PD -> Set_Input_Torque */

/*
 * THIS LINE is the mode. Edit it, rebuild, reflash.
 *
 * The build can still override it for a one-off without touching the file,
 * which is useful in CI or when trying a mode you do not want to commit:
 *
 *     NEXUS_MODE=NEXUS_MODE_LEG_CAN cmake --preset Debug    # whole project
 *     cmake -S Appli -B Appli/build -DNEXUS_MODE=...        # this one alone
 *
 * The build only defines NEXUS_MODE when you actually ask for one, so with no
 * -D and no environment variable this file wins. Configure prints which of the
 * two is in force, so the binary is never a mystery.
 *
 * Keep ROBOT committed. A board flashed with a test mode answers no USB
 * packets and drives one leg from a canned trajectory, which from the Pi's
 * side is indistinguishable from a dead link - so the failure of forgetting to
 * change it back is silent.
 */
#ifndef NEXUS_MODE
#define NEXUS_MODE  NEXUS_MODE_LEG_CAN
#endif

/* Short name for the boot banner and for the mode byte the Pi receives. */
#if   (NEXUS_MODE == NEXUS_MODE_ROBOT)
#define NEXUS_MODE_NAME  "ROBOT"
#elif (NEXUS_MODE == NEXUS_MODE_LEG_CAN)
#define NEXUS_MODE_NAME  "LEG_CAN"
#elif (NEXUS_MODE == NEXUS_MODE_IMU)
#define NEXUS_MODE_NAME  "IMU"
#elif (NEXUS_MODE == NEXUS_MODE_LEG_TORQUE)
#define NEXUS_MODE_NAME  "LEG_TORQUE"
#else
#error "NEXUS_MODE must be one of NEXUS_MODE_ROBOT / _LEG_CAN / _IMU / _LEG_TORQUE"
#endif

#endif /* NEXUS_MODE_H */
