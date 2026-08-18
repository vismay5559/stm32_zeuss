#include "safety.h"
#include "act_odrive.h"

#include <math.h>
#include <string.h>

static safety_state_t s_state;
static uint32_t       s_rejected;
static uint8_t        s_reject_run;      /* consecutive rejections           */

static uint8_t        s_have_seq;        /* has any command been accepted?   */
static uint32_t       s_last_seq;
static float          s_last_pos[NEXUS_NUM_JOINTS];

/*
 * Set when FAULT is entered, cleared only by a command that has ENABLE OFF.
 * While it is set, an ENABLE-on command cannot arm - which is what stops a
 * flickering link from silently handing control back mid-stride.
 */
static uint8_t        s_needs_rearm;

void safety_init(void)
{
    s_state       = SAFETY_BOOT;
    s_rejected    = 0;
    s_reject_run  = 0;
    s_have_seq    = 0;
    s_last_seq    = 0;
    s_needs_rearm = 0;
    memset(s_last_pos, 0, sizeof(s_last_pos));
}

safety_state_t safety_state(void)
{
    return s_state;
}

uint32_t safety_rejected(void)
{
    return s_rejected;
}

const char *safety_state_name(void)
{
    switch (s_state)
    {
    case SAFETY_BOOT:  return "BOOT";
    case SAFETY_IDLE:  return "IDLE";
    case SAFETY_ARMED: return "ARMED";
    case SAFETY_FAULT: return "FAULT";
    default:           return "?";
    }
}

static void enter_fault(void)
{
    if (s_state == SAFETY_FAULT)
    {
        return;
    }

    s_state       = SAFETY_FAULT;
    s_needs_rearm = 1;

    /* Order matters: stop commanding first, then ask the drives to idle.
       The other way round leaves one more SET_INPUT_POS chasing the IDLE. */
    act_disarm();
}

void safety_tick(uint32_t faults)
{
    if (faults & SAFETY_FATAL_FAULTS)
    {
        enter_fault();
        return;
    }

    switch (s_state)
    {
    case SAFETY_BOOT:
        /*
         * Leave BOOT only once nothing that is being watched is faulted.
         * health_faults() is already masked by health_expected(), so on a
         * partially wired bench this reflects exactly what is plugged in.
         */
        if (faults == 0u)
        {
            s_state = SAFETY_IDLE;
        }
        break;

    case SAFETY_FAULT:
        /* Faults have cleared - but go to IDLE, never straight to ARMED. */
        s_state = SAFETY_IDLE;
        break;

    case SAFETY_IDLE:
    case SAFETY_ARMED:
    default:
        break;
    }
}

static uint8_t targets_are_sane(const nexus_cmd_t *cmd)
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        float p = cmd->target_pos[j];

        /*
         * A NaN is worse than an out-of-range number. It passes every
         * comparison silently, and act_set_targets() would write it into
         * s_seg_start/s_seg_end, where it poisons the interpolator for good -
         * every subsequent output is NaN even after clean commands resume.
         */
        if (!isfinite(p))
        {
            return 0;
        }

        if ((p < SAFETY_POS_MIN_TURNS) || (p > SAFETY_POS_MAX_TURNS))
        {
            return 0;
        }

        if (s_have_seq)
        {
            float step = p - s_last_pos[j];

            if (step < 0.0f)
            {
                step = -step;
            }
            if (step > SAFETY_MAX_STEP_TURNS)
            {
                return 0;
            }
        }
    }

    return 1;
}

static void reject(void)
{
    s_rejected++;

    if (s_reject_run < 0xFFu)
    {
        s_reject_run++;
    }

    /*
     * One bad frame is a glitch. A run of them means the far end is sending
     * something this firmware does not understand, and continuing to hold the
     * last good target while that goes on is not a safe default.
     */
    if (s_reject_run >= SAFETY_MAX_REJECTS)
    {
        enter_fault();
    }
}

uint8_t safety_accept_command(const nexus_cmd_t *cmd,
                              float targets_out[NEXUS_NUM_JOINTS])
{
    uint8_t enabled = (cmd->flags & NEXUS_CMD_ENABLE) ? 1u : 0u;

    if (!enabled)
    {
        /*
         * An explicit "stand down" from the Pi. Not a rejection - this is the
         * Pi using the protocol correctly, and it is also the handshake that
         * clears a latched fault.
         */
        s_needs_rearm = 0;

        if (s_state == SAFETY_ARMED)
        {
            s_state = SAFETY_IDLE;
            act_disarm();
        }
        return 0;
    }

    if ((s_state != SAFETY_IDLE) && (s_state != SAFETY_ARMED))
    {
        return 0;   /* BOOT or FAULT - nothing may reach the actuators */
    }

    if (s_needs_rearm)
    {
        return 0;   /* waiting for the ENABLE-off handshake after a fault */
    }

    /*
     * Ordering: a replayed or stale frame is discarded before its contents
     * are examined, so a stuck sender cannot keep an old target alive.
     * Signed difference handles the 32-bit wrap without a special case.
     */
    if (s_have_seq)
    {
        int32_t advance = (int32_t)(cmd->seq - s_last_seq);

        if (advance <= 0)
        {
            reject();
            return 0;
        }
    }

    if (!targets_are_sane(cmd))
    {
        reject();
        return 0;
    }

    s_reject_run = 0;
    s_last_seq   = cmd->seq;
    s_have_seq   = 1;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_last_pos[j]  = cmd->target_pos[j];
        targets_out[j] = cmd->target_pos[j];
    }

    s_state = SAFETY_ARMED;
    return 1;
}
