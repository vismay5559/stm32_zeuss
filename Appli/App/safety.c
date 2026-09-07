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

/* Enabled commands spent waiting for the drives to reach closed loop. */
static uint32_t       s_arm_wait;

void safety_init(void)
{
    s_state       = SAFETY_BOOT;
    s_rejected    = 0;
    s_reject_run  = 0;
    s_have_seq    = 0;
    s_last_seq    = 0;
    s_needs_rearm = 0;
    s_arm_wait    = 0;
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
    s_arm_wait    = 0;

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

/*
 * Two checks, deliberately separate.
 *
 * The residual is bounded on its own so a policy cannot use it as a
 * trajectory. The SUM is then bounded again, because a residual well inside
 * its limit can still push a joint past its envelope when the reference is
 * already near one - and because the slew limit has to see the value the
 * drives will actually be given, not the correction on top of it.
 */
static uint8_t targets_are_sane(const nexus_cmd_t *cmd,
                                const float ref_turns[NEXUS_NUM_JOINTS],
                                float targets_out[NEXUS_NUM_JOINTS])
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        float r = cmd->residual[j];

        /* The reference is ours, but a broken gait table must not become a
           command either. */
        if (!isfinite(ref_turns[j]))
        {
            return 0;
        }
        if (!isfinite(r))
        {
            return 0;
        }
        if ((r < -SAFETY_MAX_RESIDUAL_TURNS) || (r > SAFETY_MAX_RESIDUAL_TURNS))
        {
            return 0;
        }

        float p = ref_turns[j] + r;

        targets_out[j] = p;

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
                              const float ref_turns[NEXUS_NUM_JOINTS],
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
        /*
         * Acknowledge whatever stopped us at the same time. A latched timing
         * fault would otherwise still be standing on the next enabled command,
         * and the robot could never be armed again without a power cycle.
         */
        if (s_needs_rearm)
        {
            health_clear_latched();
        }
        s_needs_rearm = 0;

        s_arm_wait = 0;

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

    if (!targets_are_sane(cmd, ref_turns, targets_out))
    {
        reject();
        return 0;
    }

    /*
     * The command is good. Before acting on it, make sure the drives are
     * actually in a state to act.
     *
     * This firmware used to send position commands and assume ten ODrives had
     * been put into closed-loop control by hand before power-on - so an axis
     * that tripped mid-run could only be recovered by someone with a laptop,
     * and an axis that was never armed silently ignored everything.
     */
    if (!act_all_closed_loop())
    {
        act_request_arm();

        if (++s_arm_wait >= SAFETY_ARM_TIMEOUT_CMDS)
        {
            enter_fault();
        }
        return 0;
    }

    s_arm_wait   = 0;
    s_reject_run = 0;
    s_last_seq   = cmd->seq;
    s_have_seq   = 1;

    /* targets_out was filled by targets_are_sane, which is also what the slew
       limit compared against - so the two can never disagree. */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_last_pos[j] = targets_out[j];
    }

    s_state = SAFETY_ARMED;
    return 1;
}
