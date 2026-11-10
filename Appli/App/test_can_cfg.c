/*
 * Read and set ODrive CAN telemetry rates over the bus, without arming anything.
 *
 * WHY A MODE OF ITS OWN. test_can_scan is receive-only and that is its whole
 * value - it cannot disturb the robot. This one transmits, so it says so in its
 * name and lives in its own file. It sends exactly three kinds of frame:
 * Get_Version, and SDO read and write. It NEVER sends Set_Axis_State (0x007),
 * Set_Input_Pos, or anything else that could move a joint, and there is no
 * code path here that could.
 *
 * WHY NOT flat_endpoints.json. Writing a parameter over CANSimple needs its
 * endpoint id, those ids move with every firmware revision, and a stale one
 * "lands on whatever parameter now occupies that slot and silently corrupts
 * it" (docs/ODRIVE_COMMANDS.md). The json for these drives is not in the tree,
 * so guessing is the one thing not on the table.
 *
 * It does not have to be guessed. The rates are OBSERVABLE: test_can_scan
 * already counted, per drive, how many heartbeats and encoder frames arrive per
 * second, so the period each drive is configured for is known before a single
 * byte is written. Reading a candidate endpoint and comparing it against that
 * measurement identifies the parameter positively - and reads cannot corrupt
 * anything, so a wrong guess costs nothing but a line of output.
 *
 * As measured on 2026-11-10:
 *
 *   bus 0 nodes 1-4, bus 1 node 2   heartbeat 10 ms, encoder  1 ms
 *   bus 1 nodes 3-4                 heartbeat 100 ms, encoder 10 ms
 *
 * So the endpoint that reads 10 on one and 100 on the other is
 * heartbeat_msg_rate_ms, and the one that reads 1 against 10 is
 * encoder_msg_rate_ms. Two drives disagreeing is what makes this work; if every
 * drive were already identical the sweep could not tell the parameters apart,
 * and the json would be the only way.
 *
 * CANCFG_WRITE stays 0 until a sweep has named the endpoints. Nothing is
 * written while it is 0.
 */

#include "test_can_cfg.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define CANCFG_HEARTBEAT     0x001u
#define CANCFG_GET_VERSION   0x000u
#define CANCFG_RX_SDO        0x004u
#define CANCFG_TX_SDO        0x005u
#define SDO_OP_READ          0x00u
#define SDO_OP_WRITE         0x01u

/* The firmware the endpoint ids in the tree were taken from. */
#define EXPECT_HW_LINE   5u
#define EXPECT_HW_VER    2u
#define EXPECT_FW_MAJOR  0u
#define EXPECT_FW_MINOR  6u
#define EXPECT_FW_REV    12u

/*
 * The sweep range. axis0.config.can.input_vel_scale is endpoint 292 per
 * docs/ODRIVE_COMMANDS.md, and load_encoder / commutation_encoder are 294 and
 * 295, so the axis0.config.can subtree sits around there. This brackets it
 * generously - reads are free.
 */
#define SWEEP_FIRST   280u
#define SWEEP_LAST    320u

/*
 * 1 = write the rates. The endpoints below were identified by the sweep, not
 * guessed, and each is confirmed against something measured:
 *
 *   ep 282  node_id                reads back the node we queried, which
 *                                  proves we are in the right drive's subtree
 *   ep 284  heartbeat_msg_rate_ms  read 10 on a drive measured at 10 ms and
 *                                  100 on one measured at 100 ms
 *   ep 285  encoder_msg_rate_ms    read 1 against 10, likewise
 *   ep 290  torque_msg_rate_ms     read 10 where the frame count showed a
 *                                  third stream at 100 Hz, and 0 where the
 *                                  count showed none
 */
#define CANCFG_WRITE  1

/*
 * 1 = also call save_configuration, so the rates survive a power cycle.
 *
 * This REBOOTS every drive it touches. Each one drops off the bus for a
 * second or two and comes back, so it is not something to run while someone
 * is tracing a drive that is already missing - hence a switch of its own
 * rather than part of the write above.
 *
 * The reboot is also the only honest proof of persistence. A read-back
 * straight after the save would just be reading RAM, which already held the
 * value before the save; a read-back AFTER the drive has restarted can only
 * have come from flash. So the save stage waits for the heartbeat to return
 * and reads the three rates again on the other side of the restart.
 */
#define CANCFG_SAVE   1

#define EP_NODE_ID         282u
#define EP_HEARTBEAT_MS    284u
#define EP_ENCODER_MS      285u
#define EP_TORQUE_MS       290u
#define EP_SAVE_CONFIG     718u

/* What we want, once the endpoints are known. */
#define WANT_ENCODER_MS    1u
#define WANT_HEARTBEAT_MS  10u
#define WANT_TORQUE_MS     10u

static volatile uint8_t  s_sdo_got;
static volatile uint16_t s_sdo_ep;
static volatile uint32_t s_sdo_val;
static volatile uint8_t  s_ver_got;
static volatile uint8_t  s_ver[8];
static volatile uint32_t s_hb_seen[2];
static volatile uint32_t s_count[2][8][3];   /* [bus][node][0=hb 1=enc 2=torque] */

static void start_bus(FDCAN_HandleTypeDef *h)
{
    FDCAN_FilterTypeDef f = {0};

    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0u;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000u;
    f.FilterID2    = 0x7FFu;
    (void)HAL_FDCAN_ConfigFilter(h, &f);
    (void)HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_Start(h);
    (void)HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u);
}

/*
 * CLASSIC frames, not FD.
 *
 * A drive that is not configured for CAN-FD sees an FD frame as malformed and
 * flags an error - which is what happened when a probe frame went out as
 * FD+BRS and the drives changed colour. Every drive accepts classic frames
 * whatever its own FD setting is, so a query cannot repeat that.
 */
static void tx_classic(FDCAN_HandleTypeDef *h, uint32_t node, uint32_t cmd,
                       const uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t buf[8] = {0};

    if (len > 8u) { len = 8u; }
    if (data != 0) { memcpy(buf, data, len); }

    tx.Identifier          = (node << 5) | cmd;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = len;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, buf);
}

void cancfg_on_rx(void *hfdcan)
{
    FDCAN_HandleTypeDef *h = (FDCAN_HandleTypeDef *)hfdcan;
    uint8_t bus = (h->Instance == FDCAN2) ? 1u : 0u;
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t d[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, d) != HAL_OK) { return; }
        if (hdr.IdType != FDCAN_STANDARD_ID) { continue; }

        uint32_t cmd = hdr.Identifier & 0x1Fu;

        uint32_t n = (hdr.Identifier >> 5) & 0x1Fu;

        if (cmd == CANCFG_HEARTBEAT)
        {
            s_hb_seen[bus] |= 1u << n;
            if (n < 8u) { s_count[bus][n][0]++; }
        }
        else if (cmd == 0x009u && n < 8u) { s_count[bus][n][1]++; }
        else if (cmd == 0x01Cu && n < 8u) { s_count[bus][n][2]++; }
        else if (cmd == CANCFG_TX_SDO)
        {
            s_sdo_ep  = (uint16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
            s_sdo_val = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                      | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            s_sdo_got = 1u;
        }
        else if (cmd == CANCFG_GET_VERSION)
        {
            memcpy((void *)s_ver, d, 8u);
            s_ver_got = 1u;
        }
    }
}

/*
 * Pump the receive path for a fixed time, unconditionally.
 *
 * Needed because wait_flag() returns the instant its flag is set, and the
 * first version of the frame-count proof waited on s_ver_got - which the
 * version check had already set. The window was zero milliseconds, every
 * counter read 0, and it looked exactly like seven drives having gone silent
 * right after being written to. A measurement that cannot distinguish "no
 * frames" from "no time" is worse than none.
 */
static void pump_for(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++)
    {
        cancfg_on_rx(&hfdcan1);
        cancfg_on_rx(&hfdcan2);
        HAL_Delay(1);
    }
}

static uint8_t wait_flag(volatile uint8_t *flag, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++)
    {
        cancfg_on_rx(&hfdcan1);
        cancfg_on_rx(&hfdcan2);
        if (*flag) { return 1u; }
        HAL_Delay(1);
    }
    return 0u;
}

static uint8_t sdo_read_u32(FDCAN_HandleTypeDef *h, uint8_t node,
                            uint16_t ep, uint32_t *out)
{
    uint8_t d[8] = {0};

    d[0] = SDO_OP_READ;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);

    s_sdo_got = 0u;
    tx_classic(h, node, CANCFG_RX_SDO, d, 8u);
    if (!wait_flag(&s_sdo_got, 50u)) { return 0u; }
    if (s_sdo_ep != ep) { return 0u; }
    *out = s_sdo_val;
    return 1u;
}

static uint8_t sdo_write_u32(FDCAN_HandleTypeDef *h, uint8_t node,
                             uint16_t ep, uint32_t v)
{
    uint8_t d[8] = {0};

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[4] = (uint8_t)(v & 0xFFu);
    d[5] = (uint8_t)((v >> 8) & 0xFFu);
    d[6] = (uint8_t)((v >> 16) & 0xFFu);
    d[7] = (uint8_t)((v >> 24) & 0xFFu);

    tx_classic(h, node, CANCFG_RX_SDO, d, 8u);
    HAL_Delay(5);
    return 1u;
}

/*
 * Set one rate and prove it landed. A write is fire-and-forget on this
 * protocol, so the read-back is the only evidence - and reading back the
 * value we just sent is also what would catch a wrong endpoint id, because
 * the parameter that actually occupies that slot would not hold it.
 */
static uint8_t set_rate(FDCAN_HandleTypeDef *h, uint8_t node, uint16_t ep,
                        uint32_t want, const char *what)
{
    uint32_t before = 0u, after = 0u;
    uint8_t had = sdo_read_u32(h, node, ep, &before);

    if (!had)
    {
        printf("      %-22s ep %u: no reply to the read - not written\r\n",
               what, (unsigned)ep);
        return 0u;
    }
    if (before == want)
    {
        printf("      %-22s %lu ms already\r\n", what, (unsigned long)before);
        return 1u;
    }

    (void)sdo_write_u32(h, node, ep, want);
    if (!sdo_read_u32(h, node, ep, &after))
    {
        printf("      %-22s wrote %lu but the read-back failed\r\n",
               what, (unsigned long)want);
        return 0u;
    }
    printf("      %-22s %lu -> %lu ms  %s\r\n", what,
           (unsigned long)before, (unsigned long)after,
           (after == want) ? "ok" : "*** DID NOT TAKE ***");
    return (after == want) ? 1u : 0u;
}

#if CANCFG_WRITE && CANCFG_SAVE
/*
 * Save one drive's configuration and prove it stuck.
 *
 * save_configuration is a FUNCTION endpoint, not a parameter: the call is an
 * SDO write to its id and the payload is ignored. There is nothing to read
 * back from it, and the drive reboots rather than replying, so the only
 * sequence that means anything is: watch the heartbeat stop, watch it come
 * back, then re-read the parameters. Anything that reads 1/10/10 after the
 * drive has restarted was loaded from flash, because RAM did not survive.
 *
 * A drive refuses to save unless its axis is idle. Nothing here has ever
 * armed one, so that holds - but it is the reason a failure here would show
 * up as "rates did not persist" rather than as an error: the refusal is
 * silent on this protocol.
 */
static uint8_t save_node(FDCAN_HandleTypeDef *h, uint8_t bus, uint8_t node)
{
    uint8_t  d[8] = {0};
    uint32_t id = 0xFFFFFFFFu, enc = 0u, hb = 0u, tq = 0u;

    if (!sdo_read_u32(h, node, EP_NODE_ID, &id) || id != node)
    {
        printf("    bus %u node %u: node_id reads %lu - NOT SAVED\r\n",
               (unsigned)bus, (unsigned)node, (unsigned long)id);
        return 0u;
    }

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(EP_SAVE_CONFIG & 0xFFu);
    d[2] = (uint8_t)(EP_SAVE_CONFIG >> 8);

    s_hb_seen[bus] &= ~(1u << node);
    tx_classic(h, node, CANCFG_RX_SDO, d, 8u);
    printf("    bus %u node %u: saving", (unsigned)bus, (unsigned)node);

    /* Give it a moment to actually go down, so that seeing the heartbeat is
       evidence of a restart and not of a save that never happened. */
    pump_for(200u);
    s_hb_seen[bus] &= ~(1u << node);

    /*
     * 30 s, because 10 s was not enough and that mattered. The first run of
     * this reported "DID NOT COME BACK" for every drive it reached, which
     * reads like seven bricked drives; a read-only scan afterwards found them
     * all idle, error-free and sending at the new rates. They were simply
     * still erasing and rebooting. An erase-and-reboot is one of the few
     * things on this robot that genuinely takes tens of seconds, and a
     * diagnostic that cries brick on a drive that is merely busy is worse
     * than one that says nothing.
     */
    uint32_t waited = 0u;
    while ((s_hb_seen[bus] & (1u << node)) == 0u && waited < 30000u)
    {
        pump_for(100u);
        waited += 100u;
    }
    if ((s_hb_seen[bus] & (1u << node)) == 0u)
    {
        printf(" - no heartbeat yet after %lu ms. It is probably still\r\n"
               "      rebooting; re-run CAN_SCAN in a minute before assuming\r\n"
               "      anything is wrong with it.\r\n", (unsigned long)waited);
        return 0u;
    }
    printf(" - back after %lu ms", (unsigned long)waited);

    /* Let the CAN stack settle before asking it questions. */
    pump_for(500u);

    uint8_t ok = sdo_read_u32(h, node, EP_ENCODER_MS,   &enc)
              && sdo_read_u32(h, node, EP_HEARTBEAT_MS, &hb)
              && sdo_read_u32(h, node, EP_TORQUE_MS,    &tq);
    if (!ok)
    {
        printf(", but it will not answer an SDO read\r\n");
        return 0u;
    }

    uint8_t good = (enc == WANT_ENCODER_MS) && (hb == WANT_HEARTBEAT_MS)
                && (tq == WANT_TORQUE_MS);
    printf(", from flash: enc=%lu hb=%lu torque=%lu  %s\r\n",
           (unsigned long)enc, (unsigned long)hb, (unsigned long)tq,
           good ? "PERSISTED" : "*** NOT WHAT WE WROTE ***");
    return good;
}
#endif

static uint8_t check_version(FDCAN_HandleTypeDef *h, uint8_t node)
{
    s_ver_got = 0u;
    tx_classic(h, node, CANCFG_GET_VERSION, 0, 0u);
    if (!wait_flag(&s_ver_got, 250u))
    {
        printf("    node %u: no Get_Version reply\r\n", (unsigned)node);
        return 0u;
    }
    unsigned line = s_ver[1], hw = s_ver[2], var = s_ver[3];
    unsigned maj = s_ver[4], min = s_ver[5], rev = s_ver[6];
    uint8_t ok = (line == EXPECT_HW_LINE) && (hw == EXPECT_HW_VER)
              && (maj == EXPECT_FW_MAJOR) && (min == EXPECT_FW_MINOR)
              && (rev == EXPECT_FW_REV);
    printf("    node %u: hw %u.%u.%u  fw %u.%u.%u  %s\r\n",
           (unsigned)node, line, hw, var, maj, min, rev,
           ok ? "matches the tree's endpoint ids"
              : "*** DIFFERENT FIRMWARE - endpoint ids in this repo do not apply ***");
    return ok;
}

void cancfg_init(void)
{
    memset((void *)s_hb_seen, 0, sizeof(s_hb_seen));
    start_bus(&hfdcan1);
    start_bus(&hfdcan2);
    printf("CAN CONFIG mode: Get_Version and SDO only. "
           "No axis state is ever commanded.\r\n");
#if CANCFG_WRITE
    printf("*** WRITES ENABLED ***\r\n");
#else
    printf("Writes are DISABLED (CANCFG_WRITE 0): this run only reads.\r\n");
#endif
}

void cancfg_run(void)
{
    /* Listen first: only talk to drives that are actually there. */
    printf("\r\nlistening 1 s for heartbeats...\r\n");
    pump_for(1000u);

    for (uint8_t bus = 0u; bus < 2u; bus++)
    {
        FDCAN_HandleTypeDef *h = (bus == 0u) ? &hfdcan1 : &hfdcan2;

        printf("\r\nbus %u: nodes seen 0x%02lX\r\n", (unsigned)bus,
               (unsigned long)s_hb_seen[bus]);

        for (uint8_t node = 1u; node <= 4u; node++)
        {
            if ((s_hb_seen[bus] & (1u << node)) == 0u) { continue; }
            (void)check_version(h, node);
        }
    }

    /*
     * Sweep the candidate range on the FIRST live node of each bus. Two drives
     * with different rates is what identifies the parameters, so both buses are
     * swept and the values printed side by side.
     */
    printf("\r\nendpoint sweep %u..%u, read as uint32\r\n",
           (unsigned)SWEEP_FIRST, (unsigned)SWEEP_LAST);
    printf("  looking for: heartbeat 10 vs 100, encoder 1 vs 10\r\n");
    printf("  %-6s %-14s %-14s\r\n", "ep", "bus0 node1", "bus1 node3");

    for (uint16_t ep = SWEEP_FIRST; ep <= SWEEP_LAST; ep++)
    {
        uint32_t a = 0u, b = 0u;
        uint8_t oka = sdo_read_u32(&hfdcan1, 1u, ep, &a);
        uint8_t okb = sdo_read_u32(&hfdcan2, 3u, ep, &b);

        if (!oka && !okb) { continue; }

        const char *hint = "";
        if (oka && okb)
        {
            if (a == 10u && b == 100u) { hint = "  <== heartbeat_msg_rate_ms?"; }
            else if (a == 1u && b == 10u) { hint = "  <== encoder_msg_rate_ms?"; }
        }
        printf("  %-6u ", (unsigned)ep);
        if (oka) { printf("%-14lu ", (unsigned long)a); } else { printf("%-14s ", "-"); }
        if (okb) { printf("%-14lu", (unsigned long)b); } else { printf("%-14s", "-"); }
        printf("%s\r\n", hint);
    }

    printf("\r\nsweep done.\r\n");

#if CANCFG_WRITE
    printf("\r\n=== setting rates: encoder %u ms, heartbeat %u ms, torque %u ms ===\r\n",
           (unsigned)WANT_ENCODER_MS, (unsigned)WANT_HEARTBEAT_MS,
           (unsigned)WANT_TORQUE_MS);
    printf("No axis state is commanded. Nothing is armed.\r\n");

    for (uint8_t bus = 0u; bus < 2u; bus++)
    {
        FDCAN_HandleTypeDef *h = (bus == 0u) ? &hfdcan1 : &hfdcan2;

        for (uint8_t node = 1u; node <= 4u; node++)
        {
            if ((s_hb_seen[bus] & (1u << node)) == 0u) { continue; }

            /* Confirm we are addressing the drive we think we are before
               writing a single byte to it. */
            uint32_t id = 0xFFFFFFFFu;
            if (!sdo_read_u32(h, node, EP_NODE_ID, &id) || id != node)
            {
                printf("    bus %u node %u: node_id reads %lu - SKIPPED\r\n",
                       (unsigned)bus, (unsigned)node, (unsigned long)id);
                continue;
            }

            printf("    bus %u node %u:\r\n", (unsigned)bus, (unsigned)node);
            (void)set_rate(h, node, EP_ENCODER_MS,   WANT_ENCODER_MS,   "encoder_msg_rate_ms");
            (void)set_rate(h, node, EP_HEARTBEAT_MS, WANT_HEARTBEAT_MS, "heartbeat_msg_rate_ms");
            (void)set_rate(h, node, EP_TORQUE_MS,    WANT_TORQUE_MS,    "torque_msg_rate_ms");
        }
    }

    /*
     * The proof. Read-back says the parameter holds the value; only the frame
     * count says the drive is actually SENDING at that rate, which is the
     * thing being asked for and the one check a wrong endpoint cannot pass.
     */
    printf("\r\n=== counting frames for 2 s to prove the rates took ===\r\n");
    memset((void *)s_count, 0, sizeof(s_count));
    pump_for(2000u);

    printf("    want: hb 200, enc 2000, torque 200 over 2 s\r\n");
    for (uint8_t bus = 0u; bus < 2u; bus++)
    {
        for (uint8_t node = 1u; node <= 4u; node++)
        {
            if ((s_hb_seen[bus] & (1u << node)) == 0u) { continue; }
            printf("    bus %u node %u: hb=%-5lu enc=%-5lu torque=%-5lu\r\n",
                   (unsigned)bus, (unsigned)node,
                   (unsigned long)s_count[bus][node][0],
                   (unsigned long)s_count[bus][node][1],
                   (unsigned long)s_count[bus][node][2]);
        }
    }

#if !CANCFG_SAVE
    printf("\r\nNOT SAVED. These revert when a drive is power-cycled.\r\n");
    printf("Saving reboots each drive, which drops it off the bus for a moment -\r\n");
    printf("not something to do behind someone's back while they are working on it.\r\n");
#else
    printf("\r\n=== save_configuration: writing the rates to flash ===\r\n");
    printf("Each drive REBOOTS. Still no axis state commanded, still nothing armed.\r\n");

    for (uint8_t bus = 0u; bus < 2u; bus++)
    {
        FDCAN_HandleTypeDef *h = (bus == 0u) ? &hfdcan1 : &hfdcan2;

        for (uint8_t node = 1u; node <= 4u; node++)
        {
            if ((s_hb_seen[bus] & (1u << node)) == 0u) { continue; }
            (void)save_node(h, bus, node);
        }
    }
#endif
#endif

    for (;;) { HAL_Delay(1000); }
}
