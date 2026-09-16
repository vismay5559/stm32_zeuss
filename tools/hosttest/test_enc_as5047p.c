/*
 * Host test for enc_as5047p.c - the four sensors that measure how far each
 * leg spring has been wound up.
 *
 * That measurement is how the robot knows how hard a foot is pushing on the
 * ground, so a wrong reading accepted as good is a robot that thinks it is
 * standing on something when it is not. Every sensor word carries its own
 * check digit and its own "I am unwell" flag, and the whole point of this
 * file is that both are honoured.
 *
 * The sensors sit on two chains, one per leg, each behind its own chip
 * select. The left chain is read, then the right one straight after. The
 * other half of this file is what happens when a reading never arrives. The
 * sensors are read by the hardware on its own, in the background, and the
 * driver is told afterwards. If that "afterwards" never comes - a bus glitch,
 * an aborted transfer - the driver used to wait forever, handing out its last
 * readings as though they were current.
 *
 * Vocabulary:
 *
 *   word       the sixteen bits one sensor sends back. Fourteen are the
 *              angle, one says the sensor is unhappy, one is the check digit.
 *   check digit  a single bit chosen so the number of 1s in the word is even.
 *              Flip any one bit in transit and the count goes odd, which is
 *              how a corrupted word is spotted.
 *   CS         the wire that says "I am talking to you now". One per leg,
 *              held low for that leg's read and released afterwards.
 *   in flight  a read that has been started but not yet answered.
 *
 * A chain answers in wiring order: word 0 from the sensor nearest MISO (the
 * knee), word 1 from the one nearest MOSI (the hip).
 */

#include "enc_as5047p.h"
#include "link_proto.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

static int s_fail;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

#define ERROR_FLAG     0x4000u
#define ANGLE_MASK     0x3FFFu
#define READ_ANGLE     0xFFFFu
#define READ_ERRFL     0x4001u
#define TIMEOUT_TICKS  5      /* must match ENC_XFER_TIMEOUT_TICKS */

#define ALL_VALID      0x0Fu
#define BIT(e)         ((uint8_t)(1u << (e)))

/* ---- building a sensor word the way the sensor does ------------------ */

static uint16_t with_check_digit(uint16_t payload)
{
    uint16_t v = (uint16_t)(payload & 0x7FFFu);
    uint16_t x = v;

    x ^= (uint16_t)(x >> 8);
    x ^= (uint16_t)(x >> 4);
    x ^= (uint16_t)(x >> 2);
    x ^= (uint16_t)(x >> 1);

    return (uint16_t)(v | (uint16_t)((x & 1u) << 15));
}

static uint16_t good_word(uint16_t angle)
{
    return with_check_digit((uint16_t)(angle & ANGLE_MASK));
}

static uint16_t unwell_word(uint16_t angle)
{
    return with_check_digit((uint16_t)(ERROR_FLAG | (angle & ANGLE_MASK)));
}

/* A word the sensor would never send: one bit of it flipped in transit. */
static uint16_t corrupted(uint16_t word)
{
    return (uint16_t)(word ^ 0x0004u);
}

static int left_cs_released(void)
{
    return HAL_GPIO_ReadPin(enc_cs_left_GPIO_Port, enc_cs_left_Pin) == GPIO_PIN_SET;
}

static int right_cs_released(void)
{
    return HAL_GPIO_ReadPin(enc_cs_right_GPIO_Port, enc_cs_right_Pin) == GPIO_PIN_SET;
}

static void fresh_board(void)
{
    host_spi_reset();
    host_release_all();
    enc_init();
}

/* Answer the chain in flight: words in wire order, knee first. */
static void reply_chain(uint16_t knee_word, uint16_t hip_word)
{
    uint16_t w[2] = { knee_word, hip_word };
    host_spi_reply(w, 2u);
}

/* A whole tick: start, left chain answers, right chain answers. */
static void one_read(uint16_t l_hip, uint16_t l_knee, uint16_t r_hip, uint16_t r_knee)
{
    enc_start_read();
    reply_chain(l_knee, l_hip);
    reply_chain(r_knee, r_hip);
}

static void one_good_read(uint16_t l_hip, uint16_t l_knee, uint16_t r_hip, uint16_t r_knee)
{
    one_read(good_word(l_hip), good_word(l_knee), good_word(r_hip), good_word(r_knee));
}

/* ===================================================================== */

static void test_a_clean_read_goes_left_then_right(void)
{
    printf("a clean read: left chain, then right chain\n");

    fresh_board();

    CHECK(left_cs_released() && right_cs_released(),
          "a talk-to-me wire was held down before any read started");

    enc_start_read();

    CHECK(!left_cs_released(), "the left chain was not selected first");
    CHECK(right_cs_released(), "both chains were selected at once");
    CHECK(host_spi_busy() && host_spi_started() == 1u, "no read was started");
    CHECK(host_spi_sent_word(0) == READ_ANGLE && host_spi_sent_word(1) == READ_ANGLE,
          "the left chain was asked 0x%04X 0x%04X, expected two 0xFFFF",
          host_spi_sent_word(0), host_spi_sent_word(1));

    reply_chain(good_word(0x0002u), good_word(0x0001u));        /* left knee, left hip */

    CHECK(left_cs_released(), "the left chain was still selected after it answered");
    CHECK(!right_cs_released(), "the right chain was not read straight after the left");
    CHECK(host_spi_started() == 2u, "the right chain's read was not started");

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == 0u,
          "readings were published halfway through the tick (mask 0x%X)", valid);

    reply_chain(good_word(0x3FFFu), good_word(0x1234u));        /* right knee, right hip */

    CHECK(left_cs_released() && right_cs_released(),
          "a talk-to-me wire was left held down after the read finished");
    CHECK(!host_spi_busy(), "a third transfer was started");

    enc_get(angle, &valid);
    CHECK(valid == ALL_VALID,
          "%u of the four sensors were accepted (mask 0x%X), expected all four",
          (unsigned)__builtin_popcount(valid), valid);
    CHECK(angle[NEXUS_ENC_L_HIP_PITCH] == 0x0001u && angle[NEXUS_ENC_L_KNEE_PITCH] == 0x0002u &&
              angle[NEXUS_ENC_R_HIP_PITCH] == 0x1234u && angle[NEXUS_ENC_R_KNEE_PITCH] == 0x3FFFu,
          "the angles landed as L hip %u, L knee %u, R hip %u, R knee %u",
          angle[NEXUS_ENC_L_HIP_PITCH], angle[NEXUS_ENC_L_KNEE_PITCH],
          angle[NEXUS_ENC_R_HIP_PITCH], angle[NEXUS_ENC_R_KNEE_PITCH]);
}

static void test_a_corrupted_word_is_refused(void)
{
    printf("a word damaged on the way back\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    /* The right hip's word arrives with a bit flipped. */
    one_read(good_word(101u), good_word(201u), corrupted(good_word(301u)), good_word(401u));

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK((valid & BIT(NEXUS_ENC_R_HIP_PITCH)) == 0u,
          "a damaged word was accepted as a good reading");
    CHECK(valid == (ALL_VALID & (uint8_t)~BIT(NEXUS_ENC_R_HIP_PITCH)),
          "the other three sensors were affected too (mask 0x%X)", valid);

    /* The damaged sensor keeps its LAST GOOD angle rather than rubbish. */
    CHECK(angle[NEXUS_ENC_R_HIP_PITCH] == 300u,
          "right hip became %u after a damaged word; expected its last good value, 300",
          angle[NEXUS_ENC_R_HIP_PITCH]);
    CHECK(angle[NEXUS_ENC_L_HIP_PITCH] == 101u && angle[NEXUS_ENC_R_KNEE_PITCH] == 401u,
          "the undamaged sensors did not update");
}

static void test_an_unwell_sensor_is_refused_then_cleared(void)
{
    printf("a sensor reporting a problem: refused, its error read, then back\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    /* Tick 1: the left knee sets its error flag. The word is otherwise
       perfectly formed, so this is a separate refusal from a damaged one. */
    one_read(good_word(111u), unwell_word(222u), good_word(333u), good_word(444u));

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK((valid & BIT(NEXUS_ENC_L_KNEE_PITCH)) == 0u,
          "a sensor that reported a problem with itself was believed anyway");
    CHECK(valid == (ALL_VALID & (uint8_t)~BIT(NEXUS_ENC_L_KNEE_PITCH)),
          "the mask was 0x%X - only the left knee should be refused", valid);
    CHECK(angle[NEXUS_ENC_L_KNEE_PITCH] == 200u,
          "the unwell sensor's angle became %u; expected its last good value, 200",
          angle[NEXUS_ENC_L_KNEE_PITCH]);

    /* Tick 2: the knee is asked for its error register, which is what clears
       the flag. Its hip neighbour is still asked for the angle. */
    enc_start_read();
    CHECK(host_spi_sent_word(0) == READ_ERRFL,
          "the left knee was asked 0x%04X, expected an error-register read 0x4001",
          host_spi_sent_word(0));
    CHECK(host_spi_sent_word(1) == READ_ANGLE,
          "the left hip was asked 0x%04X; only the unwell sensor should change",
          host_spi_sent_word(1));

    /* The reply is the error register - a clean word, but NOT an angle. */
    reply_chain(good_word(0x0004u), good_word(112u));
    reply_chain(good_word(445u), good_word(334u));

    enc_get(angle, &valid);
    CHECK((valid & BIT(NEXUS_ENC_L_KNEE_PITCH)) == 0u && angle[NEXUS_ENC_L_KNEE_PITCH] == 200u,
          "the error register's contents were taken as a knee angle (%u)",
          angle[NEXUS_ENC_L_KNEE_PITCH]);
    CHECK(angle[NEXUS_ENC_L_HIP_PITCH] == 112u, "the left hip stopped updating");

    /* Tick 3: back to the angle, and it is believed again. */
    enc_start_read();
    CHECK(host_spi_sent_word(0) == READ_ANGLE,
          "after reading the error register the knee was asked 0x%04X, not the angle",
          host_spi_sent_word(0));
    reply_chain(good_word(223u), good_word(113u));
    reply_chain(good_word(446u), good_word(335u));

    enc_get(angle, &valid);
    CHECK(valid == ALL_VALID && angle[NEXUS_ENC_L_KNEE_PITCH] == 223u,
          "the knee did not recover (mask 0x%X, angle %u)", valid,
          angle[NEXUS_ENC_L_KNEE_PITCH]);
}

static void test_a_read_that_never_comes_back_is_given_up_on(void)
{
    printf("a read that never comes back\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == ALL_VALID, "the four good readings were not accepted");

    /* The left chain answers; the right chain never does. */
    enc_start_read();
    reply_chain(good_word(201u), good_word(101u));
    CHECK(host_spi_busy() && !right_cs_released(), "the right chain's read was not started");

    uint32_t stalls_before = enc_stalls();

    for (int i = 0; i < TIMEOUT_TICKS - 1; i++)
    {
        enc_start_read();
    }
    CHECK(enc_stalls() == stalls_before,
          "the read was given up on after %d attempts, too early", TIMEOUT_TICKS - 1);

    enc_get(angle, &valid);
    CHECK(valid == ALL_VALID,
          "the readings were thrown away while the read was still plausibly in progress");

    /* The fifth is one too many. */
    enc_start_read();

    CHECK(enc_stalls() == stalls_before + 1u,
          "a read that never came back was not counted as a stall");
    CHECK(host_spi_aborted() == 1u, "the stuck read was not abandoned at the hardware");
    CHECK(left_cs_released() && right_cs_released(),
          "a talk-to-me wire was left held down after giving up");

    /* The readings are marked as not to be trusted - the health check watches
       only that mark. */
    enc_get(angle, &valid);
    CHECK(valid == 0u,
          "after giving up, the readings were still marked good (mask 0x%X)", valid);

    /* Having given up, the next tick starts again from the left chain. */
    uint32_t started_before = host_spi_started();
    enc_start_read();
    CHECK(host_spi_started() == started_before + 1u && !left_cs_released(),
          "no new read of the left chain was started after the stuck one was abandoned");
}

static void test_a_read_that_will_not_start_is_retried(void)
{
    printf("a read the hardware refuses to start\n");

    fresh_board();

    host_spi_refuse(1u);
    enc_start_read();

    CHECK(!host_spi_busy(), "the refused read was recorded as in progress");
    CHECK(left_cs_released() && right_cs_released(),
          "a talk-to-me wire was left held down after a refused read");

    uint32_t started_before = host_spi_started();
    enc_start_read();
    CHECK(host_spi_started() == started_before + 1u, "the next attempt did not start a read");
    CHECK(enc_stalls() == 0u, "a read that never started was counted as a stall");

    /* Now the right chain refuses: the left readings still count, the right
       ones are not valid this tick, and nothing is left hanging. */
    host_spi_refuse(1u);
    reply_chain(good_word(20u), good_word(10u));

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == (BIT(NEXUS_ENC_L_HIP_PITCH) | BIT(NEXUS_ENC_L_KNEE_PITCH)),
          "with the right chain refused, the mask was 0x%X, expected the left two", valid);
    CHECK(!host_spi_busy() && right_cs_released(),
          "the refused right chain was left in progress or selected");

    started_before = host_spi_started();
    enc_start_read();
    CHECK(host_spi_started() == started_before + 1u,
          "the next tick did not start after the right chain was refused");
}

static void test_a_bus_error_gives_up_immediately(void)
{
    printf("an error reported by the bus itself\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    enc_start_read();
    CHECK(host_spi_busy(), "no read was started");

    uint32_t errors_before = enc_errors();

    /* An error on some other bus is not ours. */
    SPI_HandleTypeDef other = { SPI2 };
    HAL_SPI_ErrorCallback(&other);

    CHECK(enc_errors() == errors_before, "an error on a different bus was counted as ours");
    CHECK(host_spi_busy(), "an error on a different bus abandoned our read");

    /* Ours is. It does not wait out the five ticks - the error IS the answer. */
    HAL_SPI_ErrorCallback(&hspi1);

    CHECK(enc_errors() == errors_before + 1u, "the bus error was not counted");
    CHECK(host_spi_aborted() == 1u, "the read was not abandoned");
    CHECK(left_cs_released() && right_cs_released(),
          "a talk-to-me wire was left held down after a bus error");
    CHECK(enc_stalls() == 0u,
          "a bus error was also counted as a stall; they are different things");

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == 0u,
          "after a bus error the readings were still marked good (mask 0x%X)", valid);
}

static void test_the_check_digit_catches_a_single_flipped_bit(void)
{
    printf("the check digit, over every bit position\n");

    /*
     * A check digit computed over the wrong range of bits still looks right
     * for most words - which is why this sweeps rather than sampling. Every
     * single-bit flip, in every position, on a spread of angles, must be
     * refused. (Flipping bit 14 raises the sensor's own error flag instead,
     * which is still a refusal.)
     */
    static const uint16_t angles[] = {
        0x0000u, 0x0001u, 0x0002u, 0x1234u, 0x2AAAu, 0x1555u, 0x3FFFu, 0x00FFu
    };

    int accepted_a_flip = 0;
    int refused_a_clean = 0;

    for (unsigned a = 0; a < sizeof(angles) / sizeof(angles[0]); a++)
    {
        uint16_t clean = good_word(angles[a]);

        for (int bit = 0; bit < 16; bit++)
        {
            uint16_t bad = (uint16_t)(clean ^ (uint16_t)(1u << bit));

            fresh_board();
            one_read(bad, bad, bad, bad);

            uint16_t angle[NEXUS_NUM_ENCODERS];
            uint8_t  valid;
            enc_get(angle, &valid);
            if (valid != 0u)
            {
                accepted_a_flip++;
            }
        }

        fresh_board();
        one_read(clean, clean, clean, clean);

        uint16_t angle[NEXUS_NUM_ENCODERS];
        uint8_t  valid;
        enc_get(angle, &valid);
        if (valid != ALL_VALID)
        {
            refused_a_clean++;
        }
    }

    CHECK(accepted_a_flip == 0, "%d damaged words were accepted as good readings", accepted_a_flip);
    CHECK(refused_a_clean == 0, "%d perfectly good words were refused", refused_a_clean);
}

static void test_the_angles_and_the_mask_describe_the_same_read(void)
{
    printf("angles and their trust mark are handed over together\n");

    fresh_board();
    one_good_read(11u, 22u, 33u, 44u);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK(valid == ALL_VALID && angle[NEXUS_ENC_L_HIP_PITCH] == 11u &&
              angle[NEXUS_ENC_R_KNEE_PITCH] == 44u,
          "the first read did not come back intact");

    /* Reading again does not consume anything. */
    uint16_t again[NEXUS_NUM_ENCODERS];
    uint8_t  valid_again;
    enc_get(again, &valid_again);

    CHECK(valid_again == valid && memcmp(again, angle, sizeof(angle)) == 0,
          "reading the sensors twice gave two different answers");

    /* An angle is fourteen bits. Nothing above that reaches a caller. */
    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        CHECK((angle[i] & ~ANGLE_MASK) == 0u,
              "sensor %d handed back 0x%04X, which has bits above the angle in it", i, angle[i]);
    }
}

int main(void)
{
    printf("enc_as5047p.c host tests\n");
    printf("------------------------\n");

    test_a_clean_read_goes_left_then_right();
    test_a_corrupted_word_is_refused();
    test_an_unwell_sensor_is_refused_then_cleared();
    test_a_read_that_never_comes_back_is_given_up_on();
    test_a_read_that_will_not_start_is_retried();
    test_a_bus_error_gives_up_immediately();
    test_the_check_digit_catches_a_single_flipped_bit();
    test_the_angles_and_the_mask_describe_the_same_read();

    printf("------------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
