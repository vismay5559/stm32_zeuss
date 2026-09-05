/*
 * Host test for enc_as5048a.c - the four sensors that measure how far each
 * leg spring has been squashed.
 *
 * That measurement is how the robot knows how hard a foot is pushing on the
 * ground, so a wrong reading accepted as good is a robot that thinks it is
 * standing on something when it is not. Every sensor word carries its own
 * check digit and its own "I am unwell" flag, and the whole point of this
 * file is that both are honoured.
 *
 * The other half is what happens when a reading never arrives. The sensors
 * are read by the hardware on its own, in the background, and the driver is
 * told afterwards. If that "afterwards" never comes - a bus glitch, an
 * aborted transfer - the driver used to wait forever, handing out its last
 * four readings as though they were current. Nothing noticed, because the
 * only thing watching was the same flag that had gone stale.
 *
 * Vocabulary:
 *
 *   word       the sixteen bits one sensor sends back. Fourteen are the
 *              angle, one says the sensor is unhappy, one is the check digit.
 *   check digit  a single bit chosen so the number of 1s in the word is even.
 *              Flip any one bit in transit and the count goes odd, which is
 *              how a corrupted word is spotted.
 *   CS         the wire that says "I am talking to you now". Held low for the
 *              duration of a read and released afterwards.
 *   in flight  a read that has been started but not yet answered.
 */

#include "enc_as5048a.h"
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

#define ERROR_FLAG   0x4000u
#define ANGLE_MASK   0x3FFFu
#define TIMEOUT_TICKS 5      /* must match ENC_XFER_TIMEOUT_TICKS */

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

/* A word the sensor would never send: one bit of it flipped in transit. */
static uint16_t corrupted(uint16_t word)
{
    return (uint16_t)(word ^ 0x0004u);
}

static int cs_is_released(void)
{
    return HAL_GPIO_ReadPin(enc_cs_GPIO_Port, enc_cs_Pin) == GPIO_PIN_SET;
}

static void fresh_board(void)
{
    host_spi_reset();
    host_release_all();
    enc_init();
}

/* Start a read and answer it with four good angles. */
static void one_good_read(uint16_t a0, uint16_t a1, uint16_t a2, uint16_t a3)
{
    uint16_t w[NEXUS_NUM_ENCODERS] = {
        good_word(a0), good_word(a1), good_word(a2), good_word(a3)
    };

    enc_start_read();
    host_spi_reply(w, NEXUS_NUM_ENCODERS);
}

/* ===================================================================== */

static void test_a_clean_read_gives_four_angles(void)
{
    printf("a clean read\n");

    fresh_board();

    CHECK(cs_is_released(),
          "the talk-to-me wire was left held down before any read started");

    enc_start_read();

    CHECK(!cs_is_released(),
          "the talk-to-me wire was not held down during a read");
    CHECK(host_spi_busy(),
          "no read was actually started");
    CHECK(host_spi_sent_word(0) == 0xFFFFu,
          "the read command sent was 0x%04X, expected 0xFFFF",
          host_spi_sent_word(0));

    uint16_t w[NEXUS_NUM_ENCODERS] = {
        good_word(0x0001u), good_word(0x1234u),
        good_word(0x3FFFu), good_word(0x0000u)
    };
    host_spi_reply(w, NEXUS_NUM_ENCODERS);

    CHECK(cs_is_released(),
          "the talk-to-me wire was still held down after the read finished");

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK(valid == 0x0Fu,
          "%u of the four sensors were accepted (mask 0x%X), expected all four",
          (unsigned)__builtin_popcount(valid), valid);
    CHECK(angle[0] == 0x0001u && angle[1] == 0x1234u &&
              angle[2] == 0x3FFFu && angle[3] == 0x0000u,
          "the angles came back as %u %u %u %u",
          angle[0], angle[1], angle[2], angle[3]);
}

static void test_a_corrupted_word_is_refused(void)
{
    printf("a word damaged on the way back\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    /* Sensor 2's word arrives with a bit flipped. */
    uint16_t w[NEXUS_NUM_ENCODERS] = {
        good_word(101u), good_word(201u),
        corrupted(good_word(301u)), good_word(401u)
    };

    enc_start_read();
    host_spi_reply(w, NEXUS_NUM_ENCODERS);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK((valid & (1u << 2)) == 0u,
          "a damaged word was accepted as a good reading");
    CHECK(valid == 0x0Bu,
          "the other three sensors were affected too (mask 0x%X, expected 0xB)",
          valid);

    /*
     * And the damaged sensor keeps its LAST GOOD angle rather than being
     * overwritten with rubbish. It is marked as not to be trusted either way,
     * but a reader that ignores the mark gets the older true value instead of
     * a corrupted one.
     */
    CHECK(angle[2] == 300u,
          "sensor 2's angle became %u after a damaged word; expected its last "
          "good value, 300", angle[2]);
    CHECK(angle[0] == 101u && angle[3] == 401u,
          "the undamaged sensors did not update");
}

static void test_a_sensor_that_says_it_is_unwell_is_refused(void)
{
    printf("a sensor reporting a problem with itself\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    /* Sensor 1 sets its own error flag. The word is otherwise perfectly
       formed - the check digit is correct - so this is a separate refusal
       from a damaged one, and it has to be honoured separately. */
    uint16_t w[NEXUS_NUM_ENCODERS] = {
        good_word(111u), with_check_digit((uint16_t)(ERROR_FLAG | 222u)),
        good_word(333u), good_word(444u)
    };

    enc_start_read();
    host_spi_reply(w, NEXUS_NUM_ENCODERS);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK((valid & (1u << 1)) == 0u,
          "a sensor that reported a problem with itself was believed anyway");
    CHECK(valid == 0x0Du,
          "the mask was 0x%X, expected 0xD - only sensor 1 refused", valid);
    CHECK(angle[1] == 200u,
          "the unwell sensor's angle became %u; expected its last good value, "
          "200", angle[1]);
}

static void test_a_read_that_never_comes_back_is_given_up_on(void)
{
    printf("a read that never comes back\n");

    fresh_board();
    one_good_read(100u, 200u, 300u, 400u);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == 0x0Fu, "the four good readings were not accepted");

    /* Start a read and simply never answer it. */
    enc_start_read();
    CHECK(host_spi_busy(), "no read was started");

    uint32_t stalls_before = enc_stalls();

    /* Four more attempts: still waiting, nothing given up on yet. */
    for (int i = 0; i < TIMEOUT_TICKS - 1; i++)
    {
        enc_start_read();
    }
    CHECK(enc_stalls() == stalls_before,
          "the read was given up on after %d attempts, too early",
          TIMEOUT_TICKS - 1);

    enc_get(angle, &valid);
    CHECK(valid == 0x0Fu,
          "the readings were thrown away while the read was still plausibly "
          "in progress");

    /* The fifth is one too many. */
    enc_start_read();

    CHECK(enc_stalls() == stalls_before + 1u,
          "a read that never came back was not counted as a stall");
    CHECK(host_spi_aborted() == 1u,
          "the stuck read was not abandoned at the hardware");
    CHECK(cs_is_released(),
          "the talk-to-me wire was left held down after giving up");

    /*
     * And - this is the part that was wrong once - the readings are marked as
     * not to be trusted. Leaving the mark set hands out four fossils forever,
     * and the health check, which watches only that mark, would go on
     * reporting the sensors as fine.
     */
    enc_get(angle, &valid);
    CHECK(valid == 0u,
          "after giving up, the readings were still marked good (mask 0x%X)",
          valid);

    /* Having given up, it starts a fresh read next time rather than staying
       stuck. */
    uint32_t started_before = host_spi_started();
    enc_start_read();
    CHECK(host_spi_started() == started_before + 1u,
          "no new read was started after the stuck one was abandoned");
}

static void test_a_read_that_will_not_start_is_retried(void)
{
    printf("a read the hardware refuses to start\n");

    fresh_board();

    host_spi_refuse(1u);
    enc_start_read();

    CHECK(!host_spi_busy(), "the refused read was recorded as in progress");
    CHECK(cs_is_released(),
          "the talk-to-me wire was left held down after a refused read");

    /*
     * The next attempt has to actually try again. Leaving the driver
     * believing a read is in flight would cost five ticks before anything
     * moved, every single time.
     */
    uint32_t started_before = host_spi_started();
    enc_start_read();
    CHECK(host_spi_started() == started_before + 1u,
          "the next attempt did not start a read");
    CHECK(enc_stalls() == 0u,
          "a read that never started was counted as a stall");
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

    CHECK(enc_errors() == errors_before,
          "an error on a different bus was counted as ours");
    CHECK(host_spi_busy(),
          "an error on a different bus abandoned our read");

    /* Ours is. It does not wait out the five ticks - the error IS the
       answer. */
    HAL_SPI_ErrorCallback(&hspi1);

    CHECK(enc_errors() == errors_before + 1u,
          "the bus error was not counted");
    CHECK(host_spi_aborted() == 1u, "the read was not abandoned");
    CHECK(cs_is_released(),
          "the talk-to-me wire was left held down after a bus error");
    CHECK(enc_stalls() == 0u,
          "a bus error was also counted as a stall; they are different things");

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);
    CHECK(valid == 0u,
          "after a bus error the readings were still marked good (mask 0x%X)",
          valid);
}

static void test_the_check_digit_catches_a_single_flipped_bit(void)
{
    printf("the check digit, over every bit position\n");

    /*
     * A check digit that is computed over the wrong range of bits still looks
     * right for most words - which is why this sweeps rather than sampling.
     * Every single-bit flip, in every position, on a spread of angles, must
     * be refused.
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
            uint16_t w[NEXUS_NUM_ENCODERS];
            for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
            {
                w[i] = (uint16_t)(clean ^ (uint16_t)(1u << bit));
            }

            fresh_board();
            enc_start_read();
            host_spi_reply(w, NEXUS_NUM_ENCODERS);

            uint16_t angle[NEXUS_NUM_ENCODERS];
            uint8_t  valid;
            enc_get(angle, &valid);

            /*
             * Flipping bit 14 sets the sensor's own error flag instead of
             * corrupting the number, so it is refused for that reason - which
             * is still a refusal.
             */
            if (valid != 0u)
            {
                accepted_a_flip++;
            }
        }

        /* The clean word itself must of course be accepted. */
        uint16_t w[NEXUS_NUM_ENCODERS] = { clean, clean, clean, clean };
        fresh_board();
        enc_start_read();
        host_spi_reply(w, NEXUS_NUM_ENCODERS);

        uint16_t angle[NEXUS_NUM_ENCODERS];
        uint8_t  valid;
        enc_get(angle, &valid);
        if (valid != 0x0Fu)
        {
            refused_a_clean++;
        }
    }

    CHECK(accepted_a_flip == 0,
          "%d damaged words were accepted as good readings", accepted_a_flip);
    CHECK(refused_a_clean == 0,
          "%d perfectly good words were refused", refused_a_clean);
}

static void test_the_angles_and_the_mask_describe_the_same_read(void)
{
    printf("angles and their trust mark are handed over together\n");

    fresh_board();
    one_good_read(11u, 22u, 33u, 44u);

    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    CHECK(valid == 0x0Fu && angle[0] == 11u && angle[3] == 44u,
          "the first read did not come back intact");

    /* Reading again does not consume anything - unlike the motor telemetry,
       these are a current value, not a one-shot delivery. */
    uint16_t again[NEXUS_NUM_ENCODERS];
    uint8_t  valid_again;
    enc_get(again, &valid_again);

    CHECK(valid_again == valid && memcmp(again, angle, sizeof(angle)) == 0,
          "reading the sensors twice gave two different answers");

    /* An angle is fourteen bits. Nothing above that reaches a caller, even
       though the sensor's word is sixteen bits wide. */
    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        CHECK((angle[i] & ~ANGLE_MASK) == 0u,
              "sensor %d handed back 0x%04X, which has bits above the angle "
              "in it", i, angle[i]);
    }
}

int main(void)
{
    printf("enc_as5048a.c host tests\n");
    printf("------------------------\n");

    test_a_clean_read_gives_four_angles();
    test_a_corrupted_word_is_refused();
    test_a_sensor_that_says_it_is_unwell_is_refused();
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
