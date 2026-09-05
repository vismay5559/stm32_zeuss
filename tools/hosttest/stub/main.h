#ifndef HOSTTEST_MAIN_H
#define HOSTTEST_MAIN_H

/*
 * Host stand-in for Appli/Core/Inc/main.h.
 *
 * The App layer is written against the HAL, but most of it is pure logic that
 * only touches the HAL at its edges. This header supplies just enough of that
 * edge - GPIO reads, pin names - to compile those files on a workstation and
 * run them under a test, with no board attached.
 *
 * The pin names below MUST match Appli/Core/Inc/main.h. They are CubeMX's
 * output; if the .ioc labels change, this file changes with it.
 */

#include <stdint.h>

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { int id; } GPIO_TypeDef;

extern GPIO_TypeDef host_gpioe;
extern GPIO_TypeDef host_gpiof;

#define GPIOE  (&host_gpioe)
#define GPIOF  (&host_gpiof)

#define GPIO_PIN_1  0x0002u
#define GPIO_PIN_2  0x0004u
#define GPIO_PIN_3  0x0008u
#define GPIO_PIN_4  0x0010u
#define GPIO_PIN_5  0x0020u

#define L_TOE_Pin        GPIO_PIN_2
#define L_TOE_GPIO_Port  GPIOE
#define L_HEEL_Pin       GPIO_PIN_3
#define L_HEEL_GPIO_Port GPIOE
#define R_TOE_Pin        GPIO_PIN_4
#define R_TOE_GPIO_Port  GPIOE
#define R_HEEL_Pin       GPIO_PIN_5
#define R_HEEL_GPIO_Port GPIOE
#define enc_cs_Pin       GPIO_PIN_1
#define enc_cs_GPIO_Port GPIOF

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState v);

/* --- IWDG and RCC, as plain RAM ---------------------------------------
 *
 * Enough of the CMSIS layout for watchdog.c to compile and run here. The
 * register NAMES must match Drivers/CMSIS/.../stm32h7s3xx.h - that part is
 * checked by grepping the real header, not by this stub - but the sequencing
 * (unlock, write, wait for the status bit, reload) is real logic and this is
 * where it gets executed.
 */

typedef struct
{
    volatile uint32_t KR;
    volatile uint32_t PR;
    volatile uint32_t RLR;
    volatile uint32_t SR;
    volatile uint32_t WINR;
    volatile uint32_t EWCR;
} IWDG_TypeDef;

typedef struct
{
    volatile uint32_t RSR;
} RCC_TypeDef;

extern IWDG_TypeDef host_iwdg;
extern RCC_TypeDef  host_rcc;

#define IWDG  (&host_iwdg)
#define RCC   (&host_rcc)

#define IWDG_SR_PVU        (1u << 0)
#define IWDG_SR_RVU        (1u << 1)
#define RCC_RSR_RMVF       (1u << 16)
#define RCC_RSR_IWDGRSTF   (1u << 26)

/* Make the status bits stay set, so the bounded-spin path can be tested. */
void host_iwdg_stick_sr(uint32_t mask);

/* --- test-side control of the simulated pins ------------------------- */

/*
 * The switches are wired to ground with pull-ups, so a PRESSED switch reads
 * LOW. host_press() takes the physical sense ("is this switch pressed") and
 * does the inversion, so tests never have to think about it.
 */
void host_press(GPIO_TypeDef *port, uint16_t pin, int pressed);
void host_release_all(void);


/*
 * Interrupt intrinsics. On the board these disable interrupts around a
 * multi-byte copy so an ISR cannot tear it in half. A host test is
 * single-threaded with no ISRs, so the guard has nothing to protect against
 * and these do nothing - but critical.h still has to compile, and the code
 * under test still has to call them in the right order.
 */
static inline uint32_t __get_PRIMASK(void)          { return 0u; }
static inline void     __set_PRIMASK(uint32_t p)    { (void)p; }
static inline void     __disable_irq(void)          { }
static inline void     __enable_irq(void)           { }

#endif /* HOSTTEST_MAIN_H */
