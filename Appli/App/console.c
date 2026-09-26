#include "console.h"
#include "main.h"
#include "stm32h7rsxx_nucleo.h"

/*
 * A power-of-two ring, so the wrap is a mask rather than a branch.
 *
 * 4 KB holds several seconds of status output at the rate the leg test prints,
 * which is enough to absorb the bursts that matter: a boot banner, a fault
 * dump, a scan report. Sustained output faster than 115200 will still overrun
 * it, and should - the answer to that is to print less, not to buffer more.
 *
 * Plain cached RAM is correct here. Only the CPU touches this buffer: the
 * producer is whoever called printf and the consumer is the UART interrupt on
 * the same core. No peripheral masters it, so none of dma_buffer.h applies.
 */
#define CONSOLE_BUF       4096u
#define CONSOLE_MASK      (CONSOLE_BUF - 1u)

static uint8_t           s_buf[CONSOLE_BUF];
static volatile uint32_t s_head;      /* written by printf     */
static volatile uint32_t s_tail;      /* written by the ISR    */
static volatile uint32_t s_dropped;
static volatile uint8_t  s_ready;

extern UART_HandleTypeDef hcom_uart[COMn];

static inline USART_TypeDef *console_uart(void)
{
    return hcom_uart[COM1].Instance;
}

void console_init(void)
{
    s_head = s_tail = 0;
    s_dropped = 0;

    HAL_NVIC_SetPriority(USART3_IRQn, 14, 0);   /* well below the control loop */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    s_ready = 1u;
}

uint32_t console_dropped(void)
{
    return s_dropped;
}

/*
 * One producer (printf, from thread context) and one consumer (the ISR), so
 * this needs no critical section: each side advances only its own index, and
 * a single aligned 32-bit load or store cannot tear. The ISR may run between
 * any two lines here; the worst it can do is drain a byte we have not counted
 * yet, which is harmless.
 */
int __io_putchar(int ch)
{
    const uint32_t head = s_head;
    const uint32_t next = (head + 1u) & CONSOLE_MASK;

    if (!s_ready)
    {
        /* Before console_init(): anything printed during early boot still has
           to appear, and nothing is time-critical yet, so send it the slow
           way rather than dropping it on the floor. */
        USART_TypeDef *u = console_uart();
        while ((u->ISR & USART_ISR_TXE_TXFNF) == 0u)
        {
        }
        u->TDR = (uint8_t)ch;
        return ch;
    }

    if (next == s_tail)
    {
        s_dropped++;                 /* full: drop, never block */
        return ch;
    }

    s_buf[head] = (uint8_t)ch;
    s_head = next;

    /* Kick the drain. Setting TXEIE when it is already set is a no-op. */
    console_uart()->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return ch;
}

void USART3_IRQHandler(void)
{
    USART_TypeDef *u = console_uart();

    while (((u->ISR & USART_ISR_TXE_TXFNF) != 0u) &&
           ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u))
    {
        const uint32_t tail = s_tail;
        if (tail == s_head)
        {
            /* Nothing left; stop asking to be interrupted or this spins
               forever the moment the buffer empties. */
            u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
            break;
        }
        u->TDR = s_buf[tail];
        s_tail = (tail + 1u) & CONSOLE_MASK;
    }
}

void console_flush(void)
{
    USART_TypeDef *u = console_uart();
    while (s_tail != s_head)
    {
    }
    while ((u->ISR & USART_ISR_TC) == 0u)
    {
    }
}
