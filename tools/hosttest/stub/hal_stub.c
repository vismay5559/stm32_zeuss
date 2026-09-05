#include "main.h"
#include <string.h>

GPIO_TypeDef host_gpioe = { 'E' };
GPIO_TypeDef host_gpiof = { 'F' };

/* One level bit per (port, pin). Idle HIGH, because the switches pull to
   ground and the MCU pull-ups hold the line high when nothing is pressed. */
static uint32_t s_level_e = 0xFFFFFFFFu;
static uint32_t s_level_f = 0xFFFFFFFFu;

static uint32_t *bank(GPIO_TypeDef *port)
{
    return (port == GPIOF) ? &s_level_f : &s_level_e;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (*bank(port) & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState v)
{
    if (v == GPIO_PIN_SET) { *bank(port) |= pin; }
    else                   { *bank(port) &= ~(uint32_t)pin; }
}

void host_press(GPIO_TypeDef *port, uint16_t pin, int pressed)
{
    /* Pressed == closed to ground == LOW. */
    HAL_GPIO_WritePin(port, pin, pressed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void host_release_all(void)
{
    s_level_e = 0xFFFFFFFFu;
    s_level_f = 0xFFFFFFFFu;
}

/* --- IWDG / RCC ------------------------------------------------------- */

IWDG_TypeDef host_iwdg;
RCC_TypeDef  host_rcc;

/*
 * SR is plain RAM here, so it reads back as 0 and watchdog.c's "wait for the
 * write to land" poll succeeds immediately - the healthy case. A test that
 * wants the other case sets a bit that nothing ever clears, which is exactly
 * what a stopped LSI looks like from the CPU's side.
 */
void host_iwdg_stick_sr(uint32_t mask)
{
    host_iwdg.SR |= mask;
}
