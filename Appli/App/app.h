#ifndef APP_H
#define APP_H

#include <stdint.h>

void app_init(void);
void app_run(void);
void app_on_tick(void);

/* Number of 1 kHz ticks missed because a cycle ran long. Should stay at 0. */
uint32_t app_overruns(void);

#endif /* APP_H */
