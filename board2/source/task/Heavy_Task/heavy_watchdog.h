#ifndef HEAVY_WATCHDOG_H
#define HEAVY_WATCHDOG_H

void HeavyWatchdog_Init(void);
void HeavyWatchdog_Check(void);
void HeavyWatchdog_Task(void *argument);

#endif /* HEAVY_WATCHDOG_H */
