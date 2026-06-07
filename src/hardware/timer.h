#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "dwt.h"

inline uint32_t millis(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    return 0;
}

inline uint32_t micros()
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        uint32_t ticks;
        uint32_t cycles;

        vTaskSuspendAll();
        ticks = xTaskGetTickCount();
        cycles = dwt_get_cycles();
        xTaskResumeAll();

        return (ticks * 1000) + ((cycles % (168 * 1000)) / 168);
    }
    return dwt_get_cycles() / 168;
}

inline void delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        uint32_t start = micros();
        while ((micros() - start) < (ms * 1000))
            ;
    }
}

inline void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((micros() - start) < us)
        ;
}

#endif // TIMER_H