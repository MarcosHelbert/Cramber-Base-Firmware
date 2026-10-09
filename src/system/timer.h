#ifndef TIMER_H
#define TIMER_H

#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/dwt.h>

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

inline void dwt_init()
{
    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    DWT_CYCCNT = 0u;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

inline uint32_t dwt_get_cycles()
{
    return DWT_CYCCNT;
}

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
