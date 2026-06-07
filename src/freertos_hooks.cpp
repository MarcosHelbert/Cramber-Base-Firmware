#include <FreeRTOS.h>
#include <task.h>

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    __asm__("bkpt 0");

    while (1)
        ;
}