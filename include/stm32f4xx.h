#ifndef STM32F4XX_H
#define STM32F4XX_H

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define USB_OTG_FS_PERIPH_BASE (0x50000000UL)
#define USB_OTG_HS_PERIPH_BASE (0x40040000UL)

#define OTG_FS_IRQn NVIC_OTG_FS_IRQ
#define OTG_HS_IRQn NVIC_OTG_HS_IRQ
    typedef int IRQn_Type;

#define USB_OTG_FS_MAX_IN_ENDPOINTS 4
#define USB_OTG_FS_TOTAL_FIFO_SIZE 1280
#define USB_OTG_HS_MAX_IN_ENDPOINTS 6
#define USB_OTG_HS_TOTAL_FIFO_SIZE 4096

    static inline void NVIC_EnableIRQ(int irq) { nvic_enable_irq(irq); }
    static inline void NVIC_DisableIRQ(int irq) { nvic_disable_irq(irq); }
#ifndef __NOP
#define __NOP() __asm__ volatile("nop")
#endif

#define SystemCoreClock 168000000U

#ifndef RCC
    typedef struct
    {
        uint32_t dummy[20];
        uint32_t AHB1LPENR;
    } RCC_TypeDef;
#define RCC ((RCC_TypeDef *)0x40023800UL)
#endif
#define RCC_AHB1LPENR_OTGHSULPILPEN (1 << 30)

#ifdef __cplusplus
}
#endif

#endif
