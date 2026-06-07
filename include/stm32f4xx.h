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

// 1. Endereços Base Reais do STM32F405 (Hardware Fixo)
#define USB_OTG_FS_PERIPH_BASE (0x50000000UL)
#define USB_OTG_HS_PERIPH_BASE (0x40040000UL)

// 2. Mapeamento de Interrupções para libopencm3
#define OTG_FS_IRQn NVIC_OTG_FS_IRQ
#define OTG_HS_IRQn NVIC_OTG_HS_IRQ
    typedef int IRQn_Type;

// 3. Macros que a TinyUSB exige para compilar o driver DWC2
#define USB_OTG_FS_MAX_IN_ENDPOINTS 4
#define USB_OTG_FS_TOTAL_FIFO_SIZE 1280
#define USB_OTG_HS_MAX_IN_ENDPOINTS 6
#define USB_OTG_HS_TOTAL_FIFO_SIZE 4096

    // 4. Tradução de funções NVIC
    static inline void NVIC_EnableIRQ(int irq) { nvic_enable_irq(irq); }
    static inline void NVIC_DisableIRQ(int irq) { nvic_disable_irq(irq); }
#ifndef __NOP
#define __NOP() __asm__ volatile("nop")
#endif

// 5. Relógio do Sistema (essencial para os timeouts do driver)
#define SystemCoreClock 168000000U

// 6. Gambiarra para satisfazer o acesso direto da TinyUSB ao RCC
// Isso evita o erro "RCC undeclared" e "no member named AHB1LPENR"
#ifndef RCC
    typedef struct
    {
        uint32_t dummy[20]; // pula os registros iniciais ate AHB1LPENR (offset 0x50)
        uint32_t AHB1LPENR;
    } RCC_TypeDef;
// Aponta para o endereço real do RCC no STM32F4
#define RCC ((RCC_TypeDef *)0x40023800UL)
#endif
#define RCC_AHB1LPENR_OTGHSULPILPEN (1 << 30)

#ifdef __cplusplus
}
#endif

#endif
