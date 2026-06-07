#include "adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>

volatile uint16_t ADC::adc_buffer[ADC::Channel::COUNT];

void ADC::init()
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_DMA2);
    rcc_periph_clock_enable(RCC_ADC1);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6); // VBUS (IN6)
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5); // AUX_TEMP (IN5)
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5); // M0_TEMP (IN15)
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); // IB (IN10)
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1); // IC (IN11)

    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA2, DMA_STREAM0, COUNT);
    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_16BIT);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);
    dma_enable_stream(DMA2, DMA_STREAM0);

    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY4);
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_enable_scan_mode(ADC1);
    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);

    uint8_t regular_channels[] = {6, 5, 15}; // VBUS(6), AUX_TEMP(5), M0_TEMP(15)
    adc_set_regular_sequence(ADC1, COUNT, regular_channels);
    adc_set_continuous_conversion_mode(ADC1);

    uint8_t injected_channels[] = {10, 11}; // IB, IC
    adc_set_injected_sequence(ADC1, INJ_COUNT, injected_channels);

    adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_TIM1_CC4, ADC_CR2_JEXTEN_RISING_EDGE);

    adc_power_on(ADC1);

    ADC_SR(ADC1) = 0;

    adc_start_conversion_regular(ADC1);
}

uint16_t ADC::get(Channel ch)
{
    return adc_buffer[ch];
}

uint16_t ADC::getInjected(InjectedChannel ch)
{
    switch (ch)
    {
    case InjectedChannel::IB:
        return ADC_JDR1(ADC1);
    case InjectedChannel::IC:
        return ADC_JDR2(ADC1);
    default:
        return 0;
    }
}