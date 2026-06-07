#ifndef ADC_H
#define ADC_H

#include <cstdint>

class ADC
{
public:
    enum Channel
    {
        VBUS = 0,
        AUX_TEMP,
        MOTOR_TEMP,
        COUNT
    };

    enum InjectedChannel
    {
        IB = 0,
        IC,
        INJ_COUNT
    };

    ADC() = default;

    static void init();
    static uint16_t get(Channel ch);
    static uint16_t getInjected(InjectedChannel ch);

private:
    static volatile uint16_t adc_buffer[Channel::COUNT];
};

#endif // ADC_H