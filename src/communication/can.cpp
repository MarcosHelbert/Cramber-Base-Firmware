#include "can.h"

#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

struct CanTiming
{
    uint32_t bitrate;
    uint32_t prescaler;
    uint32_t sjw;
    uint32_t ts1;
    uint32_t ts2;
};

static constexpr CanTiming CAN_TIMINGS[] = {
    {1000000, 3, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_10TQ, CAN_BTR_TS2_3TQ},
    {500000, 6, CAN_BTR_SJW_1TQ, CAN_BTR_TS1_10TQ, CAN_BTR_TS2_3TQ},
};

CAN::CAN()
{
}

void CAN::init(uint32_t bitrate_bps)
{
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);

    can_reset(CAN1);

    const CanTiming *timing = &CAN_TIMINGS[0];
    for (const auto &t : CAN_TIMINGS)
    {
        if (t.bitrate == bitrate_bps)
        {
            timing = &t;
            break;
        }
    }

    can_init(
        CAN1,
        false, // TTCM
        true,  // ABOM
        false, // AWUM
        false, // NART
        false, // RFLM
        false, // TXFP
        timing->sjw,
        timing->ts1,
        timing->ts2,
        timing->prescaler,
        false, // loopback off
        false  // silent off
    );

    can_filter_id_mask_32bit_init(0, 0, 0, 0, true);
}