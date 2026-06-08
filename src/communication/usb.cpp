#include "usb.h"

#include <algorithm>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include <tusb.h>

#include <system_state.h>
#include <hardware/timer.h>
#include <config.h>
#include <debug.h>

USB::USB()
{
}

void USB::init()
{
    rcc_periph_reset_pulse(RST_OTGFS);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_OTGFS);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    OTG_FS_GCCFG |= OTG_GCCFG_PWRDWN;
    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
    OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSASEN | OTG_GCCFG_VBUSBSEN);

    OTG_FS_DCTL |= OTG_DCTL_SDIS;

    nvic_set_priority(NVIC_OTG_FS_IRQ, 6 << 4);
    nvic_enable_irq(NVIC_OTG_FS_IRQ);

    tusb_init();

    for (volatile int i = 0; i < 200000; ++i)
        __asm__ volatile("nop");

    OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
}

void USB::update()
{
    tud_task();

#ifdef DEBUG
    cdcLogFlush();
#endif

    if (tud_mounted() && tud_hid_n_ready(JOYSTICK_ITF))
    {
        tud_hid_n_report(JOYSTICK_ITF, JOYSTICK_REPORT_ID, &joystickReport_, sizeof(joystickReport_));
    }
}

void USB::updateAxis(int32_t positionCounts)
{
    int16_t scaledPosition = scale_position_to_axis_(positionCounts);
    joystickReport_.x_axis = scaledPosition;
}

int16_t USB::scale_position_to_axis_(int32_t positionCounts) const
{
    float max_degrees = g_state.steering_range_deg / 2.0f;
    int32_t max_counts = static_cast<int32_t>(max_degrees * (ENCODER_CPR / 360.0f));

    if (max_counts == 0)
        return 0;

    int32_t relative_pos = positionCounts - g_state.mechanical_offset;
    relative_pos = std::clamp(relative_pos, -max_counts, max_counts);
    int32_t scaled = (int32_t)((int64_t)relative_pos * 32767 / max_counts);

    return static_cast<int16_t>(scaled);
}

extern "C" void otg_fs_isr(void)
{
    tud_int_handler(0);
}

#ifdef DEBUG
extern "C" void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    tud_cdc_read_flush();
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)dtr;
    (void)rts;
}

extern "C" void tud_cdc_tx_complete_cb(uint8_t itf)
{
    (void)itf;
}
#endif