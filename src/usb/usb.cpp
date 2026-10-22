#include "usb.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include <tusb.h>

#include <debug.h>
#include <utils.h>

USB::USB() {}

void USB::setAxis(float position_rad, float steering_range_deg)
{
    if (steering_range_deg <= 0.0f)
    {
        joystickReport_.x_axis = 0;
        return;
    }

    float half_range_rad = angleToRadians(steering_range_deg * 0.5f);
    float normalized = position_rad / half_range_rad; // -1..1 across the full lock-to-lock range

    joystickReport_.x_axis = float_to_i16(normalized);
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
}

void USB::start()
{
    tusb_init();

    for (volatile int i = 0; i < 200000; ++i)
        __asm__ volatile("nop");

    OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
}

void USB::handleEvents()
{
    tud_task_ext(0, false);
}

void USB::sendReports()
{
#ifdef DEBUG
    cdcLogFlush();
#endif

    if (tud_mounted() && tud_hid_n_ready(JOYSTICK_ITF))
    {
        tud_hid_n_report(JOYSTICK_ITF, JOYSTICK_REPORT_ID, &joystickReport_, sizeof(joystickReport_));
    }
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