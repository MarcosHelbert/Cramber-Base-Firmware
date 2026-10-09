#include "drv8301.h"

#include <libopencm3/stm32/rcc.h>

#include <debug.h>

#include <system/timer.h>

volatile uint16_t dbg_s1 = 0;
volatile uint16_t dbg_s2 = 0;

void DRV8301::init()
{
    rcc_periph_clock_enable(RCC_SPI3);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOD);

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11 | GPIO12);
    gpio_set_af(GPIOC, GPIO_AF6, GPIO10 | GPIO11 | GPIO12);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
    gpio_clear(GPIOB, GPIO12);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    gpio_set(GPIOC, GPIO13);

    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO2);

    spi_disable(spi_);
    spi_init_master(spi_, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(spi_);
    spi_set_nss_high(spi_);
    spi_enable(spi_);

    gpio_clear(en_port_, en_pin_);
}

bool DRV8301::enable()
{
    gpio_clear(en_port_, en_pin_);
    delay_us(100);

    gpio_set(en_port_, en_pin_);
    delay_ms(20);

    writeRegister(CONTROL_1, (1 << 2));
    delay_us(100);

    uint16_t gate_current = 0; // 1.7A
    uint16_t gate_reset = 0;
    uint16_t pwm_mode = 0; // 6-PWM
    uint16_t ocp_mode = 1; // Latch shutdown
    uint16_t ocp_adj = 21;

    uint16_t c1 = (ocp_adj << 6) | (ocp_mode << 4) | (pwm_mode << 3) | (gate_reset << 2) | (gate_current << 0);
    writeRegister(CONTROL_1, c1);

    delay_us(100);

    uint16_t octw_mode = 0;
    uint16_t gain = 0;
    uint16_t dc_cal_ch1 = 0;
    uint16_t oc_toff = 0;

    if (config_.current_amp_gain == 10.0f)
        gain = 0;
    else if (config_.current_amp_gain == 20.0f)
        gain = 1;
    else if (config_.current_amp_gain == 40.0f)
        gain = 2;
    else
        gain = 3;

    uint16_t c2 = (oc_toff << 6) | (dc_cal_ch1 << 4) | (gain << 2) | (octw_mode << 0);
    writeRegister(CONTROL_2, c2);

    delay_ms(20);

    uint16_t readback_c1 = readRegister(CONTROL_1);
    uint16_t readback_c2 = readRegister(CONTROL_2);

    if (readback_c1 == 0xFFFF || readback_c2 == 0xFFFF)
        return false;

    if (hasFault())
        return false;

    return true;
}

void DRV8301::disable()
{
    gpio_clear(GPIOB, GPIO12);
}

bool DRV8301::hasFault() const
{
    return gpio_get(GPIOD, GPIO2) == 0;
}

Error DRV8301::updateFault()
{
    if (gpio_get(en_port_, en_pin_) == 0)
    {
        fault_low_count_ = 0;
        return Error::ERROR_NONE;
    }

    if (!hasFault())
    {
        if (fault_low_count_ > 0)
            fault_flicker_count_++;

        fault_low_count_ = 0;
        return Error::ERROR_NONE;
    }

    fault_low_count_++;
    if (fault_low_count_ < kFaultDebounceCount)
        return Error::ERROR_NONE;

    uint16_t s1 = readRegister(STATUS_1);
    uint16_t s2 = readRegister(STATUS_2);

    dbg_s1 = s1;
    dbg_s2 = s2;

    if (s1 == 0xFFFF)
    {
        return Error::ERROR_DRV_FAULT;
    }

    cdcLog(
        "[DRV] fault pin active s1=0x%03X s2=0x%03X | s1{b0=%u b1=%u b2=%u b3=%u b4=%u b5=%u b6=%u b7=%u b8=%u b9=%u b10=%u}\r\n",
        s1,
        s2,
        (s1 >> 0) & 1,
        (s1 >> 1) & 1,
        (s1 >> 2) & 1,
        (s1 >> 3) & 1,
        (s1 >> 4) & 1,
        (s1 >> 5) & 1,
        (s1 >> 6) & 1,
        (s1 >> 7) & 1,
        (s1 >> 8) & 1,
        (s1 >> 9) & 1,
        (s1 >> 10) & 1);

    if (s1 & (1 << 7))
        return Error::ERROR_DRV_OTS;

    if (s1 & 0b00111111)
        return Error::ERROR_DRV_OC;

    if ((s1 & (1 << 9)) || (s1 & (1 << 8)))
        return Error::ERROR_DRV_GATE_FAULT;

    if (s2 != 0xFFFF && (s2 & (1 << 0)))
        return Error::ERROR_DRV_GATE_FAULT;

    if (s1 & (1 << 10))
        return Error::ERROR_DRV_FAULT;

    return Error::ERROR_DRV_FAULT;
}

uint16_t DRV8301::getStatus1()
{
    return readRegister(STATUS_1);
}

uint16_t DRV8301::getStatus2()
{
    return readRegister(STATUS_2);
}

uint16_t DRV8301::getControl1()
{
    return readRegister(CONTROL_1);
}

uint16_t DRV8301::getControl2()
{
    return readRegister(CONTROL_2);
}

uint16_t DRV8301::readRegister(Register reg)
{
    uint16_t cmd = (1 << 15) | ((reg & 0x0F) << 11);

    spiTransfer(cmd);
    delay_us(1);
    uint16_t val = spiTransfer(cmd) & 0x7FF;

    if (val == 0x7FF)
        return 0xFFFF;

    return val;
}

void DRV8301::writeRegister(Register reg, uint16_t val)
{
    uint16_t control_word = ((reg & 0x0F) << 11) | (val & 0x7FF);
    spiTransfer(control_word);
}

uint16_t DRV8301::spiTransfer(uint16_t data)
{
    while (SPI_SR(spi_) & SPI_SR_RXNE)
    {
        (void)SPI_DR(spi_);
    }

    select();
    spi_send(spi_, data);

    while (!(SPI_SR(spi_) & SPI_SR_RXNE))
        ;
    uint16_t result = spi_read(spi_);
    deselect();

    return result;
}

void DRV8301::select()
{
    gpio_clear(GPIOC, GPIO13);
}

void DRV8301::deselect()
{
    gpio_set(GPIOC, GPIO13);
}