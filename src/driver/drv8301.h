#ifndef DRV8301_H
#define DRV8301_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#include <cstdint>
#include <error.h>
#include <persistence/config_store.h>

class DRV8301
{
public:
    enum Register
    {
        STATUS_1 = 0x00,
        STATUS_2 = 0x01,
        CONTROL_1 = 0x02,
        CONTROL_2 = 0x03
    };

    explicit DRV8301(const PersistedConfig &config) : config_(config) {}

    void init();

    bool enable();
    void disable();

    bool hasFault() const;

    Error updateFault();

    uint16_t getStatus1();
    uint16_t getStatus2();

    uint16_t getControl1();
    uint16_t getControl2();

    uint32_t getFaultFlickerCount() const { return fault_flicker_count_; }

private:
    uint16_t readRegister(Register reg);
    void writeRegister(Register reg, uint16_t val);

    uint16_t spiTransfer(uint16_t data);

    void select();
    void deselect();

private:
    const PersistedConfig &config_;

    uint32_t spi_ = SPI3;

    uint32_t fault_low_count_ = 0;
    uint32_t fault_flicker_count_ = 0;
    static constexpr uint32_t kFaultDebounceCount = 100;

    static constexpr uint32_t ncs_port_ = GPIOC;
    static constexpr uint16_t ncs_pin_ = GPIO13;

    static constexpr uint32_t en_port_ = GPIOB;
    static constexpr uint16_t en_pin_ = GPIO12;

    static constexpr uint32_t fault_port_ = GPIOD;
    static constexpr uint16_t fault_pin_ = GPIO2;
};

#endif // DRV8301_H
