#ifndef DRV8301_H
#define DRV8301_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#include <cstdint>
#include <error.h>

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

    struct Status1
    {
        bool fault : 1;
        bool gvref_uv : 1;
        bool pvd_uv : 1;
        bool ots : 1;
        bool oc : 1;
        bool gate_drv_fault : 1;
        bool low_vbus : 1;
        bool ot_warning : 1;
        uint8_t device_id : 3;
    };

    DRV8301();

    void init();
    void enable();
    void disable();

    inline bool hasFault() const;

    Error updateFault();

    uint16_t getStatus1();
    uint16_t getStatus2();

private:
    uint16_t readRegister(Register reg);
    void writeRegister(Register reg, uint16_t val);

    uint16_t spiTransfer(uint16_t data);

    void select();
    void deselect();

private:
    uint32_t spi_ = SPI3;

    uint8_t error_counter_ = 0;

    static constexpr uint32_t ncs_port_ = GPIOC;
    static constexpr uint16_t ncs_pin_ = GPIO13;

    static constexpr uint32_t en_port_ = GPIOB;
    static constexpr uint16_t en_pin_ = GPIO12;

    static constexpr uint32_t fault_port_ = GPIOD;
    static constexpr uint16_t fault_pin_ = GPIO2;
};

#endif // DRV8301_H