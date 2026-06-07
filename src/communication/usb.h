#ifndef USB_H
#define USB_H

#include <cstdint>

constexpr uint8_t JOYSTICK_ITF = 0;
constexpr uint8_t JOYSTICK_REPORT_ID = 1;

#ifndef DEBUG
constexpr uint8_t VENDOR_ITF = 1;
constexpr uint8_t VENDOR_TELEMETRY_REPORT_ID = 2;
#endif

struct __attribute__((packed)) JoystickReport_Data
{
    uint8_t buttons[8];
    int16_t x_axis;
};

class USB
{
public:
    USB();

    void init();
    void update();

    void updateAxis(int32_t positionCounts);

private:
    JoystickReport_Data joystickReport_{};

    int16_t scale_position_to_axis_(int32_t positionCounts) const;
};

#endif // USB_H