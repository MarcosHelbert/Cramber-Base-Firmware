#ifndef USB_H
#define USB_H

#include <cstdint>

constexpr uint8_t JOYSTICK_ITF = 0;
constexpr uint8_t JOYSTICK_REPORT_ID = 1;

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
    void start();
    void update();

    void setAxis(float position_rad, float steering_range_deg);

private:
    JoystickReport_Data joystickReport_{};
};

#endif // USB_H
